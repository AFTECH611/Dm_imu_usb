#include "dm_imu/imu_driver.hpp"
#include "dm_imu/bsp_crc.hpp"

// POSIX / Linux serial
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <errno.h>

#include <cstring>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <thread>

namespace dm_imu {

static constexpr float DEG2RAD = static_cast<float>(M_PI / 180.0);
static constexpr float GRAVITY  = 9.81f;

// ═══════════════════════════════════════════════════════════════════
//  Helpers nội bộ
// ═══════════════════════════════════════════════════════════════════

static float readFloat32LE(const uint8_t* p)
{
    float v;
    std::memcpy(&v, p, 4);
    return v;
}

static speed_t toTermiosBaud(int baud)
{
    switch (baud) {
        case 9600:    return B9600;
        case 19200:   return B19200;
        case 38400:   return B38400;
        case 57600:   return B57600;
        case 115200:  return B115200;
        case 230400:  return B230400;
        case 460800:  return B460800;
        case 500000:  return B500000;
        case 576000:  return B576000;
        case 921600:  return B921600;
        case 1000000: return B1000000;
        case 1152000: return B1152000;
        case 1500000: return B1500000;
        case 2000000: return B2000000;
        default:
            std::cerr << "[dm_imu] Unsupported baud " << baud << ", fallback 921600\n";
            return B921600;
    }
}

// ═══════════════════════════════════════════════════════════════════
//  Constructor / Destructor
// ═══════════════════════════════════════════════════════════════════

ImuDriver::ImuDriver(const std::string& port, int baud)
    : port_(port), baud_(baud)
{
    buf_.reserve(4096);
    last_loco_ts_ = std::chrono::steady_clock::now();
}

ImuDriver::~ImuDriver()
{
    close();
}

// ═══════════════════════════════════════════════════════════════════
//  Lifecycle
// ═══════════════════════════════════════════════════════════════════

bool ImuDriver::open(bool configure_imu)
{
    if (!openSerial()) return false;

    // ── Start reader thread TRƯỚC khi gửi bất kỳ lệnh nào ─────────
    // IMU có thể đang stream ngay sau khi cổng mở.
    // Nếu thread chưa chạy mà ta gửi config → data mất vào void.
    stop_flag_.store(false);
    reader_thread_ = std::thread(&ImuDriver::readerLoop, this);

    // Flush dữ liệu cũ trong UART buffer, chờ thread ổn định
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    if (configure_imu) {
        configureForLocomotion();
    }

    return true;
}

void ImuDriver::close()
{
    stop_flag_.store(true);
    if (reader_thread_.joinable()) reader_thread_.join();
    closeSerial();
}

bool ImuDriver::isOpen() const { return fd_ >= 0; }

// ═══════════════════════════════════════════════════════════════════
//  Core API
// ═══════════════════════════════════════════════════════════════════

ImuObservation ImuDriver::getObs() const
{
    std::lock_guard<std::mutex> lk(obs_mutex_);
    return latest_obs_;
}

ImuDriver::Stats ImuDriver::getStats() const
{
    std::lock_guard<std::mutex> lk(stats_mutex_);
    return stats_;
}

// ═══════════════════════════════════════════════════════════════════
//  LocomotionState computation
// ═══════════════════════════════════════════════════════════════════

LocomotionState ImuDriver::getLocomotionState(float fall_thresh_rad,
                                               float impact_thresh_g,
                                               float stationary_dps) const
{
    ImuObservation obs = getObs();
    auto now = std::chrono::steady_clock::now();

    LocomotionState s;
    s.timestamp = now;

    // ── dt ────────────────────────────────────────────────────────
    float dt = std::chrono::duration<float>(now - last_loco_ts_).count();
    s.dt_s = (dt > 0.f && dt < 1.f) ? dt : 0.f;
    last_loco_ts_ = now;

    if (!obs.valid) return s;

    // ── Balance angles (rad) ──────────────────────────────────────
    s.pitch_rad = obs.pitch * DEG2RAD;
    s.roll_rad  = obs.roll  * DEG2RAD;
    s.yaw_rad   = obs.yaw   * DEG2RAD;

    // ── Angular rates (rad/s) ─────────────────────────────────────
    // Gyro output: x=roll-rate, y=pitch-rate, z=yaw-rate (°/s)
    s.roll_rate  = obs.angular_velocity.x * DEG2RAD;
    s.pitch_rate = obs.angular_velocity.y * DEG2RAD;
    s.yaw_rate   = obs.angular_velocity.z * DEG2RAD;

    // ── Accel derived ─────────────────────────────────────────────
    float accel_norm = obs.accel.norm();
    s.accel_norm_g     = accel_norm / GRAVITY;
    // Thành phần gia tốc theo trục Z body (chuyển đổi qua quaternion)
    // Ước lượng đơn giản từ pitch: a_vert ≈ az*cos(p) + ax*sin(p)
    s.accel_vertical_g = (obs.accel.z * std::cos(s.pitch_rad)
                        + obs.accel.x * std::sin(s.pitch_rad)) / GRAVITY;

    // ── Fall detection ────────────────────────────────────────────
    s.is_fallen = (std::fabs(s.pitch_rad) > fall_thresh_rad ||
                   std::fabs(s.roll_rad)  > fall_thresh_rad);

    // ── Impact / footstrike detection ─────────────────────────────
    s.impact_detected = (s.accel_norm_g > impact_thresh_g);

    // ── Stationary detection ──────────────────────────────────────
    float gyro_max = std::max({std::fabs(obs.angular_velocity.x),
                               std::fabs(obs.angular_velocity.y),
                               std::fabs(obs.angular_velocity.z)});
    s.is_stationary = (gyro_max < stationary_dps);

    // ── Callbacks (edge-triggered) ────────────────────────────────
    if (s.is_fallen && !prev_fallen_ && fall_cb_)   fall_cb_();
    if (s.impact_detected && !prev_impact_ && impact_cb_) impact_cb_();
    prev_fallen_ = s.is_fallen;
    prev_impact_ = s.impact_detected;

    return s;
}

// ═══════════════════════════════════════════════════════════════════
//  Fast locomotion accessors
// ═══════════════════════════════════════════════════════════════════

float ImuDriver::getPitch()    const { std::lock_guard<std::mutex> l(obs_mutex_); return latest_obs_.pitch; }
float ImuDriver::getRoll()     const { std::lock_guard<std::mutex> l(obs_mutex_); return latest_obs_.roll;  }
float ImuDriver::getYaw()      const { std::lock_guard<std::mutex> l(obs_mutex_); return latest_obs_.yaw;   }
float ImuDriver::getPitchRad() const { return getPitch() * DEG2RAD; }
float ImuDriver::getRollRad()  const { return getRoll()  * DEG2RAD; }
float ImuDriver::getYawRad()   const { return getYaw()   * DEG2RAD; }

float ImuDriver::getPitchRate() const { std::lock_guard<std::mutex> l(obs_mutex_); return latest_obs_.angular_velocity.y; }
float ImuDriver::getRollRate()  const { std::lock_guard<std::mutex> l(obs_mutex_); return latest_obs_.angular_velocity.x; }
float ImuDriver::getYawRate()   const { std::lock_guard<std::mutex> l(obs_mutex_); return latest_obs_.angular_velocity.z; }

Vec3  ImuDriver::getAccel()    const { std::lock_guard<std::mutex> l(obs_mutex_); return latest_obs_.accel;            }
Vec3  ImuDriver::getGyro()     const { std::lock_guard<std::mutex> l(obs_mutex_); return latest_obs_.angular_velocity; }

float ImuDriver::getAccelNormG() const
{
    std::lock_guard<std::mutex> l(obs_mutex_);
    return latest_obs_.accel.norm() / GRAVITY;
}

bool ImuDriver::isStationary(float thresh_dps) const
{
    std::lock_guard<std::mutex> l(obs_mutex_);
    const auto& g = latest_obs_.angular_velocity;
    return std::max({std::fabs(g.x), std::fabs(g.y), std::fabs(g.z)}) < thresh_dps;
}

bool ImuDriver::isFallen(float thresh_rad) const
{
    std::lock_guard<std::mutex> l(obs_mutex_);
    return (std::fabs(latest_obs_.pitch * DEG2RAD) > thresh_rad ||
            std::fabs(latest_obs_.roll  * DEG2RAD) > thresh_rad);
}

// ═══════════════════════════════════════════════════════════════════
//  Serial open / close  (POSIX termios — Linux USB CDC)
// ═══════════════════════════════════════════════════════════════════

bool ImuDriver::openSerial()
{
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::cerr << "[dm_imu] Cannot open " << port_
                  << ": " << std::strerror(errno) << "\n";
        return false;
    }

    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));
    if (::tcgetattr(fd_, &tty) != 0) {
        std::cerr << "[dm_imu] tcgetattr: " << std::strerror(errno) << "\n";
        ::close(fd_); fd_ = -1; return false;
    }

    speed_t sp = toTermiosBaud(baud_);
    ::cfsetispeed(&tty, sp);
    ::cfsetospeed(&tty, sp);

    tty.c_cflag  = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_iflag  = 0;
    tty.c_oflag  = 0;
    tty.c_lflag  = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
        std::cerr << "[dm_imu] tcsetattr: " << std::strerror(errno) << "\n";
        ::close(fd_); fd_ = -1; return false;
    }
    ::tcflush(fd_, TCIFLUSH);
    std::cout << "[dm_imu] Opened " << port_ << " @ " << baud_ << "\n";
    return true;
}

void ImuDriver::closeSerial()
{
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

// ═══════════════════════════════════════════════════════════════════
//  Serial write
// ═══════════════════════════════════════════════════════════════════

void ImuDriver::writeBytes(const uint8_t* buf, size_t len)
{
    if (fd_ < 0) return;
    if (::write(fd_, buf, len) < 0)
        std::cerr << "[dm_imu] write error: " << std::strerror(errno) << "\n";
}

/** Gửi command cmd (len byte) tổng cộng repeat lần, mỗi lần cách nhau delay_ms */
void ImuDriver::sendCmd(const uint8_t* cmd, size_t len, int repeat, int delay_ms)
{
    for (int i = 0; i < repeat; ++i) {
        writeBytes(cmd, len);
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
}

// ═══════════════════════════════════════════════════════════════════
//  Background reader
// ═══════════════════════════════════════════════════════════════════

void ImuDriver::readerLoop()
{
    constexpr int CHUNK = 256;
    uint8_t tmp[CHUNK];

    uint64_t total_bytes  = 0;
    uint64_t last_log_bytes = 0;
    auto     last_log_ts  = std::chrono::steady_clock::now();

    while (!stop_flag_.load(std::memory_order_relaxed)) {
        if (fd_ < 0) { std::this_thread::sleep_for(std::chrono::milliseconds(10)); continue; }

        ssize_t n = ::read(fd_, tmp, CHUNK);
        if (n > 0) {
            total_bytes += static_cast<uint64_t>(n);
            buf_.insert(buf_.end(), tmp, tmp + n);
            processBuffer();

            // Log throughput mỗi 3 giây để dễ debug
            auto now = std::chrono::steady_clock::now();
            float elapsed = std::chrono::duration<float>(now - last_log_ts).count();
            if (elapsed >= 3.0f) {
                uint64_t delta = total_bytes - last_log_bytes;
                auto& st = stats_;  // stats_ chỉ đọc ở đây, không cần lock cho log
                std::cout << "[dm_imu] RX " << delta << " bytes/3s"
                          << "  frames_ok=" << st.frames_ok
                          << "  crc_err=" << st.frames_crc_err << "\n";
                last_log_bytes = total_bytes;
                last_log_ts    = now;
            }
        } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            std::cerr << "[dm_imu] read: " << std::strerror(errno) << "\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } else {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
    }
}

// ═══════════════════════════════════════════════════════════════════
//  Frame processing — variable-length (19 or 23 bytes)
// ═══════════════════════════════════════════════════════════════════

void ImuDriver::processBuffer()
{
    size_t pos  = 0;
    size_t size = buf_.size();

    while (pos + 4 <= size) {
        if (buf_[pos] != FRAME_HDR0 || buf_[pos + 1] != FRAME_HDR1) { ++pos; continue; }

        uint8_t  rid  = buf_[pos + 3];
        uint16_t flen = frameLen(rid);

        if (pos + flen > size) break;

        if (parseFrame(buf_.data() + pos, flen)) {
            pos += flen;
        } else {
            ++pos;
        }
    }

    if (pos > 0) buf_.erase(buf_.begin(), buf_.begin() + static_cast<ptrdiff_t>(pos));
    if (buf_.size() > 8192) { buf_.clear(); }
}

uint16_t ImuDriver::frameLen(uint8_t rid)
{
    return (rid == RID_QUAT) ? FRAME_LEN_QUAT : FRAME_LEN_STD;
}

// ═══════════════════════════════════════════════════════════════════
//  Frame layout
//   Standard (RID 01/02/03)  — 19 bytes:
//     [0..1]=HDR [2]=ID [3]=RID [4..15]=3×f32LE [16..17]=CRC16LE [18]=0x0A
//   Quaternion (RID 04)      — 23 bytes:
//     [0..1]=HDR [2]=ID [3]=04  [4..19]=4×f32LE [20..21]=CRC16LE [22]=0x0A
//   CRC covers bytes [0 .. flen-4]
// ═══════════════════════════════════════════════════════════════════

bool ImuDriver::parseFrame(const uint8_t* frame, uint16_t flen)
{
    if (frame[flen - 1] != FRAME_TAIL) return false;

    uint8_t rid = frame[3];
    if (rid != RID_ACCEL && rid != RID_GYRO &&
        rid != RID_EULER && rid != RID_QUAT) return false;

    uint16_t crc_len  = static_cast<uint16_t>(flen - 3u);
    uint16_t crc_calc = Get_CRC16(frame, crc_len);
    uint16_t crc_wire = static_cast<uint16_t>(frame[flen - 3]) |
                        (static_cast<uint16_t>(frame[flen - 2]) << 8);

    if (crc_calc != crc_wire) {
        std::lock_guard<std::mutex> lk(stats_mutex_);
        ++stats_.frames_crc_err;
        return false;
    }

    float f1 = readFloat32LE(frame + 4);
    float f2 = readFloat32LE(frame + 8);
    float f3 = readFloat32LE(frame + 12);
    float f4 = (flen == FRAME_LEN_QUAT) ? readFloat32LE(frame + 16) : 0.f;

    applyFrame(rid, f1, f2, f3, f4);

    { std::lock_guard<std::mutex> lk(stats_mutex_); ++stats_.frames_ok; }
    return true;
}

void ImuDriver::applyFrame(uint8_t rid, float f1, float f2, float f3, float f4)
{
    std::lock_guard<std::mutex> lk(obs_mutex_);

    switch (rid) {
        case RID_ACCEL:
            latest_obs_.accel = {f1, f2, f3};
            got_accel_ = true;
            break;

        case RID_GYRO:
            latest_obs_.angular_velocity = {f1, f2, f3};
            got_gyro_ = true;
            break;

        case RID_EULER:
            // Datasheet: f1=Roll, f2=Pitch, f3=Yaw (degrees)
            latest_obs_.roll  = f1;
            latest_obs_.pitch = f2;
            latest_obs_.yaw   = f3;
            if (!got_quat_) {
                latest_obs_.quaternion    = eulerToQuat(f1, f2, f3);
                latest_obs_.quat_from_imu = false;
            }
            latest_obs_.timestamp = std::chrono::steady_clock::now();
            got_euler_ = true;
            break;

        case RID_QUAT:
            // f1=W, f2=X, f3=Y, f4=Z — EKF quaternion, 23-byte frame
            {
                Quaternion q{f1, f2, f3, f4};
                float n = q.norm();
                if (n > 1e-6f) { q.w/=n; q.x/=n; q.y/=n; q.z/=n; }
                else           { q = {1.f,0.f,0.f,0.f}; }
                latest_obs_.quaternion    = q;
                latest_obs_.quat_from_imu = true;
            }
            latest_obs_.timestamp = std::chrono::steady_clock::now();
            got_quat_ = true;
            break;

        default: break;
    }

    if (got_accel_ && got_gyro_ && (got_euler_ || got_quat_))
        latest_obs_.valid = true;
}

// ═══════════════════════════════════════════════════════════════════
//  Euler → Quaternion  (ZYX intrinsic)
// ═══════════════════════════════════════════════════════════════════

Quaternion ImuDriver::eulerToQuat(float roll_deg, float pitch_deg, float yaw_deg)
{
    float r = roll_deg  * DEG2RAD;
    float p = pitch_deg * DEG2RAD;
    float y = yaw_deg   * DEG2RAD;

    float cy=std::cos(y*.5f), sy=std::sin(y*.5f);
    float cp=std::cos(p*.5f), sp=std::sin(p*.5f);
    float cr=std::cos(r*.5f), sr=std::sin(r*.5f);

    Quaternion q;
    q.w =  cr*cp*cy + sr*sp*sy;
    q.x =  sr*cp*cy - cr*sp*sy;
    q.y =  cr*sp*cy + sr*cp*sy;
    q.z =  cr*cp*sy - sr*sp*cy;
    return q.normalized();
}

// ═══════════════════════════════════════════════════════════════════
//  IMU Commands  (tất cả lệnh từ bảng datasheet V1.2)
// ═══════════════════════════════════════════════════════════════════

// ── Mode ──────────────────────────────────────────────────────────
void ImuDriver::enterSettingMode()
{
    uint8_t c[] = {0xAA, 0x06, 0x01, 0x0D};
    sendCmd(c, sizeof(c));
}

void ImuDriver::exitSettingMode()
{
    uint8_t c[] = {0xAA, 0x06, 0x00, 0x0D};
    sendCmd(c, sizeof(c));
}

// ── Data channels ─────────────────────────────────────────────────
void ImuDriver::turnOnAccel()    { uint8_t c[]={0xAA,0x01,0x14,0x0D}; sendCmd(c,4); }
void ImuDriver::turnOffAccel()   { uint8_t c[]={0xAA,0x01,0x04,0x0D}; sendCmd(c,4); }
void ImuDriver::turnOnGyro()     { uint8_t c[]={0xAA,0x01,0x15,0x0D}; sendCmd(c,4); }
void ImuDriver::turnOffGyro()    { uint8_t c[]={0xAA,0x01,0x05,0x0D}; sendCmd(c,4); }
void ImuDriver::turnOnEuler()    { uint8_t c[]={0xAA,0x01,0x16,0x0D}; sendCmd(c,4); }
void ImuDriver::turnOffEuler()   { uint8_t c[]={0xAA,0x01,0x06,0x0D}; sendCmd(c,4); }
void ImuDriver::turnOnQuat()     { uint8_t c[]={0xAA,0x01,0x17,0x0D}; sendCmd(c,4); }
void ImuDriver::turnOffQuat()    { uint8_t c[]={0xAA,0x01,0x07,0x0D}; sendCmd(c,4); }

// ── Bus channels ──────────────────────────────────────────────────
void ImuDriver::turnOn485Active()  { uint8_t c[]={0xAA,0x01,0x13,0x0D}; sendCmd(c,4); }
void ImuDriver::turnOff485Active() { uint8_t c[]={0xAA,0x01,0x03,0x0D}; sendCmd(c,4); }
void ImuDriver::turnOnCanActive()  { uint8_t c[]={0xAA,0x01,0x18,0x0D}; sendCmd(c,4); }
void ImuDriver::turnOffCanActive() { uint8_t c[]={0xAA,0x01,0x08,0x0D}; sendCmd(c,4); }

// ── Output frequency ─────────────────────────────────────────────
// Firmware format: AA 02 [interval_lo] [interval_hi] 0D
// interval_ms = 1000 / hz
void ImuDriver::setOutputHz(int hz)
{
    if (hz < 1)    hz = 1;
    if (hz > 1000) hz = 1000;
    uint16_t interval = static_cast<uint16_t>(1000 / hz);
    uint8_t c[] = {0xAA, 0x02,
                   static_cast<uint8_t>(interval & 0xFF),
                   static_cast<uint8_t>((interval >> 8) & 0xFF),
                   0x0D};
    sendCmd(c, sizeof(c));
}

void ImuDriver::setOutput1000Hz()
{
    setOutputHz(1000);
}

// ── Output interface ─────────────────────────────────────────────
void ImuDriver::setOutputInterface(OutputInterface iface)
{
    uint8_t c[] = {0xAA, 0x0A, static_cast<uint8_t>(iface), 0x0D};
    sendCmd(c, sizeof(c));
}

// ── Temperature control ──────────────────────────────────────────
void ImuDriver::enableTempControl()
{
    uint8_t c[] = {0xAA, 0x04, 0x01, 0x0D};
    sendCmd(c, sizeof(c));
}

void ImuDriver::disableTempControl()
{
    uint8_t c[] = {0xAA, 0x04, 0x00, 0x0D};
    sendCmd(c, sizeof(c));
}

void ImuDriver::setTargetTemp(uint8_t celsius)
{
    uint8_t c[] = {0xAA, 0x05, celsius, 0x0D};
    sendCmd(c, sizeof(c));
}

// ── CAN / 485 IDs ────────────────────────────────────────────────
void ImuDriver::setCanId(uint8_t can_id)
{
    uint8_t c[] = {0xAA, 0x08, can_id, 0x0D};
    sendCmd(c, sizeof(c));
}

void ImuDriver::setMstId(uint8_t mst_id)
{
    uint8_t c[] = {0xAA, 0x09, mst_id, 0x0D};
    sendCmd(c, sizeof(c));
}

// ── Calibration ──────────────────────────────────────────────────
void ImuDriver::calibrateGyro()
{
    // Cần vào setting mode trước; IMU sẽ tự reset sau khi xong
    uint8_t c[] = {0xAA, 0x03, 0x02, 0x0D};
    sendCmd(c, sizeof(c));
}

void ImuDriver::calibrateAccel6Face()
{
    uint8_t c[] = {0xAA, 0x03, 0x03, 0x0D};
    sendCmd(c, sizeof(c));
}

// ── Persistence ──────────────────────────────────────────────────
void ImuDriver::saveParams()
{
    uint8_t c[] = {0xAA, 0x03, 0x01, 0x0D};
    sendCmd(c, sizeof(c));
}

void ImuDriver::factoryReset()
{
    uint8_t c[] = {0xAA, 0x0B, 0x01, 0x0D};
    sendCmd(c, sizeof(c));
}

// ── Special ──────────────────────────────────────────────────────
void ImuDriver::zeroAngle()
{
    // Không cần setting mode; hiệu lực ngay, không cần save
    uint8_t c[] = {0xAA, 0x0C, 0x01, 0x0D};
    sendCmd(c, sizeof(c));
}

void ImuDriver::restartImu()
{
    // Không cần setting mode
    uint8_t c[] = {0xAA, 0x00, 0x00, 0x0D};
    sendCmd(c, sizeof(c));
}

// ═══════════════════════════════════════════════════════════════════
//  High-level: configureForLocomotion
//
//  KHÔNG restart IMU — USB CDC re-enumerate phá hủy fd hiện tại.
//  Flow an toàn: enter setting → config → save → exit → chờ stream.
// ═══════════════════════════════════════════════════════════════════

void ImuDriver::configureForLocomotion(int hz, uint8_t temp_c)
{
    auto delay = [](int ms) {
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    };

    std::cout << "[dm_imu] Configuring: hz=" << hz
              << " temp=" << static_cast<int>(temp_c) << "C\n";

    // Enter setting mode
    std::cout << "[dm_imu]   Entering setting mode...\n";
    { uint8_t c[]={0xAA,0x06,0x01,0x0D}; sendCmd(c,sizeof(c),5,15); }
    delay(100);

    // Bật 4 kênh dữ liệu
    std::cout << "[dm_imu]   Enabling channels...\n";
    turnOnAccel(); delay(20);
    turnOnGyro();  delay(20);
    turnOnEuler(); delay(20);
    turnOnQuat();  delay(20);

    // Tần số
    std::cout << "[dm_imu]   Setting " << hz << " Hz...\n";
    setOutputHz(hz); delay(30);

    // Nhiệt độ (tuỳ chọn)
    if (temp_c > 0) {
        enableTempControl(); delay(20);
        setTargetTemp(temp_c); delay(20);
    }

    // Save & exit
    std::cout << "[dm_imu]   Saving and exiting...\n";
    saveParams(); delay(80);
    { uint8_t c[]={0xAA,0x06,0x00,0x0D}; sendCmd(c,sizeof(c),5,15); }

    // Chờ firmware khởi động DMA output lại
    delay(400);
    std::cout << "[dm_imu] Config done.\n";
}

} // namespace dm_imu
