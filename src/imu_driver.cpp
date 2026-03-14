#include "dm_imu/imu_driver.hpp"
#include "dm_imu/bsp_crc.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <errno.h>
#include <sys/select.h>
#include <pthread.h>
#include <sched.h>

#include <cstring>
#include <cmath>
#include <iostream>
#include <chrono>
#include <thread>

namespace dm_imu {

static constexpr float kDeg2Rad = 3.14159265358979f / 180.f;
static constexpr float kGravity = 9.81f;

// ─────────────────────────────────────────────────────────────────
//  SetRealTimeThread — set SCHED_FIFO + CPU affinity
// ─────────────────────────────────────────────────────────────────

static bool SetRealTimeThread(pthread_t pid, const std::string& name,
                               int rt_priority, int bind_cpu)
{
    bool ret = true;

    if (!name.empty())
        pthread_setname_np(pid, name.c_str());

    if (rt_priority >= 0) {
        int max = sched_get_priority_max(SCHED_FIFO);
        if (rt_priority > max) rt_priority = max;

        struct sched_param s_parm{};
        s_parm.sched_priority = rt_priority;
        if (pthread_setschedparam(pid, SCHED_FIFO, &s_parm) < 0) {
            std::cerr << "[dm_imu] setschedparam: " << std::strerror(errno)
                      << " (run with sudo or CAP_SYS_NICE)\n";
            ret = false;
        } else {
            std::cout << "[dm_imu] " << name << " SCHED_FIFO prio=" << rt_priority << "\n";
        }
    }

    if (bind_cpu >= 0) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(bind_cpu, &cpuset);
        if (pthread_setaffinity_np(pid, sizeof(cpuset), &cpuset) != 0) {
            std::cerr << "[dm_imu] setaffinity: " << std::strerror(errno) << "\n";
            ret = false;
        } else {
            std::cout << "[dm_imu] " << name << " pinned to cpu=" << bind_cpu << "\n";
        }
    }

    return ret;
}

// ─────────────────────────────────────────────────────────────────

static inline float f32le(const uint8_t* p) noexcept {
    float v; __builtin_memcpy(&v, p, 4); return v;
}

uint64_t ImuDriver::monotonicNs() noexcept {
    struct timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL
         + static_cast<uint64_t>(ts.tv_nsec);
}

static speed_t baudToTermios(int baud) noexcept {
    switch (baud) {
        case 9600:    return B9600;
        case 19200:   return B19200;
        case 115200:  return B115200;
        case 230400:  return B230400;
        case 460800:  return B460800;
        case 921600:  return B921600;
        case 1000000: return B1000000;
        case 1500000: return B1500000;
        case 2000000: return B2000000;
        default:
            std::cerr << "[dm_imu] Unknown baud " << baud << ", using 921600\n";
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
}

ImuDriver::~ImuDriver() { close(); }

// ═══════════════════════════════════════════════════════════════════
//  Lifecycle
// ═══════════════════════════════════════════════════════════════════

bool ImuDriver::open(bool configure_imu,
                     const std::string& rt_name,
                     int rt_priority,
                     int bind_cpu)
{
    if (!openSerial()) return false;

    stop_flag_.store(false, std::memory_order_relaxed);
    reader_thread_ = std::thread(&ImuDriver::readerLoop, this);

    SetRealTimeThread(reader_thread_.native_handle(),
                      rt_name, rt_priority, bind_cpu);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (configure_imu) configureForLocomotion();

    return true;
}

void ImuDriver::close()
{
    stop_flag_.store(true, std::memory_order_relaxed);
    if (reader_thread_.joinable()) reader_thread_.join();
    closeSerial();
}

bool ImuDriver::isOpen() const { return fd_ >= 0; }

void ImuDriver::setObsCallback(ObsCallback cb, uint32_t every_n)
{
    obs_cb_     = std::move(cb);
    cb_every_n_ = (every_n < 1) ? 1 : every_n;
}

// ═══════════════════════════════════════════════════════════════════
//  Seqlock
// ═══════════════════════════════════════════════════════════════════

static inline void cpu_pause() noexcept {
#if defined(__x86_64__) || defined(__i386__)
    __builtin_ia32_pause();
#elif defined(__aarch64__) || defined(__arm__)
    asm volatile("yield" ::: "memory");
#else
    asm volatile("" ::: "memory");
#endif
}

void ImuDriver::commitTick() noexcept
{
    ImuObservation next{};
    next.gyr_x = pending_.gyr_x;
    next.gyr_y = pending_.gyr_y;
    next.gyr_z = pending_.gyr_z;
    next.acc_x = pending_.acc_x;
    next.acc_y = pending_.acc_y;
    next.acc_z = pending_.acc_z;
    next.qw    = pending_.qw;
    next.qx    = pending_.qx;
    next.qy    = pending_.qy;
    next.qz    = pending_.qz;
    next.timestamp_ns = monotonicNs();
    next.seq   = ++seq_counter_;
    next.valid = 1;

    // Seqlock write — minimal fences:
    //   odd(relaxed)  : start write, no ordering needed (readers skip odd)
    //   obs_ = next   : single-writer thread, in-order execution guaranteed
    //   even(release) : publish — ensures obs_ visible before version becomes even
    obs_ver_.fetch_add(1, std::memory_order_relaxed);   // → odd
    obs_ = next;
    obs_ver_.fetch_add(1, std::memory_order_release);   // → even

    ++tick_counter_;
    stat_ticks_total_.fetch_add(1, std::memory_order_relaxed);

    if (obs_cb_ && (tick_counter_ % cb_every_n_ == 0)) {
        obs_cb_(obs_);
        stat_ticks_pub_.fetch_add(1, std::memory_order_relaxed);
    }

    if (fall_cb_) {
        bool fallen = (std::fabs(next.pitch_rad()) > LocomotionState::kFallThreshRad ||
                       std::fabs(next.roll_rad())  > LocomotionState::kFallThreshRad);
        if (fallen && !prev_fallen_) fall_cb_();
        prev_fallen_ = fallen;
    }

    pending_.has_accel = pending_.has_gyro = pending_.has_quat = false;
}

bool ImuDriver::getObs(ImuObservation& out) const noexcept
{
    // Seqlock read — 2 sync ops (minimal correct for ARM64 + x86):
    //
    //   v0 = load(acquire)  ARM64: ldar
    //     → acquire semantic: memcpy cannot be speculated before v0 load ✓
    //     → first fence(acquire) in old code was REDUNDANT
    //
    //   memcpy(64B)         ARM64: 4× ldp (plain)
    //
    //   fence(acquire)      ARM64: dmb ishld
    //     → prevents v1 load from being hoisted above memcpy ✓
    //
    //   v1 = load(relaxed)  ARM64: ldr  (cheaper than ldar — fence above handles ordering)
    //     → second load(acquire) in old code was REDUNDANT given preceding fence
    //
    // x86: both ldar and fence compile to nothing (TSO), so this is already free on x86.
    uint32_t v0, v1 = 0;
    do {
        v0 = obs_ver_.load(std::memory_order_acquire);
        if (__builtin_expect(v0 & 1u, 0)) { cpu_pause(); continue; }
        __builtin_memcpy(&out, &obs_, sizeof(ImuObservation));
        std::atomic_thread_fence(std::memory_order_acquire);
        v1 = obs_ver_.load(std::memory_order_relaxed);
    } while (__builtin_expect(v0 != v1, 0));
    return out.valid;
}

ImuObservation ImuDriver::getObs() const noexcept
{
    ImuObservation out{};
    getObs(out);
    return out;
}

// ═══════════════════════════════════════════════════════════════════
//  LocomotionState
// ═══════════════════════════════════════════════════════════════════

LocomotionState ImuDriver::getLocomotionState(float fall_thresh_rad,
                                               float impact_thresh_g,
                                               float stationary_rps) const noexcept
{
    ImuObservation obs{};
    getObs(obs);

    LocomotionState s{};
    if (!obs.valid) return s;

    s.pitch_rad  = obs.pitch_rad();
    s.roll_rad   = obs.roll_rad();
    s.yaw_rad    = obs.yaw_rad();
    s.pitch_rate = obs.gyr_y;
    s.roll_rate  = obs.gyr_x;
    s.yaw_rate   = obs.gyr_z;

    s.accel_norm_g     = obs.accel_norm() / kGravity;
    s.accel_vertical_g = (obs.acc_z * std::cos(s.pitch_rad)
                        + obs.acc_x * std::sin(s.pitch_rad)) / kGravity;

    s.is_fallen      = (std::fabs(s.pitch_rad) > fall_thresh_rad ||
                        std::fabs(s.roll_rad)  > fall_thresh_rad);
    s.impact_detected = (s.accel_norm_g > impact_thresh_g);
    s.is_stationary   = (std::max({std::fabs(obs.gyr_x),
                                   std::fabs(obs.gyr_y),
                                   std::fabs(obs.gyr_z)}) < stationary_rps);
    s.timestamp_ns    = obs.timestamp_ns;
    return s;
}

float ImuDriver::pitchRad()  const noexcept { return getObs().pitch_rad(); }
float ImuDriver::rollRad()   const noexcept { return getObs().roll_rad();  }
float ImuDriver::yawRad()    const noexcept { return getObs().yaw_rad();   }
float ImuDriver::pitchRate() const noexcept { return getObs().gyr_y;       }
float ImuDriver::rollRate()  const noexcept { return getObs().gyr_x;       }
float ImuDriver::yawRate()   const noexcept { return getObs().gyr_z;       }

bool ImuDriver::isFallen(float t) const noexcept {
    auto o = getObs();
    return std::fabs(o.pitch_rad()) > t || std::fabs(o.roll_rad()) > t;
}

ImuDriver::Stats ImuDriver::getStats() const noexcept {
    return { stat_ticks_total_.load(std::memory_order_relaxed),
             stat_ticks_pub_  .load(std::memory_order_relaxed),
             stat_crc_err_    .load(std::memory_order_relaxed),
             stat_dropped_    .load(std::memory_order_relaxed) };
}

// ═══════════════════════════════════════════════════════════════════
//  Serial
// ═══════════════════════════════════════════════════════════════════

bool ImuDriver::openSerial()
{
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::cerr << "[dm_imu] open " << port_ << ": " << std::strerror(errno) << "\n";
        return false;
    }
    struct termios tty{};
    ::tcgetattr(fd_, &tty);
    speed_t sp = baudToTermios(baud_);
    ::cfsetispeed(&tty, sp);
    ::cfsetospeed(&tty, sp);
    tty.c_cflag  = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_iflag  = 0; tty.c_oflag = 0; tty.c_lflag = 0;
    tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 0;
    ::tcsetattr(fd_, TCSANOW, &tty);
    ::tcflush(fd_, TCIFLUSH);
    std::cout << "[dm_imu] " << port_ << " @ " << baud_ << "\n";
    return true;
}

void ImuDriver::closeSerial() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

void ImuDriver::writeBytes(const uint8_t* buf, size_t len) {
    if (fd_ >= 0) { ssize_t r = ::write(fd_, buf, len); (void)r; }
}

void ImuDriver::sendCmd(const uint8_t* cmd, size_t len, int n, int ms) {
    for (int i = 0; i < n; ++i) {
        writeBytes(cmd, len);
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }
}

// ═══════════════════════════════════════════════════════════════════
//  Reader thread
// ═══════════════════════════════════════════════════════════════════

void ImuDriver::readerLoop()
{
    constexpr int CHUNK = 512;
    uint8_t tmp[CHUNK];

    while (!stop_flag_.load(std::memory_order_relaxed)) {
        if (fd_ < 0) { std::this_thread::sleep_for(std::chrono::milliseconds(10)); continue; }

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd_, &rfds);
        struct timeval tv;
        tv.tv_sec  = 0;
        tv.tv_usec = 2000;   // 2ms — wake ngay khi USB có data
        if (::select(fd_ + 1, &rfds, nullptr, nullptr, &tv) <= 0) continue;

        ssize_t n = ::read(fd_, tmp, CHUNK);
        if (__builtin_expect(n > 0, 1)) {
            buf_.insert(buf_.end(), tmp, tmp + n);
            processBuffer();
        }
    }
}

// ═══════════════════════════════════════════════════════════════════
//  Frame parser — EKF-only: only commit on RID_QUAT
// ═══════════════════════════════════════════════════════════════════

uint16_t ImuDriver::frameLen(uint8_t rid) noexcept {
    return (rid == RID_QUAT) ? FRAME_LEN_QUAT : FRAME_LEN_STD;
}

void ImuDriver::processBuffer()
{
    const uint8_t* data = buf_.data();
    size_t pos  = 0;
    const size_t size = buf_.size();

    while (pos + 4 <= size) {
        if (__builtin_expect(data[pos] != FRAME_HDR0 || data[pos+1] != FRAME_HDR1, 0)) {
            ++pos; continue;
        }
        uint8_t  rid  = data[pos + 3];
        uint16_t flen = frameLen(rid);
        if (pos + flen > size) break;

        if (__builtin_expect(parseFrame(data + pos, flen), 1))
            pos += flen;
        else {
            stat_dropped_.fetch_add(1, std::memory_order_relaxed);
            ++pos;
        }
    }

    if (pos > 0)
        buf_.erase(buf_.begin(), buf_.begin() + static_cast<ptrdiff_t>(pos));

    if (__builtin_expect(buf_.size() > 4096, 0)) {
        buf_.clear();
        pending_.has_accel = pending_.has_gyro = pending_.has_quat = false;
    }
}

bool ImuDriver::parseFrame(const uint8_t* frame, uint16_t flen) noexcept
{
    if (__builtin_expect(frame[flen-1] != FRAME_TAIL, 0)) return false;

    uint8_t rid = frame[3];

    // CRC
    uint16_t crc_len  = static_cast<uint16_t>(flen - 3u);
    uint16_t crc_calc = Get_CRC16(frame, crc_len);
    uint16_t crc_wire = static_cast<uint16_t>(frame[flen-3])
                      | (static_cast<uint16_t>(frame[flen-2]) << 8);
    if (__builtin_expect(crc_calc != crc_wire, 0)) {
        if (Get_CRC16(frame+2, static_cast<uint16_t>(crc_len-2u)) != crc_wire) {
            stat_crc_err_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
    }

    const float f1 = f32le(frame+4);
    const float f2 = f32le(frame+8);
    const float f3 = f32le(frame+12);

    switch (rid) {
        case RID_ACCEL:
            pending_.acc_x = f1; pending_.acc_y = f2; pending_.acc_z = f3;
            pending_.has_accel = true;
            break;

        case RID_GYRO:
            pending_.gyr_x = f1 * kDeg2Rad;
            pending_.gyr_y = f2 * kDeg2Rad;
            pending_.gyr_z = f3 * kDeg2Rad;
            pending_.has_gyro = true;
            break;

        case RID_EULER:
            // EKF always present — Euler frame accepted for sync but not used
            break;

        case RID_QUAT: {
            const float f4 = f32le(frame+16);
            float n2 = f1*f1 + f2*f2 + f3*f3 + f4*f4;
            if (__builtin_expect(n2 > 1e-6f, 1)) {
                float inv = 1.f / std::sqrt(n2);
                pending_.qw = f1*inv; pending_.qx = f2*inv;
                pending_.qy = f3*inv; pending_.qz = f4*inv;
            } else {
                pending_.qw = 1.f; pending_.qx = pending_.qy = pending_.qz = 0.f;
            }
            pending_.has_quat = true;
            // RID_QUAT is last frame of each tick — commit immediately
            if (__builtin_expect(pending_.has_accel && pending_.has_gyro, 1))
                commitTick();
            break;
        }

        default:
            return false;
    }

    return true;
}

// ═══════════════════════════════════════════════════════════════════
//  IMU Commands
// ═══════════════════════════════════════════════════════════════════

void ImuDriver::enterSettingMode()  { uint8_t c[]={0xAA,0x06,0x01,0x0D}; sendCmd(c,4,5,15); }
void ImuDriver::exitSettingMode()   { uint8_t c[]={0xAA,0x06,0x00,0x0D}; sendCmd(c,4,5,15); }
void ImuDriver::turnOnAccel()       { uint8_t c[]={0xAA,0x01,0x14,0x0D}; sendCmd(c,4); }
void ImuDriver::turnOffAccel()      { uint8_t c[]={0xAA,0x01,0x04,0x0D}; sendCmd(c,4); }
void ImuDriver::turnOnGyro()        { uint8_t c[]={0xAA,0x01,0x15,0x0D}; sendCmd(c,4); }
void ImuDriver::turnOffGyro()       { uint8_t c[]={0xAA,0x01,0x05,0x0D}; sendCmd(c,4); }
void ImuDriver::turnOnEuler()       { uint8_t c[]={0xAA,0x01,0x16,0x0D}; sendCmd(c,4); }
void ImuDriver::turnOffEuler()      { uint8_t c[]={0xAA,0x01,0x06,0x0D}; sendCmd(c,4); }
void ImuDriver::turnOnQuat()        { uint8_t c[]={0xAA,0x01,0x17,0x0D}; sendCmd(c,4); }
void ImuDriver::turnOffQuat()       { uint8_t c[]={0xAA,0x01,0x07,0x0D}; sendCmd(c,4); }
void ImuDriver::enableTempControl() { uint8_t c[]={0xAA,0x04,0x01,0x0D}; sendCmd(c,4); }
void ImuDriver::disableTempControl(){ uint8_t c[]={0xAA,0x04,0x00,0x0D}; sendCmd(c,4); }
void ImuDriver::setTargetTemp(uint8_t t){ uint8_t c[]={0xAA,0x05,t,0x0D}; sendCmd(c,4); }
void ImuDriver::saveParams()        { uint8_t c[]={0xAA,0x03,0x01,0x0D}; sendCmd(c,4); }
void ImuDriver::zeroAngle()         { uint8_t c[]={0xAA,0x0C,0x01,0x0D}; sendCmd(c,4); }
void ImuDriver::restartImu()        { uint8_t c[]={0xAA,0x00,0x00,0x0D}; sendCmd(c,3,10); }
void ImuDriver::calibrateGyro()     { uint8_t c[]={0xAA,0x03,0x02,0x0D}; sendCmd(c,4); }
void ImuDriver::calibrateAccel6Face(){uint8_t c[]={0xAA,0x03,0x03,0x0D}; sendCmd(c,4); }

void ImuDriver::setOutputHz(int hz) {
    if (hz < 1)    hz = 1;
    if (hz > 1000) hz = 1000;
    uint16_t iv = static_cast<uint16_t>(1000 / hz);
    uint8_t c[] = {0xAA, 0x02,
                   static_cast<uint8_t>(iv & 0xFF),
                   static_cast<uint8_t>((iv >> 8) & 0xFF), 0x0D};
    sendCmd(c, sizeof(c));
}
void ImuDriver::setOutput1000Hz() { setOutputHz(1000); }

void ImuDriver::configureForLocomotion(int hz, uint8_t temp_c)
{
    auto ms = [](int n){ std::this_thread::sleep_for(std::chrono::milliseconds(n)); };
    std::cout << "[dm_imu] Configuring hz=" << hz << "\n";
    { uint8_t c[]={0xAA,0x06,0x01,0x0D}; sendCmd(c,4,5,15); } ms(100);
    turnOnAccel();  ms(20); turnOnGyro();  ms(20);
    turnOnEuler();  ms(20); turnOnQuat();  ms(20);
    setOutputHz(hz); ms(30);
    if (temp_c > 0) { enableTempControl(); ms(20); setTargetTemp(temp_c); ms(20); }
    saveParams(); ms(80);
    { uint8_t c[]={0xAA,0x06,0x00,0x0D}; sendCmd(c,4,5,15); }
    ms(400);
    std::cout << "[dm_imu] Config done.\n";
}

} // namespace dm_imu