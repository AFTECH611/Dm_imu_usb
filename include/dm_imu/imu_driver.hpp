#pragma once

#include "imu_types.hpp"

#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <cstdint>
#include <chrono>
#include <functional>

namespace dm_imu {

/**
 * @brief Driver DM-IMU-L1 cho humanoid robot locomotion.
 *
 * Kết nối qua USB-CDC (/dev/ttyACM0), baudrate mặc định 921600.
 * Không phụ thuộc ROS.
 *
 * ── Quick start ───────────────────────────────────────────────
 * @code
 *   dm_imu::ImuDriver imu("/dev/ttyACM0");
 *   imu.open();
 *
 *   while (running) {
 *       // Option A: raw observation
 *       auto obs   = imu.getObs();
 *
 *       // Option B: locomotion-ready state (recommended)
 *       auto state = imu.getLocomotionState();
 *       if (state.is_fallen)  handleFall();
 *       if (state.impact_detected) handleFootstrike();
 *       float pitch = state.pitch_rad;   // → balance PD/LQR
 *       float yaw_r = state.yaw_rate;    // → steering control
 *   }
 *   imu.close();
 * @endcode
 * ─────────────────────────────────────────────────────────────
 */
class ImuDriver {
public:
    // ══════════════════════════════════════════════════════════
    //  Constructor / Destructor
    // ══════════════════════════════════════════════════════════
    explicit ImuDriver(const std::string& port = "/dev/ttyACM0",
                       int                baud = 921600);
    ~ImuDriver();

    // ══════════════════════════════════════════════════════════
    //  Lifecycle
    // ══════════════════════════════════════════════════════════

    /**
     * @brief Mở cổng, cấu hình IMU, khởi động background reader thread.
     * @param configure_imu  Nếu true: tự động gửi lệnh bật accel/gyro/euler/quat
     *                       Nếu false: skip cấu hình (IMU đã được cấu hình từ trước)
     * @return true nếu thành công
     */
    bool open(bool configure_imu = true);

    /** Dừng thread và đóng cổng */
    void close();

    /** @return true nếu cổng đang mở */
    bool isOpen() const;

    // ══════════════════════════════════════════════════════════
    //  Core API
    // ══════════════════════════════════════════════════════════

    /**
     * @brief Lấy observation IMU mới nhất (thread-safe, zero-wait).
     * @return bản sao ImuObservation
     */
    ImuObservation getObs() const;

    /**
     * @brief Tính và trả về LocomotionState từ observation mới nhất.
     *
     * Bao gồm:
     *  - Chuyển đổi sang rad/s
     *  - Phát hiện ngã (fall detection)
     *  - Phát hiện tiếp đất (impact detection)
     *  - Phát hiện đứng yên (stationary detection)
     *  - dt tính từ lần gọi trước
     *
     * @param fall_thresh_rad  Ngưỡng pitch/roll để kết luận robot ngã (default ~60°)
     * @param impact_thresh_g  Ngưỡng accel norm (×g) để nhận diện footstrike
     * @param stationary_dps   Ngưỡng tốc độ góc coi là đứng yên
     * @return LocomotionState
     */
    LocomotionState getLocomotionState(
        float fall_thresh_rad = LocomotionState::kFallThreshRad,
        float impact_thresh_g = LocomotionState::kImpactThreshG,
        float stationary_dps  = LocomotionState::kStationaryDps) const;

    /**
     * @brief Đăng ký callback khi xảy ra fall (ngã).
     *
     * Callback chạy trong context của caller getLocomotionState(),
     * không phải trong reader thread.
     */
    void onFall(std::function<void()> cb) { fall_cb_ = std::move(cb); }

    /**
     * @brief Đăng ký callback khi có impact (footstrike / tiếp đất).
     */
    void onImpact(std::function<void()> cb) { impact_cb_ = std::move(cb); }

    // ══════════════════════════════════════════════════════════
    //  Locomotion helpers (fast read-only accessors)
    // ══════════════════════════════════════════════════════════

    float getPitch()    const;   ///< độ, [-90,  90]
    float getRoll()     const;   ///< độ, [-180, 180]
    float getYaw()      const;   ///< độ, [-180, 180]
    float getPitchRad() const;   ///< rad
    float getRollRad()  const;   ///< rad
    float getYawRad()   const;   ///< rad

    float getPitchRate() const;  ///< °/s — damping input
    float getRollRate()  const;  ///< °/s
    float getYawRate()   const;  ///< °/s — steering input

    Vec3  getAccel()    const;   ///< m/s²
    Vec3  getGyro()     const;   ///< °/s
    float getAccelNormG() const; ///< |accel| / 9.81 — fall / contact detection

    bool  isStationary(float thresh_dps = LocomotionState::kStationaryDps) const;
    bool  isFallen(float thresh_rad = LocomotionState::kFallThreshRad)     const;

    // ══════════════════════════════════════════════════════════
    //  Statistics
    // ══════════════════════════════════════════════════════════
    struct Stats {
        uint64_t frames_ok{0};
        uint64_t frames_crc_err{0};
        uint64_t frames_short{0};
        uint64_t frames_nohdr{0};
    };
    Stats getStats() const;

    // ══════════════════════════════════════════════════════════
    //  IMU Configuration — tất cả lệnh từ bảng datasheet V1.2
    // ══════════════════════════════════════════════════════════
    //  Lưu ý: các lệnh dưới (trừ restart & zeroAngle) đều
    //  yêu cầu vào Setting Mode trước (enterSettingMode()).
    //  Sau khi cấu hình xong: saveParams() → exitSettingMode().
    // ══════════════════════════════════════════════════════════

    // ── Mode ──────────────────────────────────────────────────
    void enterSettingMode();     ///< AA 06 01 0D
    void exitSettingMode();      ///< AA 06 00 0D

    // ── Data output channels ──────────────────────────────────
    void turnOnAccel();          ///< AA 01 14 0D
    void turnOffAccel();         ///< AA 01 04 0D
    void turnOnGyro();           ///< AA 01 15 0D
    void turnOffGyro();          ///< AA 01 05 0D
    void turnOnEuler();          ///< AA 01 16 0D
    void turnOffEuler();         ///< AA 01 06 0D
    void turnOnQuat();           ///< AA 01 17 0D — EKF quaternion (khuyến nghị)
    void turnOffQuat();          ///< AA 01 07 0D

    // ── Bus channels ──────────────────────────────────────────
    void turnOn485Active();      ///< AA 01 13 0D
    void turnOff485Active();     ///< AA 01 03 0D
    void turnOnCanActive();      ///< AA 01 18 0D
    void turnOffCanActive();     ///< AA 01 08 0D

    // ── Output frequency ─────────────────────────────────────
    /**
     * @brief Đặt tần số output.
     * @param hz Tần số mong muốn [100 – 1000] Hz
     *           Firmware dùng interval_ms = 1000/hz (2 bytes LE)
     */
    void setOutputHz(int hz);       ///< AA 02 [lo] [hi] 0D
    void setOutput1000Hz();         ///< shortcut 1000 Hz

    // ── Output interface ──────────────────────────────────────
    /**
     * @brief Chọn giao thức đầu ra.
     * @param iface USB(0) | RS485(1) | CAN(2) | VOFA(3)
     */
    void setOutputInterface(OutputInterface iface);   ///< AA 0A X 0D

    // ── Temperature control ───────────────────────────────────
    /**
     * @brief Bật/tắt hệ thống sưởi ổn nhiệt (giảm gyro drift).
     *
     * Khuyến nghị: bật khi dùng ngoài trời hoặc môi trường lạnh.
     */
    void enableTempControl();        ///< AA 04 01 0D
    void disableTempControl();       ///< AA 04 00 0D

    /**
     * @brief Đặt nhiệt độ mục tiêu (°C).
     * @param celsius Nhiệt độ mục tiêu, thường 40–60°C
     */
    void setTargetTemp(uint8_t celsius);   ///< AA 05 X 0D

    // ── CAN / 485 IDs ────────────────────────────────────────
    void setCanId (uint8_t can_id);    ///< AA 08 X 0D
    void setMstId (uint8_t mst_id);    ///< AA 09 X 0D

    // ── Calibration ──────────────────────────────────────────
    /**
     * @brief Khởi động hiệu chỉnh gyroscope tĩnh.
     *
     * Robot phải đặt hoàn toàn yên tĩnh trong quá trình này.
     * LED chớp khoảng 6 lần, sau đó IMU tự khởi động lại.
     */
    void calibrateGyro();            ///< AA 03 02 0D

    /**
     * @brief Khởi động hiệu chỉnh accelerometer 6 mặt.
     *
     * Người dùng lần lượt đặt IMU theo 6 hướng theo tín hiệu LED.
     * Chỉ dùng để re-calibration, không dùng trong runtime.
     */
    void calibrateAccel6Face();      ///< AA 03 03 0D

    // ── Persistence & reset ───────────────────────────────────
    void saveParams();               ///< AA 03 01 0D — lưu vào flash
    void factoryReset();             ///< AA 0B 01 0D — khôi phục mặc định

    // ── Special operations ────────────────────────────────────
    /**
     * @brief Reset góc heading về 0 tại tư thế hiện tại.
     *
     * Dùng khi robot vừa đứng vào tư thế chuẩn (home stance).
     * Không cần vào Setting Mode, có hiệu lực ngay lập tức.
     * KHÔNG cần saveParams() sau lệnh này.
     */
    void zeroAngle();                ///< AA 0C 01 0D

    /**
     * @brief Khởi động lại IMU.
     *
     * Không cần vào Setting Mode. Sau lệnh này cần chờ ~1s
     * rồi gọi open() lại nếu muốn tái sử dụng driver.
     */
    void restartImu();               ///< AA 00 00 0D

    // ══════════════════════════════════════════════════════════
    //  High-level configuration helpers
    // ══════════════════════════════════════════════════════════

    /**
     * @brief Cấu hình nhanh cho humanoid locomotion.
     *
     * Bật: accel + gyro + euler + quaternion EKF
     * Tần số: hz (mặc định 1000 Hz)
     * Nhiệt độ: bật ổn nhiệt tại temp_c (0 = không bật)
     * Lưu vào flash và thoát setting mode.
     */
    void configureForLocomotion(int hz = 1000, uint8_t temp_c = 0);

private:
    // ── Serial ──
    bool openSerial();
    void closeSerial();
    void writeBytes(const uint8_t* buf, size_t len);

    /** Gửi lệnh N lần với delay giữa các lần */
    void sendCmd(const uint8_t* cmd, size_t len,
                 int repeat = 5,
                 int delay_ms = 10);

    // ── Reader ──
    void readerLoop();
    void processBuffer();

    // ── Parser ──
    static uint16_t frameLen(uint8_t rid);
    bool  parseFrame(const uint8_t* frame, uint16_t len);
    void  applyFrame(uint8_t rid, float f1, float f2, float f3, float f4 = 0.f);

    // ── Math ──
    static Quaternion eulerToQuat(float roll_deg, float pitch_deg, float yaw_deg);

    // ── Members ──
    std::string port_;
    int         baud_;
    int         fd_{-1};

    std::thread       reader_thread_;
    std::atomic<bool> stop_flag_{false};

    mutable std::mutex obs_mutex_;
    ImuObservation     latest_obs_;

    bool got_accel_{false};
    bool got_gyro_ {false};
    bool got_euler_{false};
    bool got_quat_ {false};

    std::vector<uint8_t> buf_;

    mutable std::mutex stats_mutex_;
    Stats stats_;

    // Locomotion state tracking
    mutable std::chrono::steady_clock::time_point last_loco_ts_{};
    mutable bool prev_fallen_ {false};
    mutable bool prev_impact_ {false};

    std::function<void()> fall_cb_;
    std::function<void()> impact_cb_;
};

} // namespace dm_imu
