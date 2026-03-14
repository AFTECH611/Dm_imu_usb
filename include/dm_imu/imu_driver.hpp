#pragma once

#include "imu_types.hpp"

#include <string>
#include <thread>
#include <atomic>
#include <vector>
#include <cstdint>
#include <functional>

namespace dm_imu {

// ═══════════════════════════════════════════════════════════════════
//  ImuDriver — EKF-only, AimRT pub/sub + RL thread optimized.
//
//  PUSH (AimRT):
//    imu.setObsCallback([&pub](const ImuObservation& o){ pub.Publish(o); }, 5);
//    imu.open();  // reader_thread SCHED_FIFO prio=80, cpu=3
//
//  PULL (RL hot path):
//    ImuObservation obs;
//    imu.getObs(obs);   // seqlock ~12ns, never blocks
// ═══════════════════════════════════════════════════════════════════

class ImuDriver {
public:
    using ObsCallback = std::function<void(const ImuObservation&)>;

    explicit ImuDriver(const std::string& port = "/dev/ttyACM0",
                       int                baud = 921600);
    ~ImuDriver();

    // ── Lifecycle ─────────────────────────────────────────────────

    /**
     * @param configure_imu  true: bật 4 channels, 1kHz
     * @param rt_name        pthread name cho reader_thread (empty = skip)
     * @param rt_priority    SCHED_FIFO priority; -1 = skip; khuyến nghị 80
     * @param bind_cpu       CPU affinity; -1 = skip; dùng isolated core
     */
    bool open(bool configure_imu = true,
              const std::string& rt_name  = "imu_reader",
              int                rt_priority = 80,
              int                bind_cpu    = -1);
    void close();
    bool isOpen() const;

    // ── Callback (đăng ký TRƯỚC open) ────────────────────────────

    /**
     * @param cb       Gọi từ reader_thread_ mỗi khi đủ 1 tick (accel+gyro+quat).
     *                 AimRT: cb = [&pub](const ImuObservation& o){ pub.Publish(o); }
     * @param every_n  Decimation: 1=1000Hz, 5=200Hz, 10=100Hz
     */
    void setObsCallback(ObsCallback cb, uint32_t every_n = 1);

    void onFall(std::function<void()> cb) { fall_cb_ = std::move(cb); }

    // ── PULL: seqlock ~12ns, never blocks writer ──────────────────

    bool           getObs(ImuObservation& out) const noexcept;
    ImuObservation getObs()                    const noexcept;

    // ── Safety monitor (NOT RL hot path) ─────────────────────────
    LocomotionState getLocomotionState(
        float fall_thresh_rad = LocomotionState::kFallThreshRad,
        float impact_thresh_g = LocomotionState::kImpactThreshG,
        float stationary_rps  = LocomotionState::kStationaryRps) const noexcept;

    // ── Fast scalar shortcuts ─────────────────────────────────────
    float pitchRad()  const noexcept;
    float rollRad()   const noexcept;
    float yawRad()    const noexcept;
    float pitchRate() const noexcept;
    float rollRate()  const noexcept;
    float yawRate()   const noexcept;
    bool  isFallen(float thresh = LocomotionState::kFallThreshRad) const noexcept;

    // ── Stats ─────────────────────────────────────────────────────
    struct Stats {
        uint64_t ticks_total    {0};
        uint64_t ticks_published{0};
        uint64_t frames_crc_err {0};
        uint64_t frames_dropped {0};
    };
    Stats getStats() const noexcept;

    // ── IMU configuration ─────────────────────────────────────────
    void configureForLocomotion(int hz = 1000, uint8_t temp_c = 0);
    void enterSettingMode();   void exitSettingMode();
    void turnOnAccel();        void turnOffAccel();
    void turnOnGyro();         void turnOffGyro();
    void turnOnEuler();        void turnOffEuler();
    void turnOnQuat();         void turnOffQuat();
    void setOutputHz(int hz);  void setOutput1000Hz();
    void enableTempControl();  void disableTempControl();
    void setTargetTemp(uint8_t c);
    void saveParams();
    void zeroAngle();
    void restartImu();
    void calibrateGyro();
    void calibrateAccel6Face();

private:
    bool openSerial();
    void closeSerial();
    void writeBytes(const uint8_t* buf, size_t len);
    void sendCmd(const uint8_t* cmd, size_t len, int n = 5, int ms = 10);

    void readerLoop();
    void processBuffer();
    static uint16_t frameLen(uint8_t rid) noexcept;
    bool  parseFrame(const uint8_t* frame, uint16_t len) noexcept;
    void  commitTick() noexcept;

    static uint64_t monotonicNs() noexcept;

    // ── Seqlock (cache line 1) ────────────────────────────────────
    alignas(64) mutable std::atomic<uint32_t> obs_ver_{0};

    // ── Published observation (cache line 2 = 64B) ───────────────
    ImuObservation obs_{};

    // ── Pending — reader_thread_ only (cache line 3) ──────────────
    alignas(64) struct {
        float acc_x, acc_y, acc_z;
        float gyr_x, gyr_y, gyr_z;
        float qw, qx, qy, qz;
        bool  has_accel : 1;
        bool  has_gyro  : 1;
        bool  has_quat  : 1;
    } pending_{};

    uint32_t tick_counter_{0};
    uint32_t seq_counter_ {0};

    // ── Stats (cache line 4) ──────────────────────────────────────
    alignas(64) mutable std::atomic<uint64_t> stat_ticks_total_{0};
    std::atomic<uint64_t> stat_ticks_pub_{0};
    std::atomic<uint64_t> stat_crc_err_  {0};
    std::atomic<uint64_t> stat_dropped_  {0};

    ObsCallback obs_cb_;
    uint32_t    cb_every_n_{1};

    std::function<void()> fall_cb_;
    bool prev_fallen_{false};

    std::string port_;
    int         baud_;
    int         fd_{-1};
    std::thread       reader_thread_;
    std::atomic<bool> stop_flag_{false};
    std::vector<uint8_t> buf_;
};

} // namespace dm_imu