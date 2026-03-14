#pragma once
#include <cstdint>
#include <cmath>
#include <chrono>

namespace dm_imu {

// ═══════════════════════════════════════════════════════════════════
//  ImuObservation — 64 bytes, aligned to one cache line.
//
//  EKF quaternion luôn có (RID_QUAT từ IMU firmware).
//  Không có Euler fallback, không có quat_from_imu flag.
//
//  RL policy input layout (10 floats liên tiếp từ offset 0):
//    [qw qx qy qz | gyr_x gyr_y gyr_z | acc_x acc_y acc_z]
//  → memcpy 40 bytes thẳng vào tensor input
// ═══════════════════════════════════════════════════════════════════

struct alignas(64) ImuObservation {
    // ── Orientation — EKF quaternion ZYX ──────────── offset  0 (16B)
    float qw{1.f};
    float qx{0.f};
    float qy{0.f};
    float qz{0.f};

    // ── Angular velocity (rad/s) ──────────────────── offset 16 (12B)
    float gyr_x{0.f};   // roll-rate  (body X)
    float gyr_y{0.f};   // pitch-rate (body Y)
    float gyr_z{0.f};   // yaw-rate   (body Z)

    // ── Linear acceleration (m/s²) ────────────────── offset 28 (12B)
    float acc_x{0.f};
    float acc_y{0.f};
    float acc_z{0.f};

    // ── Timing ────────────────────────────────────── offset 40  (8B)
    uint64_t timestamp_ns{0};   // CLOCK_MONOTONIC_RAW

    // ── Sequence ──────────────────────────────────── offset 48  (4B)
    uint32_t seq{0};   // monotonic; RL: seq != prev+1 → dropped tick

    // ── Status ────────────────────────────────────── offset 52  (1B)
    uint8_t valid{0};

    uint8_t _pad[11]{};   // → total 64 bytes ✓

    // ── Inline geometry (zero-cost, no stored Euler) ───────────────
    float pitch_rad() const noexcept {
        float s = 2.f * (qw*qy - qz*qx);
        if (s >=  1.f) return  1.5707963f;
        if (s <= -1.f) return -1.5707963f;
        return std::asin(s);
    }
    float roll_rad() const noexcept {
        return std::atan2(2.f*(qw*qx + qy*qz),
                          1.f - 2.f*(qx*qx + qy*qy));
    }
    float yaw_rad() const noexcept {
        return std::atan2(2.f*(qw*qz + qx*qy),
                          1.f - 2.f*(qy*qy + qz*qz));
    }
    float accel_norm() const noexcept {
        return std::sqrt(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z);
    }
};
static_assert(sizeof(ImuObservation)  == 64);
static_assert(alignof(ImuObservation) == 64);


// ═══════════════════════════════════════════════════════════════════
//  LocomotionState — derived, computed on-demand (safety monitor).
//  RL policy dùng ImuObservation trực tiếp.
// ═══════════════════════════════════════════════════════════════════

struct LocomotionState {
    float pitch_rad {0.f};
    float roll_rad  {0.f};
    float yaw_rad   {0.f};
    float pitch_rate{0.f};   // rad/s = gyr_y
    float roll_rate {0.f};   // rad/s = gyr_x
    float yaw_rate  {0.f};   // rad/s = gyr_z

    float accel_norm_g    {1.f};
    float accel_vertical_g{1.f};

    bool  is_fallen      {false};
    bool  is_stationary  {true};
    bool  impact_detected{false};

    float    dt_s{0.f};
    uint64_t timestamp_ns{0};

    static constexpr float kFallThreshRad = 1.05f;   // ~60°
    static constexpr float kImpactThreshG = 2.5f;
    static constexpr float kStationaryRps = 0.052f;  // 3°/s
};


// ═══════════════════════════════════════════════════════════════════
//  Protocol constants
// ═══════════════════════════════════════════════════════════════════

inline constexpr uint8_t  FRAME_HDR0     = 0x55;
inline constexpr uint8_t  FRAME_HDR1     = 0xAA;
inline constexpr uint8_t  FRAME_TAIL     = 0x0A;
inline constexpr uint16_t FRAME_LEN_STD  = 19;
inline constexpr uint16_t FRAME_LEN_QUAT = 23;
inline constexpr uint8_t  RID_ACCEL      = 0x01;
inline constexpr uint8_t  RID_GYRO       = 0x02;
inline constexpr uint8_t  RID_EULER      = 0x03;   // received but discarded
inline constexpr uint8_t  RID_QUAT       = 0x04;

} // namespace dm_imu