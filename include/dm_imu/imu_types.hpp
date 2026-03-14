#pragma once
#include <cstdint>
#include <cmath>
#include <chrono>

namespace dm_imu {

// ══════════════════════════════════════════════════════════════════
//  Primitive types
// ══════════════════════════════════════════════════════════════════

struct Vec3 {
    float x{0.f}, y{0.f}, z{0.f};
    float norm() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator*(float s)       const { return {x*s,   y*s,   z*s};   }
};

struct Quaternion {
    float w{1.f}, x{0.f}, y{0.f}, z{0.f};
    float norm() const { return std::sqrt(w*w + x*x + y*y + z*z); }
    Quaternion normalized() const {
        float n = norm();
        if (n < 1e-6f) return {1.f, 0.f, 0.f, 0.f};
        return {w/n, x/n, y/n, z/n};
    }
};

// ══════════════════════════════════════════════════════════════════
//  ImuObservation — kết quả của getObs()
// ══════════════════════════════════════════════════════════════════
struct ImuObservation {
    Vec3       accel;
    Vec3       angular_velocity;
    Quaternion quaternion;

    float roll {0.f};   // độ, [-180, 180]  — TIN CẬY (tham chiếu trọng lực)
    float pitch{0.f};   // độ, [-90,  90 ]  — TIN CẬY (tham chiếu trọng lực)
    float yaw  {0.f};   // độ, [-180, 180]  — DRIFT (không có magnetometer)

    std::chrono::steady_clock::time_point timestamp{};
    bool valid        {false};
    bool quat_from_imu{false};
};

// ══════════════════════════════════════════════════════════════════
//  LocomotionState — trạng thái dẫn xuất cho bộ điều khiển robot
// ══════════════════════════════════════════════════════════════════
/**
 * @brief Trạng thái locomotion tính từ ImuObservation.
 *
 * ⚠️  YAW DRIFT LÀ BÌNH THƯỜNG với 6-axis IMU (không có magnetometer).
 *     DM-IMU-L1 chỉ có accel + gyro → không có tham chiếu heading tuyệt đối.
 *     Gyro bias tích phân → yaw trôi liên tục, đặc biệt khi đứng yên.
 *
 *     Cách xử lý đúng cho humanoid:
 *       1. Dùng yaw_rate (°/s) thay vì yaw_rad cho steering control
 *          → yaw_rate từ gyro là chính xác trong ngắn hạn
 *       2. Gọi imu.zeroAngle() khi robot về home stance
 *       3. Tích hợp heading từ leg odometry (encoder) nếu cần
 *       4. yaw_rad chỉ dùng tham khảo, KHÔNG dùng làm feedback cân bằng
 *
 *  ── Tin cậy (dùng cho balance controller) ──────────────────────
 *  pitch_rad / roll_rad    : góc nghiêng (rad) — tham chiếu trọng lực
 *  pitch_rate / roll_rate  : tốc độ góc (rad/s)
 *
 *  ── Tin cậy ngắn hạn (dùng cho steering) ────────────────────────
 *  yaw_rate                : tốc độ xoay (rad/s) — chính xác tức thời
 *
 *  ── KHÔNG tin cậy dài hạn ────────────────────────────────────────
 *  yaw_rad                 : heading tuyệt đối — bị drift, chỉ tham khảo
 *
 *  ── Contact & impact ─────────────────────────────────────────────
 *  accel_norm_g            : |accel|/g — spike khi footstrike
 *  accel_vertical_g        : thành phần đứng theo body Z
 *  impact_detected         : footstrike / landing event
 *
 *  ── Fall detection ───────────────────────────────────────────────
 *  is_fallen               : |pitch| hoặc |roll| vượt ngưỡng
 *  is_stationary           : gyro noise nhỏ, robot đứng yên
 */
struct LocomotionState {
    // ── TIN CẬY — dùng cho balance PD/LQR ────────────────────────
    float pitch_rad {0.f};     // rad  [-π/2,  π/2]
    float roll_rad  {0.f};     // rad  [-π,    π  ]
    float pitch_rate{0.f};     // rad/s
    float roll_rate {0.f};     // rad/s

    // ── TIN CẬY ngắn hạn — dùng cho steering ─────────────────────
    float yaw_rate  {0.f};     // rad/s — chính xác tức thời, không drift

    // ── THAM KHẢO — drift dài hạn, không dùng cho control ─────────
    // Dùng imu.zeroAngle() để reset về 0 khi cần
    float yaw_rad   {0.f};     // rad — drift liên tục nếu đứng yên

    // ── Contact & impact ──────────────────────────────────────────
    float accel_norm_g    {1.f};
    float accel_vertical_g{1.f};
    bool  impact_detected {false};

    // ── Fall & motion ─────────────────────────────────────────────
    bool is_fallen    {false};
    bool is_stationary{true};

    // ── Timing ────────────────────────────────────────────────────
    float dt_s{0.f};
    std::chrono::steady_clock::time_point timestamp{};

    // ── Thresholds ────────────────────────────────────────────────
    static constexpr float kFallThreshRad = 1.05f;   // ~60°
    static constexpr float kImpactThreshG = 2.5f;
    static constexpr float kStationaryDps = 3.0f;
};

// ══════════════════════════════════════════════════════════════════
//  Enums / constants
// ══════════════════════════════════════════════════════════════════

enum class OutputInterface : uint8_t {
    USB   = 0x00,
    RS485 = 0x01,
    CAN   = 0x02,
    VOFA  = 0x03,
};

static constexpr uint8_t  FRAME_HDR0     = 0x55;
static constexpr uint8_t  FRAME_HDR1     = 0xAA;
static constexpr uint8_t  FRAME_TAIL     = 0x0A;
static constexpr uint16_t FRAME_LEN_STD  = 19;
static constexpr uint16_t FRAME_LEN_QUAT = 23;
static constexpr uint16_t FRAME_LEN      = FRAME_LEN_STD;

static constexpr uint8_t  RID_ACCEL = 0x01;
static constexpr uint8_t  RID_GYRO  = 0x02;
static constexpr uint8_t  RID_EULER = 0x03;
static constexpr uint8_t  RID_QUAT  = 0x04;

} // namespace dm_imu
