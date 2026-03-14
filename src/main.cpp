/**
 * @file main.cpp
 * @brief Demo + debug driver DM-IMU-L1
 *
 * Usage:
 *   sudo ./dm_imu_demo [port] [baud] [hz] [--no-config]
 *
 *   --no-config  : bỏ qua bước cấu hình (IMU đã cấu hình sẵn)
 *                  → dùng khi IMU mặc định đã stream OK
 *
 * Tip: Lần đầu chạy thêm --no-config để test xem IMU có stream không.
 *      Nếu thấy data → không cần config gì thêm.
 */
#include "dm_imu/imu_driver.hpp"

#include <iostream>
#include <iomanip>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include <string>
#include <cstring>
#include <cmath>

static std::atomic<bool> g_running{true};
static void sigHandler(int) { g_running.store(false); }

static void printObs(const dm_imu::ImuObservation& obs, uint64_t n)
{
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "─── #" << n << " ───────────────────────────────────────\n";
    std::cout << "  Accel  (m/s²) │"
              << " x=" << std::setw(8) << obs.accel.x
              << " y=" << std::setw(8) << obs.accel.y
              << " z=" << std::setw(8) << obs.accel.z << '\n';
    std::cout << "  Gyro   (°/s)  │"
              << " x=" << std::setw(8) << obs.angular_velocity.x
              << " y=" << std::setw(8) << obs.angular_velocity.y
              << " z=" << std::setw(8) << obs.angular_velocity.z << '\n';
    std::cout << "  Euler  (°)    │"
              << " roll="  << std::setw(8) << obs.roll
              << " pitch=" << std::setw(8) << obs.pitch
              << " yaw="   << std::setw(8) << obs.yaw << " [DRIFT]" << '\n';
    std::cout << "  Quat          │"
              << " w=" << std::setw(8) << obs.quaternion.w
              << " x=" << std::setw(8) << obs.quaternion.x
              << " y=" << std::setw(8) << obs.quaternion.y
              << " z=" << std::setw(8) << obs.quaternion.z
              << (obs.quat_from_imu ? "  [EKF]" : "  [Euler]") << '\n';
}

static void printLoco(const dm_imu::LocomotionState& s)
{
    std::cout << std::fixed << std::setprecision(4);
    // ── Reliable (gravity reference) ──────────────────────────────
    std::cout << "  [BALANCE] pitch=" << std::setw(7) << s.pitch_rad
              << "r  roll="  << std::setw(7) << s.roll_rad    << "r"
              << "  dPitch=" << std::setw(7) << s.pitch_rate  << "r/s"
              << "  dRoll="  << std::setw(7) << s.roll_rate   << "r/s\n";
    // ── Reliable short-term (gyro) ────────────────────────────────
    std::cout << "  [STEER ] yaw_rate=" << std::setw(7) << s.yaw_rate << "r/s"
              << "  (yaw_abs=" << std::setw(7) << s.yaw_rad << "r DRIFTS-use zeroAngle())\n";
    // ── Contact & fall ────────────────────────────────────────────
    std::cout << "  [EVENTS] |a|=" << std::setw(5) << s.accel_norm_g << "g"
              << "  impact=" << (s.impact_detected ? "YES" : "no ")
              << "  fallen=" << (s.is_fallen    ? "YES" : "no ")
              << "  still="  << (s.is_stationary ? "yes" : "no ")
              << "  dt=" << std::setprecision(1) << s.dt_s*1000.f << "ms\n";
}

int main(int argc, char** argv)
{
    std::string port      = "/dev/ttyACM0";
    int         baud      = 921600;
    int         hz        = 1000;
    bool        do_config = true;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--no-config") == 0) {
            do_config = false;
        } else if (i == 1) {
            port = argv[i];
        } else if (i == 2) {
            baud = std::stoi(argv[i]);
        } else if (i == 3) {
            hz = std::stoi(argv[i]);
        }
    }

    std::signal(SIGINT, sigHandler);

    std::cout << "DM-IMU Driver\n"
              << "  port=" << port
              << "  baud=" << baud
              << "  hz=" << hz
              << "  config=" << (do_config ? "yes" : "NO (--no-config)") << "\n\n";

    dm_imu::ImuDriver imu(port, baud);

    imu.onFall([]{
        std::cout << "\n*** FALL DETECTED ***\n\n";
    });

    if (!imu.open(do_config)) {
        std::cerr << "[error] Failed to open " << port << "\n";
        return 1;
    }

    // ── Chờ frame đầu tiên (timeout 10s với progress dots) ───────
    std::cout << "Waiting for valid frame";
    std::cout.flush();
    auto t0 = std::chrono::steady_clock::now();
    int  dot = 0;
    while (g_running.load()) {
        if (imu.getObs().valid) { std::cout << "\n"; break; }

        auto elapsed = std::chrono::steady_clock::now() - t0;
        if (elapsed > std::chrono::seconds(10)) {
            std::cout << "\n\n[TIMEOUT] No data from IMU after 10s.\n";
            std::cout << "Check:\n"
                      << "  1. IMU LED — đang nhấp nháy xanh? (normal mode)\n"
                      << "  2. ls -la /dev/ttyACM* — device có tồn tại?\n"
                      << "  3. cat /dev/ttyACM0 | xxd | head  — có byte nào ra không?\n"
                      << "  4. Thử: sudo ./dm_imu_demo --no-config\n"
                      << "     (bỏ qua config, test xem IMU có stream sẵn không)\n";
            auto st = imu.getStats();
            std::cout << "\nStats: ok=" << st.frames_ok
                      << " crc_err=" << st.frames_crc_err
                      << " nohdr=" << st.frames_nohdr << "\n";
            imu.close();
            return 1;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (++dot % 5 == 0) { std::cout << '.'; std::cout.flush(); }
    }

    if (!g_running.load()) { imu.close(); return 0; }

    std::cout << "IMU ready! Ctrl-C to stop.\n\n";

    // ── Main loop ─────────────────────────────────────────────────
    uint64_t count = 0;
    auto next_print = std::chrono::steady_clock::now();
    const auto interval = std::chrono::milliseconds(100);  // 10 Hz print

    while (g_running.load()) {
        auto obs   = imu.getObs();
        auto state = imu.getLocomotionState();

        auto now = std::chrono::steady_clock::now();
        if (now >= next_print && obs.valid) {
            printObs(obs, ++count);
            printLoco(state);
            std::cout << '\n';
            next_print = now + interval;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    auto st = imu.getStats();
    std::cout << "\nStats: frames_ok=" << st.frames_ok
              << "  crc_err=" << st.frames_crc_err << "\n";

    imu.close();
    return 0;
}
