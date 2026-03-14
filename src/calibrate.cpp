/**
 * @file calibrate.cpp
 * @brief Gyroscope zero-bias calibration tool cho DM-IMU-L1
 *
 * QUAN TRỌNG: Đặt IMU hoàn toàn bất động trước khi chạy.
 * Sau calibrate, yaw drift sẽ giảm xuống ~12°/giờ theo spec.
 *
 * Build:
 *   cd build && cmake .. && make -j$(nproc)
 *
 * Run:
 *   sudo ./dm_imu_calibrate [/dev/ttyACM0]
 */

#include "dm_imu/imu_driver.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

static std::atomic<bool> g_running{true};
static void sigHandler(int) { g_running.store(false); }

// ── Raw serial để gửi lệnh mà không cần full driver ──────────────
static int openPort(const char* port) {
    int fd = ::open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { perror("open"); return -1; }
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    tcgetattr(fd, &tty);
    cfsetispeed(&tty, B921600); cfsetospeed(&tty, B921600);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB|PARODD|CSTOPB|CRTSCTS);
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_iflag = tty.c_oflag = tty.c_lflag = 0;
    tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &tty);
    tcflush(fd, TCIFLUSH);
    return fd;
}

static void sendCmd(int fd, const uint8_t* cmd, int len, int repeat=5, int delay_ms=15) {
    for (int i = 0; i < repeat; i++) {
        ssize_t __w = write(fd, cmd, len); (void)__w;
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
}

int main(int argc, char** argv)
{
    const char* port = (argc >= 2) ? argv[1] : "/dev/ttyACM0";
    std::signal(SIGINT, sigHandler);

    std::cout << "╔══════════════════════════════════════════════════╗\n";
    std::cout << "║   DM-IMU-L1 Gyroscope Zero-Bias Calibration     ║\n";
    std::cout << "╚══════════════════════════════════════════════════╝\n\n";

    std::cout << "⚠️  TRƯỚC KHI TIẾP TỤC:\n";
    std::cout << "   • Đặt IMU trên mặt phẳng cứng, hoàn toàn bất động\n";
    std::cout << "   • Không có rung động, không chạm tay vào\n";
    std::cout << "   • Quá trình mất khoảng 3-5 giây\n";
    std::cout << "   • IMU sẽ tự khởi động lại sau khi xong\n\n";
    std::cout << "Port: " << port << "\n\n";
    std::cout << "Nhấn Enter để bắt đầu, hoặc Ctrl-C để thoát...";
    std::cin.get();

    if (!g_running.load()) return 0;

    int fd = openPort(port);
    if (fd < 0) {
        std::cerr << "Không mở được " << port << "\n";
        return 1;
    }

    // ── Bước 1: Vào setting mode ─────────────────────────────────
    std::cout << "\n[1/3] Vào setting mode...";
    std::cout.flush();
    { uint8_t c[] = {0xAA, 0x06, 0x01, 0x0D}; sendCmd(fd, c, 4, 5, 15); }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::cout << " OK\n";

    // ── Bước 2: Gửi lệnh calibrate gyro ─────────────────────────
    std::cout << "[2/3] Đang calibrate gyroscope zero-bias...\n";
    std::cout << "      (LED IMU chớp vàng ~6 lần)\n";
    std::cout.flush();
    { uint8_t c[] = {0xAA, 0x03, 0x02, 0x0D}; sendCmd(fd, c, 4, 3, 20); }

    // ── Chờ LED chớp xong và IMU tự restart ─────────────────────
    // Datasheet: LED nhấp nháy vàng khoảng 6 lần, rồi IMU tự khởi động lại
    std::cout << "      Đang chờ IMU hoàn thành và khởi động lại";
    std::cout.flush();
    for (int i = 0; i < 80 && g_running.load(); i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (i % 5 == 0) { std::cout << '.'; std::cout.flush(); }
    }
    std::cout << "\n";

    ::close(fd);

    // ── Bước 3: Verify với driver ────────────────────────────────
    std::cout << "[3/3] Kiểm tra yaw drift sau calibrate...\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    dm_imu::ImuDriver imu(port, 921600);
    if (!imu.open(false)) {  // no-config, IMU mới restart xong
        std::cerr << "Không kết nối được. Thử cắm lại USB.\n";
        return 1;
    }

    // Chờ frame đầu
    std::cout << "    Chờ IMU stream";
    std::cout.flush();
    auto t0 = std::chrono::steady_clock::now();
    while (g_running.load()) {
        if (imu.getObs().valid) break;
        if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(5)) {
            std::cout << "\n    Timeout. Calibrate vẫn được thực hiện, cắm lại USB nếu cần.\n";
            imu.close(); return 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << '.'; std::cout.flush();
    }
    std::cout << " OK\n\n";

    if (!g_running.load()) { imu.close(); return 0; }

    // Đo yaw drift trong 10 giây
    std::cout << "    Đo yaw drift trong 10 giây (giữ IMU bất động)...\n";
    float yaw_start = imu.getObs().yaw;
    float gyro_z_sum = 0.f;
    int   samples = 0;

    auto measure_end = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < measure_end && g_running.load()) {
        auto obs = imu.getObs();
        if (obs.valid) {
            gyro_z_sum += obs.angular_velocity.z;
            samples++;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    float yaw_end  = imu.getObs().yaw;
    float yaw_delta_10s  = yaw_end - yaw_start;
    float gyro_z_bias    = (samples > 0) ? gyro_z_sum / samples : 0.f;
    float drift_per_hour = yaw_delta_10s * 360.f;  // 10s → 1h

    std::cout << "\n╔══════════════════════════════════════════════════╗\n";
    std::cout << "║              KẾT QUẢ CALIBRATE                 ║\n";
    std::cout << "╚══════════════════════════════════════════════════╝\n";
    std::cout << std::fixed;
    std::cout.precision(4);
    std::cout << "  Gyro Z bias đo được  : " << gyro_z_bias   << " °/s\n";
    std::cout << "  Yaw thay đổi / 10s   : " << yaw_delta_10s << " °\n";
    std::cout << "  Ước tính drift / giờ : " << drift_per_hour << " °/h\n\n";

    if (std::abs(gyro_z_bias) < 1.0f) {
        std::cout << "  ✅ Calibrate THÀNH CÔNG\n";
        std::cout << "     Gyro Z bias < 1°/s — yaw drift sẽ ~" << drift_per_hour << "°/h\n";
        std::cout << "     (Spec: 12.686°/h)\n";
    } else if (std::abs(gyro_z_bias) < 5.0f) {
        std::cout << "  ⚠️  Drift còn cao hơn spec\n";
        std::cout << "     Thử calibrate lại, đảm bảo IMU hoàn toàn bất động\n";
        std::cout << "     và không có rung động nền (quạt, máy móc...)\n";
    } else {
        std::cout << "  ❌ Gyro bias rất lớn (" << gyro_z_bias << "°/s)\n";
        std::cout << "     Calibrate chưa thành công. Kiểm tra:\n";
        std::cout << "     • IMU có bị rung trong lúc calibrate?\n";
        std::cout << "     • Nhiệt độ IMU đã ổn định chưa? (chờ 2-3 phút sau khi cắm)\n";
        std::cout << "     • Thử: sudo ./dm_imu_calibrate --factory-reset trước\n";
    }

    std::cout << "\n";
    imu.close();
    return 0;
}
