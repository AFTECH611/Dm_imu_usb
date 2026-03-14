/**
 * @file main.cpp  — demo + benchmark getObs() latency
 *
 * Usage:
 *   sudo ./dm_imu_demo [port] [baud] [--no-config] [--bench]
 *
 *   --bench  : đo latency getObs() và jitter timestamp IMU
 *
 * Benchmark methodology:
 *   - getObs() latency: thời gian copy 64 bytes qua seqlock (không có contention)
 *   - wait_for_tick latency: thời gian từ khi gọi đến khi nhận tick mới
 *     (bao gồm USB OS jitter + select() wakeup)
 *   - IMU tick interval: khoảng cách giữa 2 consecutive seq (từ timestamp_ns)
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
#include <numeric>
#include <algorithm>
#include <vector>
#include <time.h>

static std::atomic<bool> g_running{true};
static void sigHandler(int) { g_running.store(false); }

static uint64_t nowNs() noexcept {
    struct timespec t{};
    ::clock_gettime(CLOCK_MONOTONIC_RAW, &t);
    return static_cast<uint64_t>(t.tv_sec)*1'000'000'000ULL + t.tv_nsec;
}

// ── Wait for a new tick (poll until seq changes) ───────────────────
// Trả về số ns chờ đợi (USB latency + OS jitter)
static uint64_t waitForNewObs(dm_imu::ImuDriver& imu,
                               dm_imu::ImuObservation& out) noexcept
{
    dm_imu::ImuObservation cur{};
    imu.getObs(cur);
    uint32_t old_seq = cur.seq;

    uint64_t t0 = nowNs();
    do {
        imu.getObs(out);
    } while (out.seq == old_seq);
    return nowNs() - t0;
}

static void printStats(std::vector<double>& v, const char* label,
                       const char* unit = "ns")
{
    if (v.empty()) return;
    std::sort(v.begin(), v.end());
    double sum  = std::accumulate(v.begin(), v.end(), 0.0);
    double mean = sum / v.size();
    double var  = 0;
    for (auto x : v) var += (x-mean)*(x-mean);
    double sd   = std::sqrt(var / v.size());
    double p50  = v[v.size()*50/100];
    double p99  = v[v.size()*99/100];
    double p999 = v[v.size()*999/1000 < v.size() ? v.size()*999/1000 : v.size()-1];

    std::cout << std::fixed << std::setprecision(1);
    std::cout << "  " << label << ":\n"
              << "    mean=" << mean << unit
              << "  sd=" << sd
              << "  p50=" << p50
              << "  p99=" << p99
              << "  p99.9=" << p999
              << "  min=" << v.front()
              << "  max=" << v.back()
              << "  n=" << v.size() << "\n";
}

static void runBench(dm_imu::ImuDriver& imu)
{
    constexpr int N = 2000;
    std::cout << "Benchmarking " << N << " samples...\n\n";

    // ── Test A: getObs() copy latency (seqlock, no writer contention) ─
    std::vector<double> getobs_ns;
    getobs_ns.reserve(N);
    for (int i = 0; i < N; ++i) {
        dm_imu::ImuObservation obs{};
        uint64_t t0 = nowNs();
        imu.getObs(obs);
        uint64_t t1 = nowNs();
        getobs_ns.push_back(static_cast<double>(t1 - t0));
        // Không sleep — đo pure copy latency không có contention
        (void)obs;
    }

    // ── Test B: wait-for-new-tick latency ────────────────────────────
    // Đây là latency thực tế RL thread phải chờ: USB + OS + select()
    std::vector<double> wait_ns, interval_ns;
    wait_ns.reserve(1000);
    interval_ns.reserve(1000);
    uint64_t prev_ts = 0;

    for (int i = 0; i < 1000 && g_running.load(); ++i) {
        dm_imu::ImuObservation obs{};
        uint64_t w = waitForNewObs(imu, obs);
        wait_ns.push_back(static_cast<double>(w));

        if (prev_ts && obs.timestamp_ns > prev_ts) {
            interval_ns.push_back(static_cast<double>(obs.timestamp_ns - prev_ts));
        }
        prev_ts = obs.timestamp_ns;
    }

    // ── Test C: seqlock under writer contention ───────────────────────
    // Đo getObs() trong khi writer đang liên tục update (worst case)
    std::atomic<bool> bench_stop{false};
    volatile uint64_t sink = 0;   // prevent dead-code elimination

    // Giả lập writer bằng cách trigger nhiều getObs() đồng thời
    std::vector<double> contended_ns;
    contended_ns.reserve(N);

    // Spawn 2 reader threads để tạo contention trên seqlock
    std::thread r1([&](){
        dm_imu::ImuObservation o{};
        while (!bench_stop.load(std::memory_order_relaxed)) {
            imu.getObs(o); sink = o.seq;
        }
    });
    std::thread r2([&](){
        dm_imu::ImuObservation o{};
        while (!bench_stop.load(std::memory_order_relaxed)) {
            imu.getObs(o); sink = o.seq;
        }
    });

    for (int i = 0; i < N; ++i) {
        dm_imu::ImuObservation obs{};
        uint64_t t0 = nowNs();
        imu.getObs(obs);
        uint64_t t1 = nowNs();
        contended_ns.push_back(static_cast<double>(t1 - t0));
    }
    bench_stop.store(true);
    r1.join(); r2.join();

    // ── Print results ─────────────────────────────────────────────────
    std::cout << "A. getObs() copy (no contention):\n";
    printStats(getobs_ns, "latency");

    std::cout << "\nB. wait-for-new-tick (USB+OS delivery latency):\n";
    printStats(wait_ns,      "wait time");
    printStats(interval_ns,  "tick interval (ideal=1000000ns)");

    std::cout << "\nC. getObs() under 2-reader contention:\n";
    printStats(contended_ns, "latency");

    auto st = imu.getStats();
    std::cout << "\nStats: ticks=" << st.ticks_total
              << "  crc_err=" << st.frames_crc_err
              << "  dropped=" << st.frames_dropped << "\n";
}

static void printObs(const dm_imu::ImuObservation& obs, uint64_t n)
{
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "─── #" << n << " (seq=" << obs.seq << ") ───────────────────\n";
    std::cout << "  Quat    │ w=" << std::setw(8) << obs.qw
              << " x=" << std::setw(8) << obs.qx
              << " y=" << std::setw(8) << obs.qy
              << " z=" << std::setw(8) << obs.qz << '\n';
    std::cout << "  Gyro r/s│ x=" << std::setw(8) << obs.gyr_x
              << " y=" << std::setw(8) << obs.gyr_y
              << " z=" << std::setw(8) << obs.gyr_z << '\n';
    std::cout << "  Accel   │ x=" << std::setw(8) << obs.acc_x
              << " y=" << std::setw(8) << obs.acc_y
              << " z=" << std::setw(8) << obs.acc_z << '\n';
    std::cout << std::setprecision(2);
    std::cout << "  pitch=" << std::setw(6) << obs.pitch_rad() * 57.2958f << "\xb0"
              << "  roll="  << std::setw(6) << obs.roll_rad()  * 57.2958f << "\xb0"
              << "  yaw="   << std::setw(7) << obs.yaw_rad()   * 57.2958f << "\xb0\n";
}

int main(int argc, char** argv)
{
    std::string port      = "/dev/ttyACM0";
    int         baud      = 921600;
    bool        do_config = true;
    bool        do_bench  = false;

    for (int i = 1; i < argc; ++i) {
        if      (!std::strcmp(argv[i], "--no-config")) do_config = false;
        else if (!std::strcmp(argv[i], "--bench"))     do_bench  = true;
        else if (i == 1) port = argv[i];
        else if (i == 2) baud = std::stoi(argv[i]);
    }

    std::signal(SIGINT, sigHandler);

    std::cout << "DM-IMU [AimRT-optimized]  port=" << port
              << "  baud=" << baud
              << "  config=" << (do_config ? "yes" : "no") << "\n\n";

    dm_imu::ImuDriver imu(port, baud);

    // ── PUSH callback (AimRT publisher) ───────────────────────────
    std::atomic<uint64_t> cb_count{0};
    imu.setObsCallback([&cb_count](const dm_imu::ImuObservation& obs) {
        cb_count.fetch_add(1, std::memory_order_relaxed);
        (void)obs;
        // AimRT: channel_publisher.Publish(Convert(obs));
    }, /*every_n=*/1);

    imu.onFall([]{ std::cout << "\n*** FALL ***\n\n"; });

    if (!imu.open(do_config)) {
        std::cerr << "[error] Cannot open " << port << "\n";
        return 1;
    }

    // ── Wait first valid tick ──────────────────────────────────────
    std::cout << "Waiting for first tick";
    std::cout.flush();
    auto t0 = std::chrono::steady_clock::now();
    while (g_running.load()) {
        dm_imu::ImuObservation obs{};
        if (imu.getObs(obs)) { std::cout << "\n"; break; }
        if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(10)) {
            std::cout << "\n[TIMEOUT] No data. Try --no-config\n";
            imu.close(); return 1;
        }
        std::cout << '.'; std::cout.flush();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    if (!g_running.load()) { imu.close(); return 0; }
    std::cout << "IMU ready!\n\n";

    if (do_bench) { runBench(imu); imu.close(); return 0; }

    // ── Print loop (PULL model 10Hz) ───────────────────────────────
    uint64_t count = 0;
    auto next_print = std::chrono::steady_clock::now();

    while (g_running.load()) {
        auto now = std::chrono::steady_clock::now();
        if (now >= next_print) {
            dm_imu::ImuObservation obs{};
            imu.getObs(obs);
            printObs(obs, ++count);
            auto st = imu.getStats();
            std::cout << "  ticks="    << st.ticks_total
                      << "  cb="       << cb_count.load()
                      << "  crc_err="  << st.frames_crc_err << "\n\n";
            next_print = now + std::chrono::milliseconds(100);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    auto st = imu.getStats();
    std::cout << "\nFinal: ticks=" << st.ticks_total
              << "  crc_err=" << st.frames_crc_err
              << "  cb=" << cb_count.load() << "\n";
    imu.close();
    return 0;
}