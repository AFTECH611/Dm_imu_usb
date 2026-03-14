#pragma once
/**
 * @file aimrt_imu_adapter.hpp
 * @brief Thin adapter kết nối ImuDriver với AimRT channel pub/sub.
 *
 * Sử dụng:
 * ─────────────────────────────────────────────────────────────────
 *  // Trong AimRT module Initialize():
 *  imu_adapter_ = std::make_unique<dm_imu::AimRtImuAdapter>(
 *      core_ref.GetExecutorManager().GetExecutor("imu_executor"),
 *      channel_handle_ref,
 *      "/dev/ttyACM0",
 *      921600
 *  );
 *  imu_adapter_->SetDecimation(5);       // 1000Hz / 5 = 200Hz publish
 *  imu_adapter_->Start();
 *
 *  // Trong AimRT module Shutdown():
 *  imu_adapter_->Stop();
 *
 * ─── RL side (subscriber) ────────────────────────────────────────
 *  // RL thread nhận ImuObservation qua AimRT channel
 *  channel_sub.Subscribe([](const dm_imu::ImuObservation& obs) {
 *      // obs đã ở SI units, quat đã chuẩn hoá
 *      policy_input[0] = obs.qw;   // orientation
 *      policy_input[1] = obs.qx;
 *      policy_input[2] = obs.qy;
 *      policy_input[3] = obs.qz;
 *      policy_input[4] = obs.gyr_x; // angular velocity rad/s
 *      policy_input[5] = obs.gyr_y;
 *      policy_input[6] = obs.gyr_z;
 *      policy_input[7] = obs.acc_x; // acceleration m/s²
 *      policy_input[8] = obs.acc_y;
 *      policy_input[9] = obs.acc_z;
 *      // 10 floats liên tiếp → có thể dùng memcpy vào tensor
 *  });
 *
 * ─── Latency path ────────────────────────────────────────────────
 *  IMU hardware (~1μs)
 *    → USB CDC (~100–300μs OS jitter)
 *    → read() syscall
 *    → parseFrame() + commitTick()   (< 1μs)
 *    → setObsCallback → Publish()    (< 1μs nếu không block)
 *    → AimRT channel dispatch
 *    → RL subscriber callback
 *  Total typical: 200–500μs end-to-end
 *
 * ─── Thread safety ───────────────────────────────────────────────
 *  • ImuDriver::reader_thread_ gọi obs_callback_ → Publish()
 *  • Nếu AimRT Publish() thread-safe từ external thread: OK as-is
 *  • Nếu không: dùng executor.Post() (xem PostingAdapter bên dưới)
 */

#include "imu_driver.hpp"
#include <memory>
#include <functional>

namespace dm_imu {

// ─────────────────────────────────────────────────────────────────
//  PublishFn: hàm do caller cung cấp để publish lên AimRT channel.
//  Signature phải khớp với AimRT publisher type của project.
//
//  Ví dụ nếu dùng protobuf:
//    using PublishFn = std::function<void(const imu_msgs::ImuObsMsg&)>;
//  Ví dụ nếu publish trực tiếp ImuObservation (zero-copy channel):
//    using PublishFn = std::function<void(const ImuObservation&)>;
// ─────────────────────────────────────────────────────────────────
using PublishFn = std::function<void(const ImuObservation&)>;

/**
 * @brief Option A — Direct publish từ reader_thread_.
 *
 * Yêu cầu: AimRT channel Publish() thread-safe với external thread.
 * Ưu điểm: latency thấp nhất (không post qua executor).
 * Nhược điểm: callback chạy trong reader_thread_, không phải executor.
 */
class AimRtImuAdapter {
public:
    /**
     * @param publish_fn  Hàm publish AimRT — được gọi từ reader_thread_
     * @param port        Serial port, mặc định /dev/ttyACM0
     * @param baud        Baudrate, mặc định 921600
     */
    AimRtImuAdapter(PublishFn publish_fn,
                    const std::string& port = "/dev/ttyACM0",
                    int baud = 921600)
        : publish_fn_(std::move(publish_fn))
        , driver_(port, baud)
    {}

    /**
     * @param every_n  Decimation: publish sau mỗi N IMU tick.
     *                 every_n=1 → 1000Hz (default), every_n=5 → 200Hz
     */
    void SetDecimation(uint32_t every_n) { every_n_ = every_n; }

    /** Đăng ký fall callback (truyền vào RL safety layer) */
    void SetFallCallback(std::function<void()> cb) {
        fall_cb_ = std::move(cb);
    }

    bool Start(bool configure = true) {
        driver_.setObsCallback(
            [this](const ImuObservation& obs) {
                publish_fn_(obs);
            },
            every_n_
        );
        if (fall_cb_) driver_.onFall(fall_cb_);
        return driver_.open(configure);
    }

    void Stop() { driver_.close(); }

    /** Pull trực tiếp khi cần (không qua AimRT) */
    bool GetObs(ImuObservation& out) const noexcept {
        return driver_.getObs(out);
    }

    ImuDriver::Stats GetStats() const noexcept {
        return driver_.getStats();
    }

    ImuDriver& Driver() { return driver_; }

private:
    PublishFn  publish_fn_;
    ImuDriver  driver_;
    uint32_t   every_n_{1};
    std::function<void()> fall_cb_;
};


/**
 * @brief Option B — Publish qua AimRT executor.Post() (thread-safe).
 *
 * Dùng khi AimRT channel không cho phép publish từ external thread.
 * Driver callback post một task vào executor; executor gọi Publish().
 * Tốn thêm ~1–5μs latency so với Option A.
 *
 * Khai báo executor type phù hợp với AimRT version của bạn:
 *   using ExecutorRef = aimrt::executor::ExecutorRef;
 *
 * Ví dụ sử dụng:
 *   AimRtImuAdapterPosting adapter(
 *       executor_ref,        // AimRT executor
 *       [&pub](const ImuObservation& o){ pub.Publish(Convert(o)); },
 *       "/dev/ttyACM0"
 *   );
 */
template<typename ExecutorRef>
class AimRtImuAdapterPosting {
public:
    AimRtImuAdapterPosting(ExecutorRef executor,
                            PublishFn   publish_fn,
                            const std::string& port = "/dev/ttyACM0",
                            int baud = 921600)
        : executor_(executor)
        , publish_fn_(std::move(publish_fn))
        , driver_(port, baud)
    {}

    void SetDecimation(uint32_t n) { every_n_ = n; }
    void SetFallCallback(std::function<void()> cb) { fall_cb_ = std::move(cb); }

    bool Start(bool configure = true) {
        driver_.setObsCallback(
            [this](const ImuObservation& obs) {
                // Bản sao nhỏ (64 bytes) — post vào executor queue
                executor_.Execute([this, obs]() mutable {
                    publish_fn_(obs);
                });
            },
            every_n_
        );
        if (fall_cb_) driver_.onFall(fall_cb_);
        return driver_.open(configure);
    }

    void Stop() { driver_.close(); }

    bool GetObs(ImuObservation& out) const noexcept {
        return driver_.getObs(out);
    }

    ImuDriver& Driver() { return driver_; }

private:
    ExecutorRef executor_;
    PublishFn   publish_fn_;
    ImuDriver   driver_;
    uint32_t    every_n_{1};
    std::function<void()> fall_cb_;
};


// ─────────────────────────────────────────────────────────────────
//  RL Policy helper — đọc ImuObservation thành flat float array.
//
//  Dùng khi RL policy nhận input dưới dạng float* (e.g., libtorch, onnx).
//  Layout:  [qw qx qy qz gyr_x gyr_y gyr_z acc_x acc_y acc_z]
//            0   1  2  3   4     5     6     7     8     9
// ─────────────────────────────────────────────────────────────────
inline void ObsToFloatArray(const ImuObservation& obs, float* out10) noexcept {
    // ImuObservation layout đã đúng thứ tự → memcpy trực tiếp 10 floats
    __builtin_memcpy(out10, &obs.qw, 10 * sizeof(float));
}

} // namespace dm_imu