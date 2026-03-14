# dm_imu_usb — C++ Driver for DM-IMU-L1

> Driver C++ không phụ thuộc ROS cho **DM-IMU-L1** (达妙), đọc dữ liệu qua **USB-CDC**
> và cung cấp observation API tối ưu cho **humanoid robot locomotion + RL sim2real**.

---

## Mục lục

1. [Tổng quan dự án](#1-tổng-quan-dự-án)
2. [Phần cứng & Yêu cầu hệ thống](#2-phần-cứng--yêu-cầu-hệ-thống)
3. [Cài đặt nhanh](#3-cài-đặt-nhanh)
4. [Giao thức USB & Luồng dữ liệu](#4-giao-thức-usb--luồng-dữ-liệu)
5. [Kiến trúc đa luồng & Tối ưu](#5-kiến-trúc-đa-luồng--tối-ưu)
6. [API sử dụng](#6-api-sử-dụng)
7. [Tích hợp AimRT](#7-tích-hợp-aimrt)
8. [Mô hình hóa cho Huấn luyện RL (Sim2Real)](#8-mô-hình-hóa-cho-huấn-luyện-rl-sim2real)
9. [Cấu hình & Hiệu chỉnh IMU](#9-cấu-hình--hiệu-chỉnh-imu)
10. [Cấu trúc thư mục](#10-cấu-trúc-thư-mục)

---

## 1. Tổng quan dự án

### IMU này dùng để làm gì?

DM-IMU-L1 là cảm biến đo chuyển động quán tính (**I**nertial **M**easurement **U**nit) gắn trên thân robot humanoid. Nó đo:

- **Quaternion** — tư thế không gian 3D của thân robot (nghiêng, lăn, xoay)
- **Gyroscope** — tốc độ xoay tức thời (rad/s)
- **Accelerometer** — gia tốc tuyến tính (m/s²)

Dữ liệu này là đầu vào quan trọng nhất cho bộ điều khiển cân bằng và policy RL locomotion.

### Tại sao cần driver riêng?

DM-IMU-L1 có driver gốc cho ROS1/ROS2. Driver này **loại bỏ hoàn toàn ROS** để:
- Chạy được trong môi trường nhúng không có ROS (máy tính trên robot)
- Tích hợp trực tiếp với **AimRT** pub/sub framework
- Đưa observation vào **RL policy thread** với latency thấp nhất có thể

### Số đo thực tế (benchmark trên Jetson AGX Orin)

| Metric | Kết quả |
|--------|---------|
| `getObs()` latency (p50) | **12 ns** |
| `getObs()` latency (p99.9) | 45 ns |
| IMU stream rate | 1000 Hz |
| USB OS jitter (sd) | ~25 μs |
| CRC errors | 0 |
| Dropped frames | 0 |

---

## 2. Phần cứng & Yêu cầu hệ thống

### Phần cứng

| Thành phần | Thông số |
|---|---|
| IMU | DM-IMU-L1 (达妙) |
| Chip cảm biến | **Bosch BMI088** (16-bit, 6-axis) |
| Kết nối | USB Type-C → USB-CDC-ACM |
| Baudrate | 921600 bps |
| Device file | `/dev/ttyACM0` (Linux) |
| Stream rate | 50 – 1000 Hz (configurable) |
| Nguồn | 5V qua USB |

### Phần mềm

```
OS       : Ubuntu 20.04 / 22.04 (hoặc Jetson L4T)
Compiler : GCC 9+ hoặc Clang 10+
CMake    : 3.14+
Standard : C++17
Deps     : POSIX only (không cần thư viện bên ngoài)
```

---

## 3. Cài đặt nhanh

### Bước 1 — Clone và build

```bash
git clone <repo_url> dm_imu_usb
cd dm_imu_usb
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### Bước 2 — Cấp quyền cổng USB

```bash
# Cách 1: thêm user vào group dialout (cần logout/login lại)
sudo usermod -aG dialout $USER

# Cách 2: tạm thời (reset sau reboot)
sudo chmod 666 /dev/ttyACM0
```

### Bước 3 — Kiểm tra IMU có nhận data không

```bash
# Xem byte thô từ IMU (phải thấy data liên tục)
sudo cat /dev/ttyACM0 | xxd | head -20
```

Nếu không thấy gì → kiểm tra LED trên IMU:
- **LED xanh nhấp nháy** = normal mode, đang stream ✅
- **LED vàng nhấp nháy** = setting mode, không stream ⚠️

### Bước 4 — Chạy demo

```bash
# Nếu IMU đã được cấu hình sẵn (LED xanh):
sudo ./dm_imu_demo --no-config

# Nếu lần đầu dùng (tự động cấu hình):
sudo ./dm_imu_demo

# Benchmark latency:
sudo ./dm_imu_demo --no-config --bench
```

### Bước 5 — Tích hợp vào project của bạn

```cmake
# CMakeLists.txt của project bạn
add_subdirectory(dm_imu_usb)
target_link_libraries(your_robot_node PRIVATE dm_imu_lib)
```

---

## 4. Giao thức USB & Luồng dữ liệu

### Từ cảm biến vật lý đến số liệu trong code

```
┌─────────────────────────────────────────────────────────────────┐
│                       DM-IMU-L1 Hardware                        │
│                                                                  │
│  BMI088 Accel ──┐                                               │
│  BMI088 Gyro  ──┼──→  Internal EKF  ──→  Quaternion (RID 0x04)  │
│                 │                                               │
│                 ├──────────────────────→  Accel raw  (RID 0x01) │
│                 └──────────────────────→  Gyro raw   (RID 0x02) │
│                 (Euler RID 0x03 cũng được gửi nhưng bị bỏ qua)  │
└───────────────────────────┬─────────────────────────────────────┘
                            │ USB CDC-ACM / 921600 baud
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│              Linux Kernel USB CDC-ACM Driver                     │
│           (thêm ~200–400 µs OS jitter — không tránh được)       │
└───────────────────────────┬─────────────────────────────────────┘
                            │ /dev/ttyACM0
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                 dm_imu_usb C++ Driver                            │
│                                                                  │
│  readerLoop()  →  processBuffer()  →  parseFrame()              │
│                                            │                     │
│                         RID_ACCEL ─────────┤                     │
│                         RID_GYRO  ─────────┼──→ pending_{}      │
│                         RID_EULER (discard)│    (chờ đủ tick)   │
│                         RID_QUAT  ─────────┘                     │
│                                            │                     │
│                              commitTick() ─┘ (khi đủ 3 RID)    │
│                                   │                              │
│                              seqlock write                       │
│                              ImuObservation (64B, 1 cache line)  │
└───────────────────────────┬─────────────────────────────────────┘
                            │
              ┌─────────────┴──────────────┐
              │                            │
              ▼ PUSH                       ▼ PULL
    setObsCallback(fn)             getObs(obs)
    AimRT pub / logger             RL policy / safety monitor
    (from reader_thread_)          (any thread, ~12 ns)
```

### Bố cục 1 tick IMU (80 bytes / 1ms tại 1000 Hz)

Mỗi millisecond, IMU gửi **4 frame liên tiếp**:

```
Frame 1 — RID 0x01 (Accel)     19 bytes
├── 0x55 0xAA       Header
├── 0x01            Slave ID
├── 0x01            RID = Accel
├── acc_x acc_y acc_z   3 × float32 LE
├── CRC16 LE
└── 0x0A            Tail

Frame 2 — RID 0x02 (Gyro)      19 bytes  (cấu trúc tương tự)
Frame 3 — RID 0x03 (Euler)     19 bytes  (nhận nhưng bỏ qua)
Frame 4 — RID 0x04 (Quat)      23 bytes  ← 4 float thay vì 3
├── ...
├── qw qx qy qz     4 × float32 LE
├── CRC16 LE
└── 0x0A
```

Driver **commit tick** chỉ sau khi nhận `RID_QUAT` (frame cuối của tick), đảm bảo không bao giờ publish observation không đầy đủ.

### CRC-16

Thuật toán CCITT-FALSE (polynomial `0x1021`, init `0xFFFF`), tính trên toàn bộ frame trừ 3 byte cuối (CRC_lo, CRC_hi, Tail). Có fallback cho firmware cũ bỏ qua 2 byte header.

---

## 5. Kiến trúc đa luồng & Tối ưu

### Sơ đồ thread

```
                  ┌──────────────────────────────┐
                  │        reader_thread_         │
                  │  SCHED_FIFO, priority = 80    │
                  │  (optional: CPU affinity)     │
                  │                               │
                  │  select() ← USB wake-on-data  │
                  │  read() → processBuffer()     │
                  │  parseFrame() → commitTick()  │
                  │                               │
                  │  ┌─────────────────────────┐  │
                  │  │   pending_{}  (private) │  │
                  │  │   acc / gyro / quat     │  │
                  │  └───────────┬─────────────┘  │
                  └──────────────┼─────────────────┘
                                 │ seqlock write
                                 │ obs_ver_ odd → write obs_ → even
                                 ▼
                  ┌──────────────────────────────┐
                  │   ImuObservation obs_        │
                  │   alignas(64) — 1 cache line │  ← shared memory
                  └──────────────────────────────┘
                                 │ seqlock read (~12 ns)
              ┌──────────────────┼──────────────────┐
              │                  │                  │
              ▼                  ▼                  ▼
      RL policy thread    Safety monitor    AimRT publisher
      getObs(obs)         getLocomotion()   obs_callback_
      ~200 Hz             ~50 Hz            ~1000 Hz
```

### Tại sao dùng Seqlock thay Mutex?

**Mutex** (cách thông thường): khi RL thread đọc, nó phải *xin phép* reader thread → tốn ~200ns mỗi lần, và có thể bị block.

**Seqlock** (cách hiện tại): reader thread ghi xong rồi tăng số version. RL thread đọc bản sao 64 bytes mà **không cần hỏi ai** → ~12ns, không bao giờ block.

```
Seqlock protocol:
  Writer: ver 0→1 (odd=đang ghi) → write 64B → ver 1→2 (even=xong)
  Reader: đọc ver (chẵn?) → copy 64B → đọc lại ver (vẫn không đổi?) → OK
```

Trên ARM64, toàn bộ quá trình đọc là: `ldar` + `4×ldp` + `dmb` + `ldr` = **2 memory barrier** thay vì mutex lock/unlock.

### Tối ưu `select()` thay vì `sleep()`

```
Trước (sleep):  read() → EAGAIN → sleep(200µs) → read() lại
                Nếu data đến sau 50µs → lãng phí 150µs chờ vô ích

Sau (select):   select(fd, timeout=2ms) → kernel wake ngay khi USB có byte
                Không lãng phí CPU, không bỏ sót data
```

### Tối ưu cache: `pending_` không shared

`pending_{}` (buffer tích lũy 3 frame chờ đủ tick) chỉ được reader_thread_ đọc/ghi — đặt riêng trên cache line riêng (`alignas(64)`) để không gây **false sharing** với `obs_` mà RL thread đọc.

### Bộ nhớ layout

```
Cache line 1 (64B): obs_ver_    ← seqlock version counter
Cache line 2 (64B): obs_        ← ImuObservation (shared)
Cache line 3 (64B): pending_{}  ← reader_thread_ only
Cache line 4 (64B): stats       ← atomic counters
```

---

## 6. API sử dụng

### Khởi động

```cpp
#include "dm_imu/imu_driver.hpp"

dm_imu::ImuDriver imu("/dev/ttyACM0", 921600);

// Đăng ký callback TRƯỚC khi open()
imu.setObsCallback([](const dm_imu::ImuObservation& obs) {
    // Gọi mỗi IMU tick — dùng cho AimRT publisher, logger...
}, /*every_n=*/5);  // 1000Hz / 5 = publish 200Hz

imu.onFall([&robot]{ robot.triggerRecovery(); });

// Mở cổng + cấu hình IMU + khởi động reader_thread_ SCHED_FIFO
imu.open(/*configure_imu=*/true,
         /*rt_name=*/"imu_reader",
         /*rt_priority=*/80,
         /*bind_cpu=*/-1);      // -1 = không bind CPU cụ thể
```

### PULL — đọc từ RL thread

```cpp
// Cách 1: copy vào biến có sẵn (zero allocation)
dm_imu::ImuObservation obs;
imu.getObs(obs);

if (obs.valid) {
    // Orientation — EKF quaternion, ZYX convention
    float qw = obs.qw, qx = obs.qx, qy = obs.qy, qz = obs.qz;

    // Angular velocity (rad/s) — đã convert từ °/s trong driver
    float wx = obs.gyr_x;   // roll-rate
    float wy = obs.gyr_y;   // pitch-rate
    float wz = obs.gyr_z;   // yaw-rate

    // Linear acceleration (m/s²)
    float ax = obs.acc_x, ay = obs.acc_y, az = obs.acc_z;

    // Euler angles — tính trực tiếp từ quat, không lưu thêm
    float pitch = obs.pitch_rad();
    float roll  = obs.roll_rad();
    float yaw   = obs.yaw_rad();    // ⚠️ drift dài hạn, chỉ tham khảo

    // Timestamp (CLOCK_MONOTONIC_RAW, không bị NTP adjust)
    uint64_t ts_ns = obs.timestamp_ns;

    // Sequence number — phát hiện dropped tick
    uint32_t seq = obs.seq;   // nếu seq != prev_seq+1 → bỏ sót 1 tick
}
```

### RL policy input — 10 float liên tiếp

```cpp
// Layout: [qw qx qy qz | gyr_x gyr_y gyr_z | acc_x acc_y acc_z]
// Tất cả ở offset 0 → memcpy thẳng vào tensor
float policy_input[10];
memcpy(policy_input, &obs.qw, 10 * sizeof(float));

// Hoặc dùng helper:
dm_imu::ObsToFloatArray(obs, policy_input);   // từ aimrt_imu_adapter.hpp
```

### Safety monitor

```cpp
// Gọi từ safety thread (không phải RL hot path)
auto state = imu.getLocomotionState();

if (state.is_fallen)        robot.halt();
if (state.impact_detected)  contact_estimator.onFootstrike();
if (state.is_stationary)    motor_controller.setIdleMode();

float yaw_rate = state.yaw_rate;  // rad/s — cho steering control
```

---

## 7. Tích hợp AimRT

File `include/dm_imu/aimrt_imu_adapter.hpp` cung cấp 2 adapter:

### Option A — Direct publish (latency thấp nhất)

```cpp
// Yêu cầu: AimRT channel Publish() thread-safe với external thread
dm_imu::AimRtImuAdapter adapter(
    [&pub](const dm_imu::ImuObservation& obs) {
        pub.Publish(Convert(obs));   // convert sang proto nếu cần
    },
    "/dev/ttyACM0"
);
adapter.SetDecimation(5);   // 1000Hz → 200Hz
adapter.Start();
```

### Option B — Qua AimRT Executor (thread-safe tuyệt đối)

```cpp
// Dùng khi channel không cho publish từ external thread
dm_imu::AimRtImuAdapterPosting<aimrt::executor::ExecutorRef> adapter(
    executor_ref,
    [&pub](const dm_imu::ImuObservation& obs){ pub.Publish(Convert(obs)); },
    "/dev/ttyACM0"
);
adapter.SetDecimation(5);
adapter.Start();
```

### Latency end-to-end

```
IMU sampling
  → USB CDC OS stack        200–400 µs  (hardware limit)
  → read() + parseFrame()    < 5 µs
  → seqlock write            < 1 µs
  → obs_callback + Publish   < 2 µs
  ─────────────────────────────────
  Total                  ~200–400 µs   (99% do USB)
```

---

## 8. Mô hình hóa cho Huấn luyện RL (Sim2Real)

### Tại sao cần model IMU trong simulator?

Trong Isaac Gym / MuJoCo, robot biết chính xác tư thế của mình. Ngoài đời thực, IMU đo sai do noise và delay. Nếu train không có model IMU, policy học được sẽ **không chạy được trên hardware** (sim2real gap).

### Chip BMI088 — Thông số noise từ datasheet

```
─── GYROSCOPE ────────────────────────────────────────────────────
Noise density      : 0.014 °/s/√Hz
Zero-rate offset   : ±1 °/s  (typ) / ±5 °/s (max)
Bias stability     : <2 °/hr  (rất tốt, nhờ calib factory)
Thermal drift      : <0.015 °/s/K

─── ACCELEROMETER ────────────────────────────────────────────────
Noise density      : 170 µg/√Hz  (avg X/Y/Z)
Zero offset        : ±20 mg
Thermal drift      : <0.2 mg/K
```

### Tính σ noise cho simulation

```python
import numpy as np

POLICY_HZ = 200        # Hz — tần số policy

# Gyro white noise tại policy frequency
# BW = policy_hz / 2 (Nyquist)
gyro_sigma = 0.014 * (np.pi/180) * np.sqrt(POLICY_HZ/2)
# = 0.00244 rad/s ≈ 0.14 °/s

# Accelerometer white noise
acc_sigma = 170e-6 * 9.81 * np.sqrt(POLICY_HZ/2)
# = 0.0167 m/s² ≈ 1.7 mg
```

### Model đầy đủ (thứ tự áp dụng trong simulator)

Áp dụng **theo thứ tự** này mỗi policy step:

```python
class BMI088NoiseModel:
    def step(self, quat_true, gyro_true, acc_true):
        # 1. Bias khởi tạo ngẫu nhiên đầu episode (±1°/s)
        #    + bias drift từng step (2°/hr random walk)
        gyro_bias  += randn * GYRO_BIAS_WALK * sqrt(dt)
        gyro_noisy  = gyro_true + gyro_bias + randn * gyro_sigma

        acc_bias   = resample_per_episode(±20mg)
        acc_noisy   = acc_true  + acc_bias  + randn * acc_sigma

        # 2. Low-pass filter phần cứng BMI088 (~145 Hz cutoff)
        alpha = dt / (1/(2π×145) + dt)
        gyro_filtered = alpha*gyro_noisy + (1-alpha)*gyro_prev
        acc_filtered  = alpha*acc_noisy  + (1-alpha)*acc_prev

        # 3. Quantization 16-bit
        gyro_lsb = (2000°/s) / 32768 → round to LSB
        acc_lsb  = (24g)     / 32768 → round to LSB

        # 4. Latency USB: trễ 1 step (5ms tại 200Hz)
        obs_delayed = shift_history_buffer(1 step)

        # EKF quaternion đã filtered → noise nhỏ hơn gyro raw
        quat_noisy = quat_true + randn * 0.003   # ~0.17° RMS
        return quat_noisy, gyro_out, acc_out
```

### Domain Randomization ranges

| Tham số | Min | Max | Đơn vị |
|---|---|---|---|
| Gyro noise scale | 0.5× | 2.0× | × sigma |
| Gyro bias | 0 | 5.0 | °/s |
| Gyro bias walk | 0 | 5.0 | °/hr |
| Acc noise scale | 0.5× | 2.0× | × sigma |
| Acc bias | 0 | 50 | mg |
| Quat noise | 0 | 0.005 | rad RMS |
| Observation delay | 0 | 2 | policy steps |

### Những gì **không** cần model thêm

- **Vibration rejection** — BMI088 có built-in vibration suppression cho motor resonance. Không cần notch filter trong sim.
- **EKF quaternion** đã qua internal filter → noise nhỏ hơn nhiều so với raw gyro. Model `quat_noisy` riêng biệt với gyro.

### Thứ tự ưu tiên sim2real

```
Quan trọng nhất ──→ 1. Gyro bias (±1°/s) + bias walk
                       Ảnh hưởng trực tiếp đến balance

                    2. Observation latency (1–2 steps)
                       Policy học sai timing nếu bỏ qua

                    3. Accelerometer bias (±20mg)
                       Ảnh hưởng gravity estimation

                    4. White noise (scale ×0.5–2.0)

                    5. LPF phase lag
Ít quan trọng ────→ 6. Quantization (16-bit → sai số <0.1°/s)
```

### Khuyến nghị tần số policy

```
IMU stream    : 1000 Hz  (giữ nguyên)
Policy freq   : 200 Hz   (every_n=5 trong setObsCallback)
Joint control : 200 Hz   (đồng bộ với policy)
```

Tại 200 Hz, jitter USB 25µs = 0.5% chu kỳ → không đáng kể. Chạy nhanh hơn không có lợi vì bottleneck là USB latency ~300µs ≈ 1.5 chu kỳ ở 200Hz.

---

## 9. Cấu hình & Hiệu chỉnh IMU

### Hiệu chỉnh Gyroscope (bắt buộc trước khi deploy)

```bash
# Robot phải đứng HOÀN TOÀN YÊN TĨNH, không có rung động
# LED nhấp nháy ~6 lần, IMU tự restart sau khi xong (~8 giây)
sudo ./dm_imu_demo --calibrate-gyro
```

Hoặc trong code:
```cpp
imu.enterSettingMode();
imu.calibrateGyro();    // robot đứng yên ~6s
// IMU sẽ tự restart → cần open() lại
```

### Kiểm tra kết quả calibration

```bash
# Log 60 giây khi đứng yên, tính bias thực tế
sudo ./dm_imu_demo --no-config --bench 2>&1 | grep gyro_bias
# Mục tiêu: |gyro_bias| < 0.5 °/s sau calibration
```

### Reset heading về 0

```cpp
// Gọi khi robot đứng ở tư thế home stance
// Không cần setting mode, hiệu lực ngay, không cần save
imu.zeroAngle();
```

### Cấu hình tần số (nếu muốn thay đổi)

```cpp
imu.enterSettingMode();
imu.setOutputHz(500);   // 500 Hz thay vì 1000 Hz
imu.saveParams();
imu.exitSettingMode();
```

---

## 10. Cấu trúc thư mục

```
dm_imu_usb/
├── CMakeLists.txt
├── README.md
├── include/
│   └── dm_imu/
│       ├── bsp_crc.hpp            – CRC-8 / CRC-16 lookup table declarations
│       ├── imu_types.hpp          – ImuObservation (64B), LocomotionState, protocol constants
│       ├── imu_driver.hpp         – ImuDriver class (seqlock, PUSH/PULL API)
│       └── aimrt_imu_adapter.hpp  – AimRT pub/sub adapter (Option A & B)
└── src/
    ├── bsp_crc.cpp                – CRC implementation (ported từ DM firmware)
    ├── imu_driver.cpp             – Serial, readerLoop, seqlock, parser
    ├── main.cpp                   – Demo + benchmark (--bench flag)
    ├── calibrate.cpp              – Tool hiệu chỉnh gyro/accel
    ├── startup_pipeline.cpp       – Startup sequence cho robot controller
    └── main_startup.cpp           – Entry point cho startup pipeline
```

### File quan trọng nhất để đọc

| File | Đọc khi muốn hiểu... |
|---|---|
| `imu_types.hpp` | Struct dữ liệu, layout bộ nhớ, inline geometry |
| `imu_driver.hpp` | Toàn bộ public API |
| `imu_driver.cpp` | Seqlock implementation, SetRealTimeThread, frame parser |
| `aimrt_imu_adapter.hpp` | Cách integrate với AimRT channel |

---

## Lưu ý kỹ thuật

- **SCHED_FIFO** cần quyền `sudo` hoặc `CAP_SYS_NICE`. Nếu thiếu quyền, driver vẫn chạy nhưng in warning và dùng `SCHED_OTHER` (jitter cao hơn).
- **Yaw drift** là bình thường với 6-axis IMU (không có magnetometer). Dùng `yaw_rate` (rad/s) cho steering control, KHÔNG dùng `yaw_rad()` làm feedback dài hạn.
- **Accel dưới impact** (chân chạm đất): spike 2–5g trong ~1ms. Không đưa acc trực tiếp vào RL policy input nếu không có impact filter — dùng quat + gyro cho policy, acc cho contact detection.
- Driver **không có auto-reconnect** sau khi IMU restart. Nếu cần restart (sau calib), phải `close()` + `open()` lại.