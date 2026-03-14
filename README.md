# dm_imu_usb — C++ Driver (No ROS)

Driver C++ thuần cho **DM-IMU** (达妙), đọc dữ liệu qua cổng **USB-CDC**  
(`/dev/ttyACM0` trên Linux) và cung cấp API đơn giản `getObs()`.

---

## Cấu trúc thư mục

```
dm_imu_usb/
├── CMakeLists.txt
├── README.md
├── include/
│   └── dm_imu/
│       ├── bsp_crc.hpp      – CRC-8 / CRC-16 lookup table
│       ├── imu_types.hpp    – Struct dữ liệu & hằng số giao thức
│       └── imu_driver.hpp   – API driver
└── src/
    ├── bsp_crc.cpp          – Implement CRC
    ├── imu_driver.cpp       – Serial + parser + background thread
    └── main.cpp             – Demo / ví dụ sử dụng
```

---

## Giao thức USB

Thiết bị kết nối như USB CDC-ACM (`/dev/ttyACM0`), baudrate **921600**.

### Bố cục frame (19 byte)

| Byte | Nội dung                          |
|------|-----------------------------------|
| 0    | `0x55` – Frame Header             |
| 1    | `0xAA` – Frame Header             |
| 2    | `slave_id` (thường = `0x01`)      |
| 3    | **RID** – Register ID             |
| 4–7  | `float32` LE – giá trị 1          |
| 8–11 | `float32` LE – giá trị 2          |
| 12–15| `float32` LE – giá trị 3          |
| 16–17| `CRC16` LE (tính trên byte 0–15)  |
| 18   | `0x0A` – Frame Tail               |

### RID mapping

| RID    | Dữ liệu                      | Đơn vị |
|--------|------------------------------|--------|
| `0x01` | Accel X, Y, Z                | m/s²   |
| `0x02` | Gyro X, Y, Z                 | °/s    |
| `0x03` | Roll, Pitch, Yaw (Euler)     | độ     |

Quaternion được tính từ Euler theo quy ước **ZYX intrinsic**.

---

## API chính

```cpp
#include "dm_imu/imu_driver.hpp"

dm_imu::ImuDriver imu("/dev/ttyACM0", 921600);
imu.open();   // cấu hình IMU + khởi động background thread

// Gọi ở bất kỳ tần số nào (thread-safe)
dm_imu::ImuObservation obs = imu.getObs();

if (obs.valid) {
    // Gia tốc kế (m/s²)
    obs.accel.x; obs.accel.y; obs.accel.z;

    // Tốc độ góc (°/s)
    obs.angular_velocity.x;
    obs.angular_velocity.y;
    obs.angular_velocity.z;

    // Quaternion (ZYX, normalized)
    obs.quaternion.w; obs.quaternion.x;
    obs.quaternion.y; obs.quaternion.z;

    // Góc Euler gốc (độ)
    obs.roll; obs.pitch; obs.yaw;
}

imu.close();
```

---

## Build

```bash
# Yêu cầu: CMake >= 3.14, g++ >= 9 (C++17)

mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# Chạy demo
./dm_imu_demo /dev/ttyACM0 921600
```

### Quyền truy cập cổng serial

```bash
sudo usermod -aG dialout $USER
# hoặc tạm thời:
sudo chmod 666 /dev/ttyACM0
```

---

## Tích hợp vào project khác

Thêm thư mục này vào CMake project của bạn:

```cmake
add_subdirectory(dm_imu_usb)
target_link_libraries(your_target PRIVATE dm_imu_lib)
```

Rồi `#include "dm_imu/imu_driver.hpp"` và gọi `getObs()`.

---

## Lưu ý

- Driver **không phụ thuộc ROS**, chỉ dùng POSIX API (Linux/macOS).
- Background thread đọc ở tốc độ tối đa (~1000 Hz firmware), `getObs()`  
  trả về snapshot **mới nhất** tại thời điểm gọi (zero-copy nếu dùng `const&`).
- CRC-16 CCITT tính trên **toàn bộ 16 byte** (kể cả header `0x55 0xAA`),  
  với fallback không-header cho firmware cũ.
