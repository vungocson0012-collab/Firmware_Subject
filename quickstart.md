# Quickstart — ESP32 Flight Controller

Firmware **ESP-IDF + FreeRTOS**, build/nạp bằng **PlatformIO trong VS Code**.
Chân dưới đây khớp với [`include/config.h`](include/config.h).

## 1. Linh kiện cần có

| Linh kiện | Số lượng | Chức năng / giao thức |
|---|---:|---|
| Board ESP32-WROOM-DA, flash 4 MB | 1 | Bộ điều khiển chính; profile `esp32dev` |
| Module ICM-20602 | 1 | Gia tốc + gyro; SPI mode 0, 10 MHz |
| Module BMP388 | 1 | Áp suất / độ cao; I2C 400 kHz, mặc định `0x76` |
| Tay điều khiển + receiver có ngõ SBUS | 1 bộ | UART2, 100000 baud, 8E2, tín hiệu đảo |
| ESC nhận PWM thông thường + motor brushless | 4 bộ | LEDC 391 Hz, 11 bit; xung khoảng 1000–2000 µs |
| Nguồn ổn áp/BEC, pin và mạch phân phối nguồn | Theo bộ máy | Cấp nguồn phù hợp cho board, receiver và ESC |
| Cáp USB truyền dữ liệu, dây nối | 1 bộ | Nạp chương trình và đấu nối |
| LED + điện trở nối tiếp 1 kΩ | 1 bộ, tùy chọn | Báo hiệu chỉnh IMU trên GPIO14 |

## 2. Đấu nối tín hiệu

**Các số bên dưới là số GPIO, không phải thứ tự chân trên hàng header.**

| Thiết bị | Chân trên thiết bị | Nối với ESP32 |
|---|---|---|
| ICM-20602 | SCK / SCLK | GPIO18 |
| ICM-20602 | SDO / MISO | GPIO19 |
| ICM-20602 | SDI / MOSI | GPIO23 |
| ICM-20602 | CS / NCS | GPIO5 |
| BMP388 | SDA / SDI | GPIO21 |
| BMP388 | SCL / SCK | GPIO22 |
| Receiver | SBUS OUT | GPIO35; firmware tự đảo tín hiệu UART |
| ESC 1 | Signal | GPIO27 |
| ESC 2 | Signal | GPIO26 |
| ESC 3 | Signal | GPIO25 |
| ESC 4 | Signal | GPIO33 |
| LED trạng thái | Anode (+) | GPIO14 qua điện trở 1 kΩ; cathode (−) về GND |
| Tất cả thiết bị | GND | Chung GND với ESP32 |

**BMP388:** chọn chế độ I2C theo module; CSB ở mức VDDIO khi cấp nguồn.
SDO nối GND chọn `0x76`, nối VDDIO chọn `0x77`; nếu dùng `0x77`, sửa
`BARO_I2C_ADDR` trong `include/config.h`. SDA/SCL cần điện trở kéo lên mức
logic 3,3 V; kiểm tra module đã có sẵn chưa.
Tham khảo [datasheet BMP388 của Bosch](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp388-ds001.pdf).

## 3. Sơ đồ nguồn tổng quát

```text
Pin ── mạch phân phối nguồn ── 4 ESC ── 4 motor
  └── nguồn ổn áp/BEC ─────── ESP32 + receiver
ESP32 / nguồn 3,3 V phù hợp ── ICM-20602 + BMP388

GND ESP32 ── GND cảm biến ── GND receiver ── GND ESC/BEC
```

- Chọn điện áp cấp theo **đúng module/board đang dùng**; chân `VIN` và `3V3`
  có yêu cầu khác nhau. Không cấp pin trực tiếp vào cảm biến hoặc GPIO.
- Tín hiệu vào ESP32 dùng mức **3,3 V**; nếu SBUS OUT là 5 V, dùng mạch chuyển
  mức phù hợp trước GPIO35. Xem [thông số điện ESP32](https://documentation.espressif.com/esp32_datasheet_en.html).
- Motor lấy nguồn qua ESC, không lấy từ ESP32. Nếu ESC có dây BEC (+), bố trí
  nguồn trước khi nối; tránh đấu song song tùy tiện các ngõ ra BEC.

## 4. Build, nạp và chạy lần đầu

Mở thư mục `Fw_Subject` bằng VS Code, dùng terminal **PlatformIO Core CLI**:

```sh
pio run -e esp32_fc
pio run -e esp32_fc -t upload
pio device monitor -b 500000
```

Nếu có nhiều thiết bị USB, thêm `--upload-port <cổng>` vào lệnh nạp.
Cấu hình hiện tại: `espressif32@6.9.0`, ESP-IDF 5.3.1, CPU 240 MHz,
flash 4 MB QIO 80 MHz. Firmware nằm ở `.pio/build/esp32_fc/firmware.bin`;
lệnh Upload tự nạp cả bootloader và bảng phân vùng.

**Tháo cánh quạt khi kiểm tra trên bàn.** Đặt board nằm yên khi khởi động để
hiệu chỉnh gyro; nếu không tìm thấy ICM-20602, firmware sẽ dừng khởi tạo.
Kiểm tra riêng thứ tự ESC 1–4, chiều motor và chiều đo roll/pitch trước khi bay.

Kênh receiver mặc định: **CH1 roll, CH2 pitch, CH3 ga, CH4 yaw, CH5 arm,
CH6 mode**. CH6 nấc thấp/giữa là ANGLE, nấc cao là ALT HOLD.
Khi bật nguồn, gạt arm xuống trước rồi mới gạt lên để arm.

Chỉnh chân, PID, bù gia tốc và bật từng tùy chọn `DEBUG_*` tại
[`include/config.h`](include/config.h). Bản sửa luồng điều khiển ngày 08/11/2026
đã có test host; chưa xác nhận lại build ESP32 và chưa kiểm thử phần cứng thật.
Xem [FIRMWARE_FIXES.md](FIRMWARE_FIXES.md) để biết hành vi khi RC/baro/IMU lỗi
và các tham số mới. Khi baro không hợp lệ, yêu cầu ALT HOLD sẽ trở về ANGLE.

Chạy kiểm thử logic trên máy tính:

```sh
python3 tests/run_host_tests.py
```
