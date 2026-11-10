# Mạch bay ESP32 — Kỹ Thuật UAV

Bản sửa ngày 08/11/2026: xử lý lỗi RC/SBUS, cảm biến, chuyển mode và disarm.
Xem [FIRMWARE_FIXES.md](FIRMWARE_FIXES.md) để biết thay đổi và giới hạn kiểm thử.
Chạy `python3 tests/run_host_tests.py` để kiểm thử logic trên host.

Mã nguồn mở của một bộ điều khiển bay (flight controller) cho drone bốn cánh,
chạy trên ESP32. Kèm tài liệu hướng dẫn từ lúc cài phần mềm tới lúc bay thử.

Dành cho sinh viên và người tự học. Miễn phí, không điều kiện.

---

## Phần cứng

| Thành phần | Linh kiện | Giao tiếp |
|---|---|---|
| Vi điều khiển | ESP32-WROOM-DA | — |
| Cảm biến quán tính | ICM-20602 | SPI, 10 MHz |
| Cảm biến áp suất | BMP388 | I2C |
| Bộ thu điều khiển | Bất kỳ loại nào ra SBUS | UART2, tín hiệu đảo |
| Động cơ | 4 ESC, xung PWM chuẩn | LEDC 391 Hz |

## Firmware làm được gì

- **Chế độ ANGLE** — cần gạt quy định góc nghiêng, thả cần thì drone tự về bằng.
- **Chế độ ALT HOLD** — cần ga quy định tốc độ lên xuống, để giữa thì giữ độ cao.
- **Tự hạ cánh khi mất sóng** — giữ thăng bằng, hạ đều 60 cm/s, chạm đất tự tắt máy.
- **Hiệu chỉnh gyro tự kiểm tra** — lặp lại tới khi drone thật sự đứng yên mới nhận.
- **Chế độ hiệu chỉnh gia tốc kế** — đo xong in ra sẵn ba dòng để dán vào code.

Chạy trên FreeRTOS, chia hai nhân của ESP32:

| Tác vụ | Nhân | Nhịp | Việc |
|---|---|---|---|
| `control_task` | 1 | 500 Hz | IMU, ước lượng góc, Kalman độ cao, PID, xuất ESC |
| `rc_task` | 0 | 500 Hz | Giải mã SBUS |
| `baro_task` | 0 | 100 Hz | Đọc BMP388, in dữ liệu gỡ lỗi |

Nhân 1 chỉ chạy vòng điều khiển, không chia sẻ với việc gì khác.

---

## Build trong VSCode

Firmware dùng **C++ (`.cpp` / `.h`)**, **ESP-IDF thuần** và FreeRTOS.
Mỗi file `.cpp` được biên dịch riêng; các hàm và dữ liệu dùng chung có khai báo
trong `include/`. Không còn phụ thuộc bước ghép sketch / tự sinh prototype của Arduino IDE.

1. Cài extension **PlatformIO IDE** (`platformio.platformio-ide`) trong VSCode.
2. Chọn **File → Open Folder**, mở thư mục `Fw_Subject` có `platformio.ini`.
3. PlatformIO dùng các gói đã cài nếu khớp phiên bản trong `platformio.ini`.
4. Vào **PlatformIO → Project Tasks → esp32_fc → General → Build**.
5. Khi cần nạp mạch, dùng **Upload**; mở **Monitor** để xem Serial ở 500000 baud.

Hoặc chạy trong **PlatformIO Core CLI** terminal:

```sh
pio run -e esp32_fc
pio run -e esp32_fc -t upload
pio device monitor -b 500000
```

Kết quả build nằm ở `.pio/build/esp32_fc/firmware.bin`.
Khi dùng **Upload** hoặc `pio run -e esp32_fc -t upload`, PlatformIO nạp cả
bootloader, bảng phân vùng và firmware vào đúng địa chỉ. Nếu có nhiều mạch,
thêm `--upload-port <cổng USB>` vào lệnh upload.
`platformio.ini` ghim platform `espressif32@6.9.0` với framework `espidf`
(ESP-IDF 5.3.1). Không dùng Arduino core hoặc BasicLinearAlgebra.
Điểm khởi động là `extern "C" void app_main()` trong `src/main.cpp`.
SPI, I2C, UART SBUS và PWM ESC dùng driver ESP-IDF; Kalman 2 trạng thái dùng
các phương trình ma trận khai triển trực tiếp.

Board `esp32dev`, flash **4 MB QIO 80 MHz**, CPU **240 MHz** dành cho
ESP32-WROOM-DA. Chân SPI được khai báo rõ trong `include/config.h`:
SCLK 18, MISO 19, MOSI 23, CS 5.
`sdkconfig.defaults` đặt FreeRTOS tick **1000 Hz** để giữ chu kỳ task 2 ms,
và UART console **500000 baud**. Sau khi đổi các giá trị mặc định này, xóa
`sdkconfig.esp32_fc` đã sinh rồi build lại để áp dụng.

Cấu trúc CMake và cấu hình framework theo
[tài liệu ESP-IDF của PlatformIO](https://docs.platformio.org/en/latest/frameworks/espidf.html).

Chân cắm, kênh điều khiển, hệ số PID, bù gia tốc và tùy chọn debug nằm trong
**[`include/config.h`](include/config.h)**.
Hướng dẫn chi tiết: **[Tài liệu kèm theo/HUONG_DAN_CAI_DAT.md](Tài%20liệu%20kèm%20theo/HUONG_DAN_CAI_DAT.md)**.

---

## Cấu trúc thư mục

```text
Fw_Subject/
├── platformio.ini              platform, board, ESP-IDF, console
├── CMakeLists.txt              cấu hình project ESP-IDF
├── sdkconfig.defaults          FreeRTOS tick, CPU, flash, console
├── include/
│   ├── config.h                chân cắm, PID, giới hạn, debug, chu kỳ task
│   ├── types.h                 kiểu dữ liệu dùng chung
│   ├── tasks.h                 khai báo các task FreeRTOS
│   ├── shared_state.h          khai báo extern và mutex
│   └── *.h                     API của các module cảm biến / điều khiển
├── src/
│   ├── CMakeLists.txt          đăng ký source và component ESP-IDF
│   ├── main.cpp                app_main, rc_task, baro_task, debug
│   ├── shared_state.cpp        định nghĩa dữ liệu dùng chung
│   ├── control_task.cpp        vòng điều khiển, chế độ bay, mixer
│   ├── imu_icm20602.cpp         driver IMU, ước lượng góc, hiệu chỉnh
│   ├── baro_bmp388.cpp          driver áp suất qua I2C
│   ├── altitude_kf.cpp         Kalman độ cao và tốc độ lên xuống
│   ├── pid.cpp                 PID xếp tầng
│   ├── rc_sbus.cpp             giải mã SBUS
│   └── esc_output.cpp          xuất PWM cho 4 ESC
├── CODE_FC/                    tài liệu Word của bản sketch cũ
├── Tài liệu kèm theo/          hướng dẫn, sơ đồ lắp, danh sách linh kiện
└── driver esp32/               driver USB CP210x cho Windows
```

Các file `.ino` cũ đã chuyển sang C++ trong `src/` và `include/`; firmware dùng
ESP-IDF thuần cho toàn bộ lớp truy cập phần cứng và khởi động.
Các tài liệu Word cũ được giữ để tham khảo, đường dẫn sketch trong đó là đường dẫn trước khi chuyển đổi.

Kiểm tra bộ lọc Kalman trên máy tính, không cần thư viện ngoài:

```sh
c++ -std=c++17 -Iinclude src/altitude_kf.cpp tests/altitude_kf_test.cpp -o /tmp/fc_kf_test
/tmp/fc_kf_test
```

Test đối chiếu 480000 lần cập nhật với mô hình ma trận, gồm trạng thái tĩnh,
bước độ cao, chuyển động sin và gia tốc có bias.

---

## An toàn

Firmware biên dịch được **không có nghĩa là bay được**. Trước khi lắp cánh, làm
đủ ba bước kiểm tra trong phần cuối tài liệu hướng dẫn:

1. Kiểm tra các kênh tay điều khiển đúng thứ tự
2. Kiểm tra thứ tự motor 1 → 2 → 3 → 4
3. Kiểm tra dấu của cảm biến và cần gạt **cùng chiều**

**Luôn tháo cánh quạt** khi thử nghiệm trên bàn.

---

## Đã bay nghiệm thu

Bộ hệ số PID mặc định trong code đã bay thật trên cấu hình:

Thông tin nghiệm thu này thuộc bản firmware gốc. Bản chuyển sang ESP-IDF
cần được kiểm thử lại trên phần cứng.

| | |
|---|---|
| Khung | 5 inch |
| Động cơ | 2204, 2300KV |
| Cánh | 5 inch |
| Pin | 3S 2200mAh |

Khung của bạn khác nhiều so với cấu hình này — nặng hơn, cánh to hơn, pin nhiều
cell hơn — thì nhiều khả năng phải chỉnh lại PID. Đó là chuyện bình thường, không
phải code sai.

Hướng chỉnh khi khung khác:

- **Khung to và nặng hơn** → giảm `KP_RATE_ROLL` và `KP_RATE_PITCH`. Khung càng
  nặng thì phản ứng càng chậm, giữ hệ số cao sẽ dao động.
- **Nghe motor rít tần số cao** → giảm `KD_RATE_ROLL` và `KD_RATE_PITCH`.
- **Lắc chậm biên độ lớn** → giảm `KP_RATE_*` trước, chưa hết thì giảm `KI_RATE_*`.

Lần bay đầu tiên luôn buộc dây và bay thấp, dù cấu hình có giống hệt.

[GPL-3.0](LICENSE).

Bạn được tự do dùng, sửa, và chia sẻ lại. Điều kiện: bản phát hành lại phải mở
mã nguồn và giữ nguyên phần ghi công.

## Ghi công

Bộ mã nguồn này học từ những người đi trước, và có ghi rõ học cái gì từ ai.
Xem **[Tài liệu kèm theo/LOI_NHAN_VA_GHI_CONG.md](Tài%20liệu%20kèm%20theo/LOI_NHAN_VA_GHI_CONG.md)**.

Trong đó cũng nói vì sao dự án này được công khai, và ba điều mong bạn giữ khi
dùng lại.

---

## Khoá học

Bộ mã nguồn ở đây là **bậc đầu tiên**: bay cân bằng. Miễn phí, đầy đủ, và tài
liệu viết để bạn tự đi hết được một mình. **Không cần học khoá nào cũng dùng
được trọn bộ này.**

Ai muốn đi nhanh hơn thì có khoá học, đi từ số 0, kèm 1-1:

| Bậc | Nội dung |
|---|---|
| 1 | **Lập trình bay cân bằng** — ước lượng góc, PID, mixer |
| 2 | **Lập trình giữ vị trí** — baro, lidar, optical flow, hợp nhất cảm biến |
| 3 | **Lập trình đa luồng và xử lý nhiễu** — làm drone hết rung, motor hết nóng, chỉnh PID sâu hơn mà không bị rít |
| 4 | **Lập trình hệ định vị và tự hành** |

Học trực tiếp tại TP.HCM, hoặc online cho các tỉnh thành khác.

Khoá học là để đi nhanh, an toàn và xa hơn — không phải để mở khoá thứ gì.

## Liên hệ

**Nguyễn Văn Quý** — Kỹ Thuật UAV
📞 0817 550 271 (gọi hoặc Zalo)

Lỗi trong mã nguồn thì mở [Issue](../../issues) trên GitHub — để người sau gặp
cùng lỗi còn đọc được câu trả lời.
