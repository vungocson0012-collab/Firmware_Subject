# Sửa lỗi luồng điều khiển — 08/11/2026

Bản sửa áp dụng trực tiếp cho **Fw_Subject**. GPIO, hệ số PID, chu kỳ task và
tham số Kalman độ cao giữ nguyên. Các thay đổi về điều kiện điều khiển và xử lý
lỗi được mô tả dưới đây; chưa xác nhận trên ESP32 hoặc phần cứng thật.

## Các lỗi đã xử lý

| Lỗi trong review | Thay đổi |
|---|---|
| Mutex RC bận làm ARM thành DISARM | Chỉ map công tắc khi lấy được snapshot mới; giữ nấc cũ khi mutex bận |
| Bỏ qua failsafe receiver | Bit failsafe làm RC không hợp lệ ngay; frame-lost không gia hạn tuổi mẫu tốt |
| Parser nhầm header trong payload | Thu đủ khung trước khi xét footer; chỉ tìm đồng bộ lại sau khung lỗi/timeout |
| Ga tụt khi vào ALT HOLD/FAILSAFE | Nạp trạng thái PID climb từ ga đang dùng; giới hạn tốc độ thay đổi ga độ cao |
| Không kiểm tra baro còn mới | Kiểm tra timestamp ở CTRL; thiếu baro không cho vào ALT HOLD; init phải đủ 50 mẫu reference hợp lệ |
| Mẫu IMU lỗi gây NaN | Kiểm tra SPI và số liệu trước filter; kiểm tra hữu hạn trước mixer; trả trạng thái lỗi về CTRL |
| Disarm chỉ dựa vào thời gian mất RC | Ga thấp và tốc độ đứng gần 0 phải duy trì liên tục 2 giây; reset timer khi điều kiện bị ngắt |
| Nhận chuyển động quay đều 10°/s làm bias | Hai cửa sổ ổn định, khoảng 1 giây/cửa sổ; giới hạn rate, phương sai gyro/accel và norm gia tốc |

Ngoài ra, CTRL tự timeout khi task RC ngừng publish. UART SBUS chỉ mở sau
hiệu chỉnh cảm biến để không đọc backlog lệnh từ lúc khởi động. Kênh SBUS
được chặn trong 1000–2000. Reset PID xóa cả lịch sử và khởi tạo lần đầu để
tránh xung đạo hàm do reset/rearm.

## Hành vi khi lỗi

| Tình huống | Xử lý |
|---|---|
| RC hợp lệ, baro thiếu hoặc mẫu quá 200 ms | Chạy ANGLE, ga trực tiếp từ người lái, kể cả khi CH6 yêu cầu ALT HOLD |
| Đang arm, RC mất nhưng baro còn hợp lệ | FAILSAFE với tốc độ hạ theo cấu hình |
| RC mất và baro không còn hợp lệ | Disarm, cả bốn ESC về `ESC_IDLE` |
| Đọc SPI lỗi hoặc dữ liệu/góc IMU không hữu hạn | Disarm ngay; chỉ arm lại sau khi dữ liệu tốt và công tắc ARM được gạt xuống rồi lên |
| Gia tốc gần 0 g, WHO_AM_I vẫn đúng | Chỉ dự đoán góc bằng gyro cho tới khi có hướng trọng lực dùng được; không chia cho 0 |
| Ga/tín hiệu PID không hữu hạn | Disarm trước phép chuyển số nguyên/mixer |

Fallback về ANGLE trả ga cho người lái, nên mức ga có thể khác ga tự động trước
đó. Khi baro có mẫu hợp lệ trở lại, mode được chọn lại theo CH6. Không có tự
khởi tạo lại chip baro nếu khởi tạo ban đầu thất bại.

## Tham số mới

Nằm trong [include/config.h](include/config.h):

- `RC_TIMEOUT_MS=200`, `BARO_TIMEOUT_MS=200`: tuổi mẫu tốt, không phải tuổi lần đọc shared state.
- `LANDING_CONFIRM_MS=2000`: thời gian liên tục thỏa điều kiện kết thúc failsafe.
- `ALT_THROTTLE_SLEW_PER_S=400`: duty/s; khoảng 0,8 duty mỗi vòng 2 ms ở ALT HOLD/FAILSAFE.
- `GYRO_CAL_MAX_RATE_DPS=3`, `GYRO_CAL_MAX_STD_DPS=0.2`,
  `GYRO_CAL_MAX_ACC_STD_G=0.02`: các ngưỡng hiệu chỉnh; kiểm tra cả norm gia tốc 0,9–1,1 g.

Gyro không thể tự phân biệt hoàn toàn một tốc độ quay rất chậm với bias; vẫn
phải đặt board đứng yên lúc bật nguồn. Timer hạ cánh xác nhận một điều kiện
ước lượng, không có cảm biến tiếp xúc mặt đất. Giới hạn tốc độ ga làm thay đổi
đáp ứng ALT HOLD/FAILSAFE và cần được kiểm chứng trên khung thực tế.

## Kiểm thử

Từ thư mục project:

```sh
python3 tests/run_host_tests.py
```

Runner biên dịch source firmware thật với stub driver/FreeRTOS trong
`tests/host/`. Dùng C++17, `-Wall -Wextra -Werror`; binary được đặt trong thư
mục tạm và tự dọn. Test bao gồm mutex bận, dữ liệu cũ, SBUS flags/payload/khung
thiếu, NaN, SPI/I2C lỗi, hiệu chỉnh lúc chuyển động, chuyển mode, khởi động/arm,
reset PID và bộ đếm thời gian đi qua `UINT32_MAX`.

Test Kalman hiện có vẫn đối chiếu 480000 lần cập nhật với mô hình ma trận.
Kết quả ngày 08/11/2026: **46 kịch bản PASS**, test Kalman **PASS**; sai số lớn
nhất so với mô hình tham chiếu là 0,007649 cm và 0,004303 cm/s.
Test host không kiểm tra bộ lập lịch ESP32, jitter 500 Hz, stack, bus vật lý,
xung PWM hay đáp ứng bay.

Build target còn cần thực hiện:

```sh
pio run -e esp32_fc
```

Đợt sửa này chưa chạy được build target vì yêu cầu quyền ghi cache PlatformIO
ngoài sandbox bị từ chối. Không dùng `firmware.bin` cũ làm bằng chứng bản sửa đã
build. Chưa nạp firmware. Kiểm tra trên bàn với cánh quạt tháo rời trước khi
đánh giá trên phần cứng.
