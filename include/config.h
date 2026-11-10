// ============================================================================
//  KyThuatUAV FC - firmware bay ESP32 (FreeRTOS), che do Angle + Alt Hold
//
//  Copyright (C) 2026  Nguyễn Văn Quý (Ky Thuat UAV)
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
//  Giu nguyen phan ghi cong nay khi dung lai hoac chinh sua ma nguon.
//
//  Nguon tham khao va ghi cong day du: xem file LOI_NHAN_VA_GHI_CONG.md
//  Lien he: Nguyen Van Quy - 0817 550 271 (Zalo)
// ============================================================================
#pragma once

// ---- Chân kết nối -----------------------------------------------------------
#define PIN_ESC_1        27
#define PIN_ESC_2        26
#define PIN_ESC_3        25
#define PIN_ESC_4        33

#define PIN_IMU_SCLK     18
#define PIN_IMU_MISO     19
#define PIN_IMU_MOSI     23
#define PIN_IMU_CS        5     // ICM20602 chip select (SPI)
#define PIN_IMU_LED      14     // LED báo trạng thái hiệu chỉnh gyro

#define PIN_SBUS_RX      35     // SBUS là tín hiệu đảo, dùng UART2

#define PIN_I2C_SDA      21
#define PIN_I2C_SCL      22
#define BARO_I2C_ADDR  0x76     // BMP388, nối kiểu 0x77 thì đổi ở đây

// ---- Kênh tay điều khiển ----------------------------------------------------
#define CH_ROLL           1
#define CH_PITCH          2
#define CH_THROTTLE       3
#define CH_YAW            4
#define CH_ARM            5     // công tắc arm, nấc trên cùng = arm
#define CH_MODE           6     // công tắc 3 nấc chọn chế độ bay

// ---- Chế độ bay và cách gán cho từng nấc công tắc --------------------------
#define MODE_ANGLE        0
#define MODE_ALT_HOLD     1
#define MODE_FAILSAFE     2     // dùng nội bộ, không gán cho công tắc

#define MODE_FOR_SW_0     MODE_ANGLE
#define MODE_FOR_SW_1     MODE_ANGLE
#define MODE_FOR_SW_2     MODE_ALT_HOLD

// ---- Giới hạn lệnh từ cần gạt ----------------------------------------------
#define LIM_TILT_DEG           20.0f   // độ nghiêng tối đa ở chế độ angle
#define LIM_YAW_RATE_DPS      100.0f   // tốc độ xoay tối đa, độ/giây
#define LIM_CLIMB_RATE_CMS    100.0f   // tốc độ lên xuống tối đa, cm/giây
#define FAILSAFE_DESCENT_CMS  -60.0f   // tốc độ hạ khi mất sóng, cm/giây

// Tuổi tối đa của mẫu hợp lệ, không phải thời điểm CTRL đọc shared state.
#define RC_TIMEOUT_MS           200U
#define BARO_TIMEOUT_MS         200U
#define LANDING_CONFIRM_MS     2000U
constexpr float ALT_THROTTLE_SLEW_PER_S = 400.0f; // duty/s; làm mượt ga khi đổi mode
constexpr float GYRO_CAL_MAX_RATE_DPS = 3.0f;
constexpr float GYRO_CAL_MAX_STD_DPS = 0.2f;
constexpr float GYRO_CAL_MAX_ACC_STD_G = 0.02f;

// ---- Giới hạn xung ra ESC ---------------------------------------------------
#define ESC_IDLE        800     // giá trị khi chưa arm
#define ESC_MIN_ARMED   900     // đã arm thì không cho tụt dưới mức này
#define ESC_MAX        1600

// ---- Hệ số PID --------------------------------------------------------------
// KP / KI / KD là ký hiệu chuẩn của khâu tỉ lệ, tích phân, vi phân.
// Đây là bộ số của chính khung này. fc_basic_loiter dùng KP_RATE_ROLL 0.4,
// KI 0.4, KD 0.03 vì khác khung và khác động cơ - không chép chéo qua lại.
constexpr float KP_RATE_ROLL  = 0.5f,  KI_RATE_ROLL  = 1.4f,  KD_RATE_ROLL  = 0.03f;
constexpr float KP_RATE_PITCH = 0.5f,  KI_RATE_PITCH = 1.4f,  KD_RATE_PITCH = 0.03f;
constexpr float KP_RATE_YAW   = 1.0f,  KI_RATE_YAW   = 10.0f, KD_RATE_YAW   = 0.0f;

constexpr float KP_TILT_ROLL  = 10.0f, KI_TILT_ROLL  = 0.0f,  KD_TILT_ROLL  = 0.0f;
constexpr float KP_TILT_PITCH = 10.0f, KI_TILT_PITCH = 0.0f,  KD_TILT_PITCH = 0.0f;

constexpr float KP_CLIMB      = 3.0f,  KI_CLIMB      = 15.0f, KD_CLIMB      = 0.0f;

// Chặn khâu tích phân và chặn đầu ra của từng tầng
constexpr float I_LIM_RATE   = 200.0f,  U_LIM_RATE   = 400.0f;
constexpr float I_LIM_TILT   = 200.0f,  U_LIM_TILT   = 400.0f;
constexpr float I_LIM_YAW    = 100.0f,  U_LIM_YAW    = 100.0f;
constexpr float I_LIM_CLIMB  = 700.0f,  P_LIM_CLIMB  = 200.0f;

// ---- Bù lệch gia tốc kế (đơn vị g) -----------------------------------------
// Cách đo: bật DEBUG_ACC_OFFSET bên dưới, đặt drone nằm phẳng, chờ số ổn định,
// rồi điền:  X = -giá_trị_X,  Y = -giá_trị_Y,  Z = 1 - giá_trị_Z
#define ACC_OFFSET_X_G   0.00f
#define ACC_OFFSET_Y_G   0.00f
#define ACC_OFFSET_Z_G   0.00f

// ---- Bộ lọc thông thấp bên trong ICM20602 ----------------------------------
// 0x04=20Hz  0x05=10Hz  0x06=5.1Hz  0x07=2.5Hz  (số nhỏ = ít trễ, nhiều nhiễu)
#define IMU_DLPF_GYRO    0x06
#define IMU_DLPF_ACC     0x05

// #define CALIBRATE_ACCEL    // Hiệu chỉnh gia tốc kế rồi dừng, không chạy task bay

// ---- Debug ------------------------------------------------------------------
// Mỗi lần chỉ bật MỘT dòng. Việc in do baro_task lo, không bao giờ in từ
// vòng điều khiển, nên bật debug không làm lệch nhịp PID.
// #define DEBUG_ATTITUDE      // góc hiện tại so với góc mong muốn
// #define DEBUG_RATE          // tốc độ góc mong muốn so với thực tế
// #define DEBUG_ALTITUDE      // độ cao và tốc độ lên xuống sau KF
// #define DEBUG_RC            // giá trị các kênh tay điều khiển
// #define DEBUG_ACC_OFFSET    // để đo ACC_OFFSET_* bên trên
// #define DEBUG_LOOP_TIME     // chu kì vòng điều khiển, phải luôn ~2000 us

// ============================================================================
//                        HẾT PHẦN CẤU HÌNH
// ============================================================================

#define PERIOD_CTRL_MS    2       // 500 Hz
#define PERIOD_RC_MS      2       // 500 Hz
#define PERIOD_BARO_MS   10       // 100 Hz
#define DT_CTRL      0.002f       // chu kì vòng điều khiển, tính bằng giây
