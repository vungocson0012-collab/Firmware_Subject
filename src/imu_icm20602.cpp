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
#include <cstdint>
#include <cstdio>
#include "fc_runtime.h"
#include <cmath>
#include "config.h"
#include "imu_icm20602.h"

static void imu_calibrate_gyro();
static bool imu_read_raw();
static void imu_kalman_1d(float &state_deg, float &variance, float rate_dps, float measure_deg);
static void imu_write_reg(uint8_t reg, uint8_t value);
static uint8_t imu_read_reg(uint8_t reg);
static bool imu_read_accel_raw(int16_t &x, int16_t &y, int16_t &z);
static bool imu_read_gyro_raw(int16_t &x, int16_t &y, int16_t &z);
#ifdef CALIBRATE_ACCEL
static void imu_calibrate_accel();
#endif

// ============================================================================
//  ICM20602 - driver thanh ghi qua SPI, kèm bộ lọc Kalman 1D ước lượng góc.
//
//  Góc roll/pitch lấy từ Kalman 1D: gyro cho phần biến thiên nhanh, gia tốc kế
//  kéo về cho khỏi trôi. Bản đầy đủ dùng thêm Madgwick/Mahony để có góc yaw từ
//  la bàn, ở đây bỏ vì angle và alt hold không cần giữ hướng - yaw chỉ điều
//  khiển theo tốc độ xoay.
//
//  GHI CÔNG: cách ước lượng góc bằng bộ lọc Kalman một chiều ở hàm
//  imu_kalman_1d() học từ tài liệu của Carbon Aeronautics
//  (https://github.com/CarbonAeronautics). Chi tiết xem file LOI_NHAN_VA_GHI_CONG.md.
// ============================================================================

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

#define IMU_SPI_HZ   10000000     // 10 MHz

// Địa chỉ thanh ghi
#define REG_WHO_AM_I       0x75
#define REG_PWR_MGMT_1     0x6B
#define REG_ACCEL_XOUT_H   0x3B
#define REG_GYRO_XOUT_H    0x43
#define REG_GYRO_CONFIG    0x1B
#define REG_ACCEL_CONFIG   0x1C
#define REG_CONFIG         0x1A
#define REG_ACCEL_CONFIG2  0x1D

#define ICM20602_WHO_AM_I_VALUE  0x12

// Hệ số quy đổi số thô sang đơn vị vật lý
#define GYRO_LSB_PER_DPS   16.4f     // thang do +-2000 do/giay
#define ACC_LSB_PER_G      2048.0f   // thang do +-16g

static spi_device_handle_t imu_spi;

// Số đo thô đã quy đổi, CHƯA trừ bias
static float imu_acc_x_g, imu_acc_y_g, imu_acc_z_g;
static float imu_rate_roll_dps, imu_rate_pitch_dps, imu_rate_yaw_dps;

// Bias gyro đo được lúc khởi động
static float gyro_bias_roll_dps, gyro_bias_pitch_dps, gyro_bias_yaw_dps;

// Góc suy ra từ riêng gia tốc kế, dùng làm phép đo cho Kalman
static float acc_roll_deg, acc_pitch_deg;
static bool accel_angle_valid = false;

// Kết quả ước lượng góc
static float att_roll_deg  = 0.0f, att_roll_var  = 2 * 2;
static float att_pitch_deg = 0.0f, att_pitch_var = 2 * 2;


// Chỉ mở bus, chưa đụng gì tới cảm biến. Tách riêng để app_main() kiểm tra
// WHO_AM_I trước khi bước vào vòng hiệu chỉnh gyro.
void imu_init_bus() {
  spi_bus_config_t bus = {};
  bus.mosi_io_num = PIN_IMU_MOSI;
  bus.miso_io_num = PIN_IMU_MISO;
  bus.sclk_io_num = PIN_IMU_SCLK;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  bus.max_transfer_sz = 8;
  ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_DISABLED));
  spi_device_interface_config_t device = {};
  device.clock_speed_hz = IMU_SPI_HZ;
  device.mode = 0;
  device.spics_io_num = PIN_IMU_CS;
  device.queue_size = 1;
  ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &device, &imu_spi));
  gpio_config_t led = {};
  led.pin_bit_mask = 1ULL << PIN_IMU_LED;
  led.mode = GPIO_MODE_INPUT_OUTPUT;
  ESP_ERROR_CHECK(gpio_config(&led));
  gpio_set_level(static_cast<gpio_num_t>(PIN_IMU_LED), 0);
}

bool imu_is_present() {
  return (imu_read_reg(REG_WHO_AM_I) == ICM20602_WHO_AM_I_VALUE);
}


void imu_init() {
  imu_write_reg(REG_PWR_MGMT_1, 0x00);          // thoát chế độ ngủ
  fc_delay_ms(100);
  imu_write_reg(REG_ACCEL_CONFIG,  0x18);       // thang đo gia tốc +-16g
  imu_write_reg(REG_GYRO_CONFIG,   0x18);       // thang đo gyro +-2000 độ/giây
  imu_write_reg(REG_CONFIG,        IMU_DLPF_GYRO);
  imu_write_reg(REG_ACCEL_CONFIG2, IMU_DLPF_ACC);
  fc_delay_ms(100);

  puts("[ICM20602 khoi tao xong]");
  printf("CONFIG 0x%02X\n", imu_read_reg(REG_CONFIG));
  printf("ACCEL_CONFIG2 0x%02X\n", imu_read_reg(REG_ACCEL_CONFIG2));

#ifdef CALIBRATE_ACCEL
  imu_calibrate_accel();      // hàm này không bao giờ trả về
#endif

  imu_calibrate_gyro();
}


#ifdef CALIBRATE_ACCEL
// ===== Hiệu chỉnh gia tốc kế =====
// Chỉ chạy khi bật #define CALIBRATE_ACCEL. Đo xong thì in ra đúng ba dòng cần
// dán vào khối CẤU HÌNH rồi dừng hẳn - cố tình không cho bay ở chế độ này.
//
// Nguyên lý: đặt drone nằm phẳng thì đúng ra gia tốc kế phải đọc
// X = 0, Y = 0, Z = +1g. Lệch bao nhiêu so với ba số đó chính là sai số cần bù.
static void imu_calibrate_accel() {
  const int SAMPLES = 2000;          // 2000 mẫu x 2ms = 4 giây

  puts("");
  puts("=========================================");
  puts("     HIEU CHINH GIA TOC KE");
  puts("=========================================");
  puts("Dat drone nam phang tren mat ban phang.");
  puts("Bo tay ra, dung cham vao ban trong luc do.");

  for (int s = 5; s > 0; s--) {
    printf("Bat dau sau %d giay...\n", s);
    fc_delay_ms(1000);
  }
  puts("Dang do, giu yen...");

  double sum_x = 0, sum_y = 0, sum_z = 0;
  bool samples_valid = true;
  for (int i = 0; i < SAMPLES; i++) {
    if (!imu_read_raw()) samples_valid = false;
    sum_x += imu_acc_x_g;            // lấy số THÔ, chưa cộng offset cũ
    sum_y += imu_acc_y_g;
    sum_z += imu_acc_z_g;
    if ((i % 500) == 0) gpio_set_level(static_cast<gpio_num_t>(PIN_IMU_LED), !gpio_get_level(static_cast<gpio_num_t>(PIN_IMU_LED)));
    fc_delay_ms(2);
  }
  gpio_set_level(static_cast<gpio_num_t>(PIN_IMU_LED), 0);

  float mean_x = sum_x / SAMPLES;
  float mean_y = sum_y / SAMPLES;
  float mean_z = sum_z / SAMPLES;

  puts("");
  printf("Do duoc:  X = %+.4f   Y = %+.4f   Z = %+.4f  (g)\n",
                mean_x, mean_y, mean_z);
  puts("");

  // Kiểm tra tư thế trước khi tin kết quả
  if (!samples_valid) {
    puts("!! LOI: mau IMU khong hop le, khong nhan offset. Kiem tra va khoi dong lai.");
  } else if (mean_z < 0.80f) {
    puts("!! LOI: Z phai gan +1.00 g khi drone nam phang.");
    puts("   Z am  -> drone dang bi lat nguoc.");
    puts("   Z nho -> drone dang dung nghieng.");
    puts("   Dat lai cho dung roi khoi dong lai mach.");
  } else if (fabsf(mean_x) > 0.30f || fabsf(mean_y) > 0.30f) {
    puts("!! LOI: X hoac Y lech qua nhieu (>0.30 g).");
    puts("   Mat ban khong phang, hoac cam bien gan lech tren mach.");
    puts("   Kiem tra lai roi khoi dong lai mach.");
  } else {
    puts("Chep DUNG ba dong duoi day, thay vao khoi CAU HINH");
    puts("trong file include/config.h:");
    puts("");
    printf("#define ACC_OFFSET_X_G   %+.4ff\n", -mean_x);
    printf("#define ACC_OFFSET_Y_G   %+.4ff\n", -mean_y);
    printf("#define ACC_OFFSET_Z_G   %+.4ff\n", 1.0f - mean_z);
    puts("");
    puts("Xong thi TAT #define CALIBRATE_ACCEL di roi nap lai.");
    puts("Muon kiem tra: bat DEBUG_ACC_OFFSET, phai thay X~0 Y~0 Z~1.");
  }

  puts("=========================================");
  while (1) {                        // dừng hẳn, không cho bay ở chế độ cali
    gpio_set_level(static_cast<gpio_num_t>(PIN_IMU_LED), !gpio_get_level(static_cast<gpio_num_t>(PIN_IMU_LED)));
    fc_delay_ms(500);
  }
}
#endif


// Một cửa sổ 1 giây, lấy mẫu theo chu kỳ IMU; từ chối chuyển động/nhiễu lớn.
// Chuyển động rất chậm vẫn có thể giống bias: board phải được đặt đứng yên.
static bool imu_collect_gyro_bias(float mean[3]) {
  constexpr int samples = 500;
  double sum[6] = {}, square[6] = {};
  bool valid = true;
  for (int i = 0; i < samples; ++i) {
    if (!imu_read_raw()) valid = false;
    const float values[] = {imu_rate_roll_dps, imu_rate_pitch_dps, imu_rate_yaw_dps,
                            imu_acc_x_g, imu_acc_y_g, imu_acc_z_g};
    const float norm2 = values[3]*values[3] + values[4]*values[4] + values[5]*values[5];
    if (norm2 < 0.9f*0.9f || norm2 > 1.1f*1.1f) valid = false;
    for (int axis = 0; axis < 6; ++axis) {
      if (axis < 3 && fabsf(values[axis]) > GYRO_CAL_MAX_RATE_DPS) valid = false;
      sum[axis] += values[axis];
      square[axis] += double(values[axis]) * values[axis];
    }
    fc_delay_ms(PERIOD_CTRL_MS);
  }
  for (int axis = 0; axis < 6; ++axis) {
    const double average = sum[axis] / samples;
    const double variance = fmax(0.0, square[axis] / samples - average * average);
    const float limit = axis < 3 ? GYRO_CAL_MAX_STD_DPS : GYRO_CAL_MAX_ACC_STD_G;
    if (!std::isfinite(average) || !std::isfinite(variance) || variance > limit*limit) valid = false;
    if (axis < 3) mean[axis] = static_cast<float>(average);
  }
  return valid;
}

static void imu_calibrate_gyro() {
  float previous[3] = {}, mean[3] = {};
  bool previous_valid = false;
  puts("bat dau hieu chinh gyro - dat board dung yen");
  gpio_set_level(static_cast<gpio_num_t>(PIN_IMU_LED), 1);
  for (;;) {
    const bool valid = imu_collect_gyro_bias(mean);
    if (valid && previous_valid &&
        fabsf(mean[0] - previous[0]) < 0.08f &&
        fabsf(mean[1] - previous[1]) < 0.08f &&
        fabsf(mean[2] - previous[2]) < 0.08f) break;
    for (int axis = 0; axis < 3; ++axis) previous[axis] = mean[axis];
    previous_valid = valid;
    puts("Dang hieu chinh gyro - Khong di chuyen Drone!");
  }
  gyro_bias_roll_dps = mean[0];
  gyro_bias_pitch_dps = mean[1];
  gyro_bias_yaw_dps = mean[2];
  gpio_set_level(static_cast<gpio_num_t>(PIN_IMU_LED), 0);
  printf("bias gyro (do/giay) %.2f | %.2f | %.2f\n",
         gyro_bias_roll_dps, gyro_bias_pitch_dps, gyro_bias_yaw_dps);
}


// Gọi mỗi vòng điều khiển: đọc cảm biến rồi cập nhật ước lượng góc.
bool imu_update() {
  if (!imu_read_raw()) return false;
  if (!std::isfinite(att_roll_deg) || !std::isfinite(att_pitch_deg) ||
      !std::isfinite(att_roll_var) || !std::isfinite(att_pitch_var)) {
    att_roll_deg = acc_roll_deg; att_pitch_deg = acc_pitch_deg;
    att_roll_var = att_pitch_var = 4.0f;
  }
  if (accel_angle_valid) {
    imu_kalman_1d(att_roll_deg, att_roll_var,
                 imu_rate_roll_dps - gyro_bias_roll_dps, acc_roll_deg);
    imu_kalman_1d(att_pitch_deg, att_pitch_var,
                 imu_rate_pitch_dps - gyro_bias_pitch_dps, acc_pitch_deg);
  } else {
    // Gần 0 g không cung cấp hướng trọng lực. Nếu chip vẫn đáp ứng, chỉ dự
    // đoán bằng gyro thay vì chia 0 hoặc coi trạng thái rơi tự do là mất IMU.
    att_roll_deg += DT_CTRL * (imu_rate_roll_dps - gyro_bias_roll_dps);
    att_pitch_deg += DT_CTRL * (imu_rate_pitch_dps - gyro_bias_pitch_dps);
    att_roll_var += DT_CTRL * DT_CTRL;
    att_pitch_var += DT_CTRL * DT_CTRL;
  }
  return std::isfinite(att_roll_deg) && std::isfinite(att_pitch_deg);
}


static bool imu_read_raw() {
  int16_t ax_lsb, ay_lsb, az_lsb, gx_lsb, gy_lsb, gz_lsb;
  if (!imu_read_accel_raw(ax_lsb, ay_lsb, az_lsb) ||
      !imu_read_gyro_raw(gx_lsb, gy_lsb, gz_lsb)) return false;
  const float raw_x = float(ax_lsb) / ACC_LSB_PER_G;
  const float raw_y = float(ay_lsb) / ACC_LSB_PER_G;
  const float raw_z = float(az_lsb) / ACC_LSB_PER_G;
  // Kiểm tra cả raw để offset không che lấp bus trả toàn 0 hoặc 0xFF.
  const float raw_norm2 = raw_x*raw_x + raw_y*raw_y + raw_z*raw_z;
  if (raw_norm2 < 1e-6f && !imu_is_present()) return false;
  const float ax = raw_x + ACC_OFFSET_X_G;
  const float ay = raw_y + ACC_OFFSET_Y_G;
  const float az = raw_z + ACC_OFFSET_Z_G;
  const float norm2 = ax*ax + ay*ay + az*az;
  if (!std::isfinite(norm2)) return false;
  const bool angle_valid = raw_norm2 >= 1e-6f && norm2 >= 1e-6f;
  const float roll = angle_valid ? atan2f(ay, sqrtf(ax*ax + az*az)) * FC_RAD_TO_DEG : acc_roll_deg;
  const float pitch = angle_valid ? -atan2f(ax, sqrtf(ay*ay + az*az)) * FC_RAD_TO_DEG : acc_pitch_deg;
  if (!std::isfinite(roll) || !std::isfinite(pitch)) return false;
  imu_acc_x_g = raw_x; imu_acc_y_g = raw_y; imu_acc_z_g = raw_z;
  imu_rate_roll_dps = float(gx_lsb) / GYRO_LSB_PER_DPS;
  imu_rate_pitch_dps = float(gy_lsb) / GYRO_LSB_PER_DPS;
  imu_rate_yaw_dps = float(gz_lsb) / GYRO_LSB_PER_DPS;
  acc_roll_deg = roll; acc_pitch_deg = pitch;
  accel_angle_valid = angle_valid;
  return true;
}


// Kalman 1 chiều: gyro là đầu vào dự đoán, góc từ gia tốc kế là phép đo.
// var_process = 1 (độ/giây)^2, var_measure = 3^2 độ^2.
static void imu_kalman_1d(float &state_deg, float &variance,
                   float rate_dps, float measure_deg) {
  state_deg += DT_CTRL * rate_dps;
  variance  += DT_CTRL * DT_CTRL * 1.0f * 1.0f;

  float gain = variance / (variance + 3.0f * 3.0f);
  state_deg += gain * (measure_deg - state_deg);
  variance   = (1.0f - gain) * variance;
}


// ---- SPI mức thấp -----------------------------------------------------------
static void imu_write_reg(uint8_t reg, uint8_t value) {
  uint8_t tx[2] = {static_cast<uint8_t>(reg & 0x7F), value};
  spi_transaction_t transaction = {};
  transaction.length = sizeof(tx) * 8;
  transaction.tx_buffer = tx;
  ESP_ERROR_CHECK(spi_device_polling_transmit(imu_spi, &transaction));
}

static bool imu_read_regs(uint8_t reg, uint8_t *data, size_t len) {
  uint8_t tx[7] = {};
  uint8_t rx[7] = {};
  tx[0] = reg | 0x80;
  spi_transaction_t transaction = {};
  transaction.length = (len + 1) * 8;
  transaction.tx_buffer = tx;
  transaction.rx_buffer = rx;
  if (spi_device_polling_transmit(imu_spi, &transaction) != ESP_OK) return false;
  for (size_t i = 0; i < len; ++i) data[i] = rx[i + 1];
  return true;
}

static uint8_t imu_read_reg(uint8_t reg) {
  uint8_t value = 0;
  if (!imu_read_regs(reg, &value, 1)) return 0;
  return value;
}

static bool imu_read_axes(uint8_t reg, int16_t &x, int16_t &y, int16_t &z) {
  uint8_t data[6];
  if (!imu_read_regs(reg, data, sizeof(data))) return false;
  x = static_cast<int16_t>((data[0] << 8) | data[1]);
  y = static_cast<int16_t>((data[2] << 8) | data[3]);
  z = static_cast<int16_t>((data[4] << 8) | data[5]);
  return true;
}

static bool imu_read_accel_raw(int16_t &x, int16_t &y, int16_t &z) {
  return imu_read_axes(REG_ACCEL_XOUT_H, x, y, z);
}

static bool imu_read_gyro_raw(int16_t &x, int16_t &y, int16_t &z) {
  return imu_read_axes(REG_GYRO_XOUT_H, x, y, z);
}

// ---- Các hàm lấy dữ liệu ----------------------------------------------------
float imu_get_acc_x_g() { return imu_acc_x_g + ACC_OFFSET_X_G; }
float imu_get_acc_y_g() { return imu_acc_y_g + ACC_OFFSET_Y_G; }
float imu_get_acc_z_g() { return imu_acc_z_g + ACC_OFFSET_Z_G; }

float imu_get_rate_roll_dps()  { return imu_rate_roll_dps  - gyro_bias_roll_dps;  }
float imu_get_rate_pitch_dps() { return imu_rate_pitch_dps - gyro_bias_pitch_dps; }
float imu_get_rate_yaw_dps()   { return imu_rate_yaw_dps   - gyro_bias_yaw_dps;   }

float imu_get_roll_deg()  { return att_roll_deg;  }
float imu_get_pitch_deg() { return att_pitch_deg; }
