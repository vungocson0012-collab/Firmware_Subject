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
#include "driver/gpio.h"
#include "config.h"
#include "shared_state.h"
#include "tasks.h"
#include "imu_icm20602.h"
#include "baro_bmp388.h"
#include "altitude_kf.h"
#include "rc_sbus.h"
#include "esc_output.h"

static void debug_print();


extern "C" void app_main() {
  setvbuf(stdout, nullptr, _IOLBF, 0);

  esc_init();
  esc_write(ESC_IDLE, ESC_IDLE, ESC_IDLE, ESC_IDLE);

  // Kiểm tra IMU TRƯỚC khi hiệu chỉnh gyro. Bay với IMU chết là rơi ngay,
  // nên ở đây dừng hẳn chứ không chỉ in cảnh báo rồi chạy tiếp.
  imu_init_bus();
  sensor_present.imu = imu_is_present();
  printf("ICM20602 = %d\n", sensor_present.imu);
  if (!sensor_present.imu) {
    while (1) {
      puts("KHONG TIM THAY ICM20602 - dung lai, khong cho bay");
      gpio_set_level(static_cast<gpio_num_t>(PIN_IMU_LED), !gpio_get_level(static_cast<gpio_num_t>(PIN_IMU_LED)));
      fc_delay_ms(300);
    }
  }
  imu_init();

  sensor_present.baro = baro_init();
  printf("BMP388   = %d\n", sensor_present.baro);

  alt_kf_init();
  // Mở UART sau hiệu chỉnh để không xử lý lệnh cũ tích trong buffer lúc boot.
  rc_init();

  puts("KyThuatUAV FC - angle + alt hold");

  mtx_rc   = xSemaphoreCreateMutex();
  mtx_baro = xSemaphoreCreateMutex();
  mtx_tlm  = xSemaphoreCreateMutex();

  if (mtx_rc == NULL || mtx_baro == NULL || mtx_tlm == NULL) {
    puts("Loi: khong tao duoc Mutex!");
    while (1) fc_delay_ms(1000);
  }

  ESP_ERROR_CHECK(xTaskCreatePinnedToCore(rc_task, "RC", 3072, NULL, 4, NULL, 0)
                  == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
  ESP_ERROR_CHECK(xTaskCreatePinnedToCore(baro_task, "BARO", 4096, NULL, 2, NULL, 0)
                  == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
  ESP_ERROR_CHECK(xTaskCreatePinnedToCore(control_task, "CTRL", 8192, NULL, 5, NULL, 1)
                  == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

  // app_main returns after starting the application tasks.
}



// ============================================================================
//  rc_task  -  core 0, 500Hz
// ============================================================================
void rc_task(void *parameter) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    bool ok = rc_update();

    if (xSemaphoreTake(mtx_rc, 0) == pdTRUE) {
      shared_rc_ok = ok;
      shared_rc_frame_ms = rc_last_valid_frame_ms();
      for (int i = 1; i <= 16; i++) shared_rc_ch[i] = rc_get_channel(i);
      xSemaphoreGive(mtx_rc);
    }
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_RC_MS));
  }
}


// ============================================================================
//  baro_task  -  core 0, 100Hz. Kiêm luôn việc in debug.
// ============================================================================
void baro_task(void *parameter) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    if (sensor_present.baro) {
      // baro_update() chỉ trả true khi cảm biến có mẫu MỚI
      if (baro_update()) {
        float alt_m = baro_get_altitude_m();
        if (xSemaphoreTake(mtx_baro, 0) == pdTRUE) {
          shared_baro_alt_m = alt_m;
          shared_baro_valid = true;
          shared_baro_sample_ms = fc_time_ms();
          xSemaphoreGive(mtx_baro);
        }
      }
    }

    debug_print();

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_BARO_MS));
  }
}


// ============================================================================
//  In debug. Chạy ở core 0 priority thấp nên không ảnh hưởng vòng điều khiển.
// ============================================================================
static void debug_print() {
#if defined(DEBUG_ATTITUDE) || defined(DEBUG_RATE) || defined(DEBUG_ALTITUDE) || \
    defined(DEBUG_RC) || defined(DEBUG_ACC_OFFSET) || defined(DEBUG_LOOP_TIME)

  static uint32_t tick = 0;
  if (++tick % 5) return;              // 20Hz là đủ nhìn

  if (xSemaphoreTake(mtx_tlm, 0) != pdTRUE) return;

  #ifdef DEBUG_ATTITUDE
    printf("%.1f,%.1f,%.1f,%.1f\n",
                  tlm_roll_sp_deg, tlm_roll_deg, tlm_pitch_sp_deg, tlm_pitch_deg);
  #endif

  #ifdef DEBUG_RATE
    printf("%.0f,%.0f,%.0f,%.0f\n",
                  tlm_rate_roll_sp_dps,  tlm_rate_roll_dps,
                  tlm_rate_pitch_sp_dps, tlm_rate_pitch_dps);
  #endif

  #ifdef DEBUG_ALTITUDE
    printf("%.1f,%.1f,%.1f\n", tlm_alt_cm, tlm_climb_cms, tlm_climb_sp_cms);
  #endif

  #ifdef DEBUG_RC
    printf("ch1:%d ch2:%d ch3:%d ch4:%d arm:%d mode:%d | rc_ok:%d armed:%d fm:%d\n",
                  tlm_rc_ch[CH_ROLL], tlm_rc_ch[CH_PITCH],
                  tlm_rc_ch[CH_THROTTLE], tlm_rc_ch[CH_YAW],
                  tlm_rc_ch[CH_ARM], tlm_rc_ch[CH_MODE],
                  tlm_rc_ok, tlm_armed, tlm_flight_mode);
  #endif

  #ifdef DEBUG_ACC_OFFSET
    static float ax = 0, ay = 0, az = 1;
    ax = ax * 0.98f + tlm_acc_x_g * 0.02f;
    ay = ay * 0.98f + tlm_acc_y_g * 0.02f;
    az = az * 0.98f + tlm_acc_z_g * 0.02f;
    printf("X:%.4f Y:%.4f Z:%.4f\n", ax, ay, az);
  #endif

  #ifdef DEBUG_LOOP_TIME
    printf("loop_us:%u\n", static_cast<unsigned>(tlm_loop_us));
  #endif

  xSemaphoreGive(mtx_tlm);
#endif
}
