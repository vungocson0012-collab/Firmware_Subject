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
#include "esc_output.h"
#include "driver/ledc.h"

// ============================================================================
//  Xuất xung PWM cho 4 ESC.
//  Tần số 391Hz với độ phân giải 11 bit: giá trị 800-1600 ứng với xung
//  1000-2000 us, đúng chuẩn PWM của ESC thường.
// ============================================================================

#define ESC_PWM_FREQ_HZ    391
#define ESC_PWM_BITS        11

static const int esc_pins[] = {PIN_ESC_1, PIN_ESC_2, PIN_ESC_3, PIN_ESC_4};

void esc_init() {
  ledc_timer_config_t timer = {};
  timer.speed_mode = LEDC_HIGH_SPEED_MODE;
  timer.duty_resolution = LEDC_TIMER_11_BIT;
  timer.timer_num = LEDC_TIMER_0;
  timer.freq_hz = ESC_PWM_FREQ_HZ;
  timer.clk_cfg = LEDC_USE_APB_CLK;
  ESP_ERROR_CHECK(ledc_timer_config(&timer));
  for (int i = 0; i < 4; ++i) {
    ledc_channel_config_t channel = {};
    channel.gpio_num = esc_pins[i];
    channel.speed_mode = LEDC_HIGH_SPEED_MODE;
    channel.channel = static_cast<ledc_channel_t>(i);
    channel.timer_sel = LEDC_TIMER_0;
    channel.duty = ESC_IDLE;
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
  }

  esc_write(ESC_IDLE, ESC_IDLE, ESC_IDLE, ESC_IDLE);
  fc_delay_ms(100);
  esc_write(ESC_IDLE, ESC_IDLE, ESC_IDLE, ESC_IDLE);
  fc_delay_ms(100);

  // THÁO CÁNH rồi mở khối này để dò chân nào ra motor nào.
  // Mỗi motor sẽ quay nhẹ lần lượt theo đúng thứ tự 1 -> 2 -> 3 -> 4.
  //   esc_write(900, 800, 800, 800);  fc_delay_ms(200);
  //   esc_write(800, 900, 800, 800);  fc_delay_ms(200);
  //   esc_write(800, 800, 900, 800);  fc_delay_ms(200);
  //   esc_write(800, 800, 800, 900);  fc_delay_ms(200);
  //   esc_write(800, 800, 800, 800);  fc_delay_ms(1000);
}

void esc_write(int m1, int m2, int m3, int m4) {
  const int duty[] = {m1, m2, m3, m4};
  for (int i = 0; i < 4; ++i) {
    const auto channel = static_cast<ledc_channel_t>(i);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, duty[i]));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel));
  }
}
