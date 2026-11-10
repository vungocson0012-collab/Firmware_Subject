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
#include <cstring>
#include "config.h"
#include "rc_sbus.h"

// ============================================================================
//  Giải mã SBUS.
//  SBUS là tín hiệu ĐẢO, 100000 baud, khung 8E2. ESP32 đảo được bằng phần cứng
//  nên chỉ cần bật cờ invert lúc mở UART, không cần mạch đảo bên ngoài.
//
//  Một khung SBUS dài 25 byte: 0x0F, 22 byte dữ liệu (16 kênh x 11 bit),
//  1 byte cờ, 1 byte kết thúc 0x00.
// ============================================================================

#include "driver/uart.h"

static constexpr uart_port_t SBUS_UART = UART_NUM_2;

#define SBUS_FRAME_LEN     25
#define SBUS_RAW_MIN      173     // giá trị thô ứng với cần gạt hết về một phía
#define SBUS_RAW_MAX     1811
#define SBUS_OUT_MIN      990     // quy đổi ra dải quen thuộc 1000-2000
#define SBUS_OUT_MAX     2010
#define SBUS_FRAME_TIMEOUT_MS 10U  // khung 25 byte / 100 kbaud mất khoảng 3 ms

static uint8_t  sbus_frame[SBUS_FRAME_LEN];
static unsigned int sbus_byte_index;
static unsigned int sbus_raw_ch[17];
static unsigned int sbus_ch_us[17];
static bool sbus_have_valid_frame, sbus_failsafe;
static uint32_t sbus_last_byte_ms;
static uint32_t sbus_last_frame_ms;


void rc_init() {
  uart_config_t config = {};
  config.baud_rate = 100000;
  config.data_bits = UART_DATA_8_BITS;
  config.parity = UART_PARITY_EVEN;
  config.stop_bits = UART_STOP_BITS_2;
  config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  config.source_clk = UART_SCLK_DEFAULT;
  ESP_ERROR_CHECK(uart_param_config(SBUS_UART, &config));
  ESP_ERROR_CHECK(uart_set_pin(SBUS_UART, UART_PIN_NO_CHANGE, PIN_SBUS_RX,
                             UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_set_line_inverse(SBUS_UART, UART_SIGNAL_RXD_INV));
  ESP_ERROR_CHECK(uart_driver_install(SBUS_UART, 512, 0, 0, nullptr, 0));
  ESP_ERROR_CHECK(uart_flush_input(SBUS_UART));
  sbus_byte_index = 0;
  sbus_have_valid_frame = false;
  sbus_failsafe = false;
  sbus_last_frame_ms = sbus_last_byte_ms = 0;
  for (int i = 1; i <= 16; ++i) sbus_ch_us[i] = 1500;
  sbus_ch_us[CH_THROTTLE] = 1000;
  sbus_ch_us[CH_ARM] = sbus_ch_us[CH_MODE] = 1000;
}


// Header chỉ có ý nghĩa khi tìm đầu khung, không phải trong payload.
static bool sbus_footer_valid(uint8_t byte) {
  return byte == 0 || byte == 0x04 || byte == 0x14 || byte == 0x24 || byte == 0x34;
}

static bool sbus_read_frame() {
  if (sbus_byte_index && fc_time_ms() - sbus_last_byte_ms > SBUS_FRAME_TIMEOUT_MS)
    sbus_byte_index = 0;
  uint8_t byte;
  // Giới hạn công việc ngay cả khi UART nhận nhiễu liên tục.
  for (unsigned budget = 0; budget < 64; ++budget) {
    if (uart_read_bytes(SBUS_UART, &byte, 1, 0) != 1) break;
    sbus_last_byte_ms = fc_time_ms();
    if (sbus_byte_index == 0 && byte != 0x0F) continue;
    sbus_frame[sbus_byte_index++] = byte;
    if (sbus_byte_index != SBUS_FRAME_LEN) continue;
    if (sbus_footer_valid(byte) && (sbus_frame[23] & 0xF0) == 0) {
      sbus_byte_index = 0;
      return true;
    }
    // Chỉ tìm lại header sau khi toàn bộ khung đã bị xác định hỏng.
    unsigned next = 1;
    while (next < SBUS_FRAME_LEN && sbus_frame[next] != 0x0F) ++next;
    sbus_byte_index = SBUS_FRAME_LEN - next;
    std::memmove(sbus_frame, sbus_frame + next, sbus_byte_index);
  }
  return false;
}

// Tách 16 kênh, mỗi kênh 11 bit, xếp liên tục không theo biên byte
static void sbus_decode_channels() {
  int bit_ptr  = 0;
  int byte_ptr = 1;
  sbus_raw_ch[0] = sbus_frame[23];

  for (int chan = 1; chan <= 16; chan++) {
    sbus_raw_ch[chan] = 0;
    for (int bit = 0; bit < 11; bit++) {
      sbus_raw_ch[chan] |= ((sbus_frame[byte_ptr] >> bit_ptr) & 1) << bit;
      if (++bit_ptr > 7) {
        bit_ptr = 0;
        byte_ptr++;
      }
    }
  }
}


// Trả về true khi vẫn còn sóng, false khi mất sóng.
bool rc_update() {
  // Đọc tới khung mới nhất trong backlog hữu hạn, không lặp vô hạn khi UART bận.
  for (unsigned frames = 0; frames < 8 && sbus_read_frame(); ++frames) {
    const uint8_t flags = sbus_frame[23];
    if (flags & 0x08) { sbus_failsafe = true; continue; }
    // Frame-lost không được gia hạn tuổi mẫu hoặc ghi đè lệnh tốt trước đó.
    if (flags & 0x04) continue;
    sbus_decode_channels();
    for (int i = 1; i <= 16; ++i) {
      int mapped = (static_cast<int>(sbus_raw_ch[i]) - SBUS_RAW_MIN) *
                   (SBUS_OUT_MAX - SBUS_OUT_MIN) / (SBUS_RAW_MAX - SBUS_RAW_MIN) + SBUS_OUT_MIN;
      sbus_ch_us[i] = mapped < 1000 ? 1000 : mapped > 2000 ? 2000 : mapped;
    }
    sbus_last_frame_ms = fc_time_ms();
    sbus_have_valid_frame = true;
    sbus_failsafe = false;
  }
  return sbus_have_valid_frame && !sbus_failsafe &&
         fc_time_ms() - sbus_last_frame_ms <= RC_TIMEOUT_MS;
}

uint32_t rc_last_valid_frame_ms() { return sbus_last_frame_ms; }
int rc_get_channel(int index) {
  return index >= 1 && index <= 16 ? sbus_ch_us[index] : 1500;
}
