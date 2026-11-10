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
#include "shared_state.h"

// Trạng thái cảm biến lúc khởi động (kiểu khai báo trong types.h)
SensorPresent sensor_present;

// Mutex cho từng nhóm dữ liệu dùng chung
SemaphoreHandle_t mtx_rc;
SemaphoreHandle_t mtx_baro;
SemaphoreHandle_t mtx_tlm;

// rc_task ghi, control_task đọc
int  shared_rc_ch[17];
bool shared_rc_ok = false;
uint32_t shared_rc_frame_ms = 0;

// baro_task ghi, control_task đọc; timestamp chỉ đổi khi có mẫu hợp lệ mới.
float shared_baro_alt_m = 0.0f;
bool shared_baro_valid = false;
uint32_t shared_baro_sample_ms = 0;

// control_task ghi, baro_task đọc để in debug
float tlm_roll_deg, tlm_pitch_deg;
float tlm_roll_sp_deg, tlm_pitch_sp_deg;
float tlm_rate_roll_dps, tlm_rate_pitch_dps, tlm_rate_yaw_dps;
float tlm_rate_roll_sp_dps, tlm_rate_pitch_sp_dps;
float tlm_alt_cm, tlm_climb_cms, tlm_climb_sp_cms;
float tlm_acc_x_g, tlm_acc_y_g, tlm_acc_z_g;
int   tlm_rc_ch[17];
int   tlm_esc[5];
int   tlm_flight_mode;
bool  tlm_armed, tlm_rc_ok;
uint32_t tlm_loop_us;

