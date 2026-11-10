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

#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "types.h"

// Defined once in src/shared_state.cpp; access through the matching mutex.
extern SensorPresent sensor_present;
extern SemaphoreHandle_t mtx_rc;
extern SemaphoreHandle_t mtx_baro;
extern SemaphoreHandle_t mtx_tlm;
extern int  shared_rc_ch[17];
extern bool shared_rc_ok;
extern uint32_t shared_rc_frame_ms;
extern float shared_baro_alt_m;
extern bool shared_baro_valid;
extern uint32_t shared_baro_sample_ms;
extern float tlm_roll_deg, tlm_pitch_deg;
extern float tlm_roll_sp_deg, tlm_pitch_sp_deg;
extern float tlm_rate_roll_dps, tlm_rate_pitch_dps, tlm_rate_yaw_dps;
extern float tlm_rate_roll_sp_dps, tlm_rate_pitch_sp_dps;
extern float tlm_alt_cm, tlm_climb_cms, tlm_climb_sp_cms;
extern float tlm_acc_x_g, tlm_acc_y_g, tlm_acc_z_g;
extern int   tlm_rc_ch[17];
extern int   tlm_esc[5];
extern int   tlm_flight_mode;
extern bool  tlm_armed, tlm_rc_ok;
extern uint32_t tlm_loop_us;
