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

float pid_rate_roll(float rate_dps, float rate_sp_dps);
float pid_rate_pitch(float rate_dps, float rate_sp_dps);
float pid_rate_yaw(float rate_dps, float rate_sp_dps);
float pid_tilt_roll(float angle_deg, float angle_sp_deg);
float pid_tilt_pitch(float angle_deg, float angle_sp_deg);
float pid_tilt_roll_output();
float pid_tilt_pitch_output();
float pid_climb(float climb_cms, float climb_sp_cms_in);
void pid_climb_seed(float output, float climb_cms, float climb_sp_cms);
void pid_reset_all();
void pid_bleed_integrators();
