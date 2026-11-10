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

void imu_init_bus();
bool imu_is_present();
void imu_init();
// false: không có mẫu IMU hợp lệ; trạng thái lọc không nhận mẫu lỗi.
bool imu_update();
float imu_get_acc_x_g();
float imu_get_acc_y_g();
float imu_get_acc_z_g();
float imu_get_rate_roll_dps();
float imu_get_rate_pitch_dps();
float imu_get_rate_yaw_dps();
float imu_get_roll_deg();
float imu_get_pitch_deg();
