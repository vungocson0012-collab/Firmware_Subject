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
#include <cmath>
#include "config.h"
#include "altitude_kf.h"

// ============================================================================
//  Kalman 2 trạng thái ước lượng độ cao và tốc độ thẳng đứng.
//
//     trạng thái   S = [ z (cm) , vz (cm/s) ]'
//     đầu vào      u = gia tốc thẳng đứng (cm/s^2), từ IMU
//     phép đo      M = độ cao từ baro (cm)
//
//  Giữ nguyên dạng ma trận và bộ tham số của bản virex v14, chạy đúng nhịp
//  2ms của vòng điều khiển.
//
//  ------------------------------------------------------------------------
//  QUAN TRỌNG - vì sao gọi CẢ predict LẪN correct ở MỖI vòng 500Hz,
//  kể cả khi baro chưa có mẫu mới:
//
//  Bộ tham số này (sigma gia tốc = 4 cm/s^2, R = 30^2) cho ma trận P mọc rất
//  chậm, nên độ lợi Kalman K rất nhỏ. Nó chỉ kéo được ước lượng bám theo baro
//  nhờ ĐƯỢC ÁP DỤNG 500 LẦN MỖI GIÂY - cộng dồn lại mới đủ lực.
//
//  Từng thử đổi sang "chỉ correct khi baro có mẫu mới" (~50Hz) cho đúng lý
//  thuyết hơn. Kết quả: mất 9/10 số lần hiệu chỉnh mà K vẫn nhỏ như cũ, phần
//  kéo về gần như biến mất, vz thành tích phân gia tốc thuần và TRÔI VÔ HẠN.
//
//  Muốn làm bản đúng lý thuyết thì phải tăng sigma gia tốc lên cho P mọc
//  nhanh tương ứng (cỡ 3 lần khi giảm 10 lần số nhịp correct), rồi bay thử
//  lại từ đầu. Chưa kiểm chứng trên phần cứng thì ĐỪNG đổi.
//  ------------------------------------------------------------------------
// ============================================================================

// Expanded 2x2 matrix equations; no external matrix library is required.
static constexpr float sigma_acc = 4.0f;
static constexpr float sigma_baro = 30.0f;
static float z, vz;
static float p00, p01, p10, p11;

void alt_kf_init() {
  z = vz = 0.0f;
  p00 = p01 = p10 = p11 = 0.0f;
}

void alt_kf_update(float alt_meas_m, float acc_earth_z_g) {
  constexpr float dt = DT_CTRL;
  constexpr float g0 = 0.5f * dt * dt;
  constexpr float g1 = dt;
  constexpr float q = sigma_acc * sigma_acc;
  const float acc = acc_earth_z_g * 981.0f;
  z += dt * vz + g0 * acc;
  vz += g1 * acc;

  // P = F P F' + G G' sigma_acc^2.
  const float a = p00 + dt * (p01 + p10) + dt * dt * p11 + g0 * g0 * q;
  const float b = p01 + dt * p11 + g0 * g1 * q;
  const float c = p10 + dt * p11 + g1 * g0 * q;
  const float d = p11 + g1 * g1 * q;
  const float innovation_variance = a + sigma_baro * sigma_baro;
  const float k0 = a / innovation_variance;
  const float k1 = c / innovation_variance;
  const float innovation = alt_meas_m * 100.0f - z;
  z += k0 * innovation;
  vz += k1 * innovation;

  // P = (I - K H) P, H = [1, 0].
  p00 = (1.0f - k0) * a;
  p01 = (1.0f - k0) * b;
  p10 = c - k1 * a;
  p11 = d - k1 * b;
}

float alt_kf_get_altitude_cm() { return z; }
float alt_kf_get_climb_cms() { return vz; }
