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
#include "tasks.h"
#include "shared_state.h"
#include "imu_icm20602.h"
#include "altitude_kf.h"
#include "pid.h"
#include "esc_output.h"

static void mode_angle();
static void mode_alt_hold();
static void mode_failsafe();
static void baro_fetch_shared();
static int rc_switch_position(int ch_value);
static void rc_fetch_and_map();
static int mix_to_esc(float duty);
static void fault_disarm();
static float climb_throttle(float setpoint);
static void telemetry_snapshot();

// ============================================================================
//  control_task  -  core 1, 500Hz. Đây là vòng điều khiển chính.
//
//  Thứ tự mỗi vòng:
//     đọc IMU -> ước lượng góc -> gia tốc thẳng đứng -> KF độ cao
//     -> lệnh tay điều khiển -> arm/mode -> PID -> mixer -> xuất ESC
// ============================================================================

// ---- Trạng thái đo được -----------------------------------------------------
static float acc_x_g,  acc_y_g,  acc_z_g;          // gia tốc thân máy, đơn vị g
static float rate_roll_dps, rate_pitch_dps, rate_yaw_dps;   // tốc độ góc, độ/giây
static float roll_deg, pitch_deg;                  // góc nghiêng đã lọc
static float acc_earth_z_g;                        // gia tốc thẳng đứng đã trừ trọng lực

// ---- Giá trị mong muốn ------------------------------------------------------
static float roll_sp_deg, pitch_sp_deg;
static float yaw_rate_sp_dps;
static float climb_sp_cms;

// ---- Đầu ra bộ điều khiển ---------------------------------------------------
// u = tín hiệu điều khiển, ký hiệu chuẩn trong lý thuyết điều khiển
static float u_roll, u_pitch, u_yaw;
static float u_throttle = ESC_IDLE;
static float altitude_throttle = ESC_IDLE;

// ---- Đầu ra cuối cùng cho 4 ESC ---------------------------------------------
static int esc_1, esc_2, esc_3, esc_4;
static int esc_lim_lo = ESC_IDLE, esc_lim_hi = ESC_IDLE;

// ---- Trạng thái bay ---------------------------------------------------------
static int  rc_ch[17];
static bool rc_ok;
static bool rc_cached_ok;
static uint32_t rc_frame_ms;
static bool armed = false;
static int  flight_mode = MODE_ANGLE;

// Khởi tạo bằng 2 chứ không phải -1: nếu cắm nguồn lúc công tắc arm ĐANG bật,
// điều kiện "vừa vào nấc 2" sẽ không thoả, nên drone không tự arm ngay khi
// khởi động. Muốn arm phải gạt công tắc xuống rồi gạt lên lại.
static int  arm_sw_prev = 2;

static float baro_alt_m;      // độ cao baro mới nhất, đơn vị mét

static bool baro_healthy, baro_cached_valid;
static uint32_t baro_sample_ms;
static bool landing_candidate;
static uint32_t landing_since_ms;
static uint32_t loop_period_us;


void control_task(void *parameter) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    static uint32_t t_prev_us = 0;
    uint32_t t_now_us = fc_time_us();
    loop_period_us = t_now_us - t_prev_us;
    t_prev_us = t_now_us;

    const bool was_armed = armed;
    const int previous_mode = flight_mode;
    const bool imu_ok = imu_update();
    acc_x_g        = imu_get_acc_x_g();
    acc_y_g        = imu_get_acc_y_g();
    acc_z_g        = imu_get_acc_z_g();
    rate_roll_dps  = imu_get_rate_roll_dps();
    rate_pitch_dps = imu_get_rate_pitch_dps();
    rate_yaw_dps   = imu_get_rate_yaw_dps();
    roll_deg       = imu_get_roll_deg();
    pitch_deg      = imu_get_pitch_deg();
    rc_fetch_and_map();
    baro_fetch_shared();

    const bool finite_imu = std::isfinite(acc_x_g) && std::isfinite(acc_y_g) &&
        std::isfinite(acc_z_g) && std::isfinite(rate_roll_dps) &&
        std::isfinite(rate_pitch_dps) && std::isfinite(rate_yaw_dps) &&
        std::isfinite(roll_deg) && std::isfinite(pitch_deg);
    if (!imu_ok || !finite_imu) {
      fault_disarm();
      esc_1 = esc_2 = esc_3 = esc_4 = ESC_IDLE;
      esc_write(esc_1, esc_2, esc_3, esc_4);
      telemetry_snapshot();
      vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_CTRL_MS));
      continue;
    }

    const float sr = sinf(roll_deg * FC_DEG_TO_RAD), cr = cosf(roll_deg * FC_DEG_TO_RAD);
    const float sp = sinf(pitch_deg * FC_DEG_TO_RAD), cp = cosf(pitch_deg * FC_DEG_TO_RAD);
    acc_earth_z_g = -sp * acc_x_g + cp * sr * acc_y_g + cp * cr * acc_z_g - 1.0f;
    // Giữ nhịp correct 500 Hz của bộ tham số gốc, nhưng chỉ dùng mẫu còn hạn.
    if (baro_healthy) alt_kf_update(baro_alt_m, acc_earth_z_g);
    else alt_kf_init();
    if (!std::isfinite(alt_kf_get_altitude_cm()) || !std::isfinite(alt_kf_get_climb_cms())) {
      alt_kf_init();
      baro_healthy = false;
    }

    if (rc_ok) {
      if (rc_ch[CH_ARM] != 2) armed = false;
      else if (arm_sw_prev != 2 && rc_ch[CH_THROTTLE] < 1050) armed = true;
      arm_sw_prev = rc_ch[CH_ARM];
      if      (rc_ch[CH_MODE] == 0) flight_mode = MODE_FOR_SW_0;
      else if (rc_ch[CH_MODE] == 1) flight_mode = MODE_FOR_SW_1;
      else                         flight_mode = MODE_FOR_SW_2;
      // Còn RC nhưng mất độ cao: trả quyền ga trực tiếp cho người lái.
      if (flight_mode == MODE_ALT_HOLD && !baro_healthy) flight_mode = MODE_ANGLE;
    } else if (armed && baro_healthy) {
      flight_mode = MODE_FAILSAFE;
    } else {
      fault_disarm();
    }

    if (!armed) {
      pid_reset_all();
      u_roll = u_pitch = u_yaw = 0;
      u_throttle = altitude_throttle = ESC_IDLE;
    } else {
      if (flight_mode != MODE_ANGLE && (flight_mode != previous_mode || !was_armed)) {
        altitude_throttle = fmaxf(ESC_MIN_ARMED, fminf(ESC_MAX, u_throttle));
        const float setpoint = flight_mode == MODE_FAILSAFE ? FAILSAFE_DESCENT_CMS : climb_sp_cms;
        pid_climb_seed(altitude_throttle - ESC_IDLE, alt_kf_get_climb_cms(), setpoint);
      }
      switch (flight_mode) {
        case MODE_ALT_HOLD: mode_alt_hold(); break;
        case MODE_FAILSAFE: mode_failsafe(); break;
        default:
          if (rc_ch[CH_THROTTLE] * 0.8f < 830) pid_bleed_integrators();
          mode_angle();
          break;
      }
    }

    // Không chuyển NaN/Inf sang int trong mixer hoặc chuyển xuống driver PWM.
    if (!std::isfinite(u_throttle) || !std::isfinite(u_roll) ||
        !std::isfinite(u_pitch) || !std::isfinite(u_yaw)) fault_disarm();

    const bool landing_now = armed && !rc_ok && baro_healthy &&
        flight_mode == MODE_FAILSAFE && u_throttle <= ESC_MIN_ARMED + 1 &&
        fabsf(alt_kf_get_climb_cms()) < 5.0f;
    if (landing_now) {
      if (!landing_candidate) { landing_candidate = true; landing_since_ms = fc_time_ms(); }
      if (fc_time_ms() - landing_since_ms >= LANDING_CONFIRM_MS) fault_disarm();
    } else landing_candidate = false;

    esc_lim_lo = armed ? ESC_MIN_ARMED : ESC_IDLE;
    esc_lim_hi = armed ? ESC_MAX : ESC_IDLE;
    esc_1 = mix_to_esc(u_throttle - u_roll - u_pitch - u_yaw);
    esc_2 = mix_to_esc(u_throttle + u_roll + u_pitch - u_yaw);
    esc_3 = mix_to_esc(u_throttle + u_roll - u_pitch + u_yaw);
    esc_4 = mix_to_esc(u_throttle - u_roll + u_pitch + u_yaw);
    esc_write(esc_1, esc_2, esc_3, esc_4);

    telemetry_snapshot();
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_CTRL_MS));
  }
}


// ============================================================================
//  Các chế độ bay
// ============================================================================

// ANGLE: cần gạt quy định góc nghiêng, thả cần thì drone tự về bằng.
static void mode_angle() {
  u_throttle = rc_ch[CH_THROTTLE] * 0.8f;
  u_roll  = pid_rate_roll (rate_roll_dps,  pid_tilt_roll (roll_deg,  roll_sp_deg));
  u_pitch = pid_rate_pitch(rate_pitch_dps, pid_tilt_pitch(pitch_deg, pitch_sp_deg));
  u_yaw   = pid_rate_yaw  (rate_yaw_dps,   yaw_rate_sp_dps);
}

// ALT HOLD: cần ga quy định TỐC ĐỘ lên xuống. Cần ga ở giữa = giữ độ cao.
static void mode_alt_hold() {
  u_throttle = climb_throttle(climb_sp_cms);
  u_roll  = pid_rate_roll (rate_roll_dps,  pid_tilt_roll (roll_deg,  roll_sp_deg));
  u_pitch = pid_rate_pitch(rate_pitch_dps, pid_tilt_pitch(pitch_deg, pitch_sp_deg));
  u_yaw   = pid_rate_yaw  (rate_yaw_dps,   yaw_rate_sp_dps);
}

// FAILSAFE: giữ thăng bằng và hạ đều cho tới khi chạm đất rồi tự tắt máy.
static void mode_failsafe() {
  u_throttle = climb_throttle(FAILSAFE_DESCENT_CMS);
  u_roll  = pid_rate_roll (rate_roll_dps,  pid_tilt_roll (roll_deg,  0));
  u_pitch = pid_rate_pitch(rate_pitch_dps, pid_tilt_pitch(pitch_deg, 0));
  u_yaw   = pid_rate_yaw  (rate_yaw_dps,   0);
}


// ============================================================================
//  Lấy dữ liệu dùng chung từ các task khác
// ============================================================================

// Luôn lấy giá trị baro mới nhất. Lấy mutex không được thì giữ giá trị vòng
// trước, không sao - bộ lọc vẫn chạy đúng nhịp.
static void baro_fetch_shared() {
  if (xSemaphoreTake(mtx_baro, 0) == pdTRUE) {
    baro_alt_m = shared_baro_alt_m;
    baro_cached_valid = shared_baro_valid;
    baro_sample_ms = shared_baro_sample_ms;
    xSemaphoreGive(mtx_baro);
  }
  baro_healthy = sensor_present.baro && baro_cached_valid && std::isfinite(baro_alt_m) &&
                 fc_time_ms() - baro_sample_ms <= BARO_TIMEOUT_MS;
}

// Đổi giá trị 1000-2000 của công tắc thành nấc 0 / 1 / 2
static int rc_switch_position(int ch_value) {
  if      (ch_value > 900  && ch_value < 1100) return 0;
  else if (ch_value > 1400 && ch_value < 1600) return 1;
  else if (ch_value > 1900 && ch_value < 2100) return 2;
  else                                          return 0;
}

static void rc_fetch_and_map() {
  if (xSemaphoreTake(mtx_rc, 0) == pdTRUE) {
    rc_cached_ok = shared_rc_ok;
    rc_frame_ms = shared_rc_frame_ms;
    for (int i = 1; i <= 16; i++) rc_ch[i] = shared_rc_ch[i];
    xSemaphoreGive(mtx_rc);
    // Chỉ map snapshot mới, không map lần hai các nấc 0/1/2 khi mutex bận.
    rc_ch[CH_ARM] = rc_switch_position(rc_ch[CH_ARM]);
    rc_ch[CH_MODE] = rc_switch_position(rc_ch[CH_MODE]);
  }
  rc_ok = rc_cached_ok && fc_time_ms() - rc_frame_ms <= RC_TIMEOUT_MS;
  if (!rc_ok) return;

  // Lọc mượt lệnh cần gạt. Hệ số 0.05 ở 500Hz tương đương hằng số thời gian ~40ms
  const float a = 0.05f;
  roll_sp_deg     = roll_sp_deg     * (1 - a) + ( float(rc_ch[CH_ROLL]     - 1500) / (500.0f / LIM_TILT_DEG))        * a;
  pitch_sp_deg    = pitch_sp_deg    * (1 - a) + ( float(rc_ch[CH_PITCH]    - 1500) / (500.0f / LIM_TILT_DEG))        * a;
  yaw_rate_sp_dps = yaw_rate_sp_dps * (1 - a) + (-float(rc_ch[CH_YAW]      - 1500) / (500.0f / LIM_YAW_RATE_DPS))    * a;
  climb_sp_cms    = climb_sp_cms    * (1 - a) + ( float(rc_ch[CH_THROTTLE] - 1500) / (500.0f / LIM_CLIMB_RATE_CMS))  * a;
}

static int mix_to_esc(float duty) {
  if (!std::isfinite(duty)) return ESC_IDLE;
  if (duty < esc_lim_lo) return esc_lim_lo;
  if (duty > esc_lim_hi) return esc_lim_hi;
  return static_cast<int>(duty);
}

static void fault_disarm() {
  armed = false;
  arm_sw_prev = 2; // cần một lần ARM xuống rồi lên sau lỗi, không tự khởi động lại
  landing_candidate = false;
  pid_reset_all();
  u_roll = u_pitch = u_yaw = 0;
  u_throttle = altitude_throttle = ESC_IDLE;
}

static float climb_throttle(float setpoint) {
  const float requested = ESC_IDLE + pid_climb(alt_kf_get_climb_cms(), setpoint);
  if (!std::isfinite(requested)) return requested; // nhánh bảo vệ CTRL sẽ disarm
  const float target = fmaxf(ESC_MIN_ARMED, fminf(ESC_MAX, requested));
  const float step = ALT_THROTTLE_SLEW_PER_S * DT_CTRL;
  altitude_throttle += fmaxf(-step, fminf(step, target - altitude_throttle));
  return altitude_throttle;
}


// Chụp lại trạng thái cho baro_task in debug. Lấy mutex không được thì bỏ qua,
// vì đây chỉ là dữ liệu hiển thị, không được phép làm chậm vòng điều khiển.
static void telemetry_snapshot() {
  if (xSemaphoreTake(mtx_tlm, 0) != pdTRUE) return;

  tlm_roll_deg     = roll_deg;      tlm_pitch_deg    = pitch_deg;
  tlm_roll_sp_deg  = roll_sp_deg;   tlm_pitch_sp_deg = pitch_sp_deg;

  tlm_rate_roll_dps  = rate_roll_dps;
  tlm_rate_pitch_dps = rate_pitch_dps;
  tlm_rate_yaw_dps   = rate_yaw_dps;
  tlm_rate_roll_sp_dps  = pid_tilt_roll_output();
  tlm_rate_pitch_sp_dps = pid_tilt_pitch_output();

  tlm_alt_cm       = alt_kf_get_altitude_cm();
  tlm_climb_cms    = alt_kf_get_climb_cms();
  tlm_climb_sp_cms = climb_sp_cms;

  tlm_acc_x_g = acc_x_g;  tlm_acc_y_g = acc_y_g;  tlm_acc_z_g = acc_z_g;

  for (int i = 1; i <= 16; i++) tlm_rc_ch[i] = rc_ch[i];
  tlm_esc[1] = esc_1;  tlm_esc[2] = esc_2;  tlm_esc[3] = esc_3;  tlm_esc[4] = esc_4;

  tlm_flight_mode = flight_mode;
  tlm_armed       = armed;
  tlm_rc_ok       = rc_ok;
  tlm_loop_us     = loop_period_us;

  xSemaphoreGive(mtx_tlm);
}
