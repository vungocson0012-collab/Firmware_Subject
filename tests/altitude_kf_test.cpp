// Standalone host regression test. No ESP-IDF or third-party libraries needed.
// c++ -std=c++17 -Iinclude src/altitude_kf.cpp tests/altitude_kf_test.cpp -o /tmp/fc_kf_test
#include "altitude_kf.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>

using Matrix = std::array<std::array<double, 2>, 2>;
static Matrix multiply(const Matrix &a, const Matrix &b) {
  Matrix result{};
  for (int i = 0; i < 2; ++i)
    for (int j = 0; j < 2; ++j)
      for (int k = 0; k < 2; ++k) result[i][j] += a[i][k] * b[k][j];
  return result;
}

int main() {
  constexpr double dt = 0.002;
  const Matrix f = {{{1, dt}, {0, 1}}};
  const Matrix ft = {{{1, 0}, {dt, 1}}};
  const std::array<double, 2> g = {0.5 * dt * dt, dt};
  double max_z_error = 0, max_v_error = 0;
  for (int scenario = 0; scenario < 4; ++scenario) {
    alt_kf_init();
    assert(alt_kf_get_altitude_cm() == 0);
    assert(alt_kf_get_climb_cms() == 0);
    Matrix p{};
    std::array<double, 2> state{};
    for (int sample = 0; sample < 120000; ++sample) {
      const double t = sample * dt;
      const float altitude = scenario == 0 ? 0.0f : scenario == 1 ? 2.0f
        : static_cast<float>(2.0 * std::sin(t * 0.4) + 0.05 * std::sin(t * 7));
      const float acceleration = scenario < 2 ? 0.0f : scenario == 2
        ? static_cast<float>(-0.32 / 9.81 * std::sin(t * 0.4))
        : static_cast<float>((sample % 10000 < 500 ? 0.1 : -0.005) + 0.002 * std::sin(t));

      // Original matrix model: S=F*S+G*u, P=F*P*F'+Q, H=[1,0].
      std::array<double, 2> predicted{};
      for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) predicted[i] += f[i][j] * state[j];
        predicted[i] += g[i] * acceleration * 981.0;
      }
      p = multiply(multiply(f, p), ft);
      for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) p[i][j] += g[i] * g[j] * 16.0;
      const std::array<double, 2> gain = {p[0][0] / (p[0][0] + 900),
                                         p[1][0] / (p[0][0] + 900)};
      for (int i = 0; i < 2; ++i)
        state[i] = predicted[i] + gain[i] * (altitude * 100.0 - predicted[0]);
      const Matrix correction = {{{1 - gain[0], 0}, {-gain[1], 1}}};
      p = multiply(correction, p);

      alt_kf_update(altitude, acceleration);
      const double z = alt_kf_get_altitude_cm();
      const double v = alt_kf_get_climb_cms();
      assert(std::isfinite(z) && std::isfinite(v));
      max_z_error = std::max(max_z_error, std::abs(z - state[0]));
      max_v_error = std::max(max_v_error, std::abs(v - state[1]));
    }
  }
  assert(max_z_error < 0.05); // cm, tolerates single-precision rounding.
  assert(max_v_error < 0.05); // cm/s.
  std::printf("PASS: 480000 updates, max error %.6f cm / %.6f cm/s\n",
              max_z_error, max_v_error);
}
