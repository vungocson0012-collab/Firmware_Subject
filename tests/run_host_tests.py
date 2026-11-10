#!/usr/bin/env python3
"""Compile actual firmware modules against deterministic host driver/RTOS stubs."""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
HOST = ROOT / "tests" / "host"
SUITES = {
    "main": (["src/shared_state.cpp"], ["startup_order", "rc_publish", "baro_publish"]),
    "control": (["src/shared_state.cpp", "src/pid.cpp", "src/altitude_kf.cpp"], [
        "normal", "mutex", "alt_without_baro", "mode_transition", "failsafe_transition",
        "high_throttle_transition", "landing_timer", "landing_wrap", "stale_baro",
        "baro_mutex_timeout", "rc_stall", "rc_mutex_timeout", "rc_wrap",
        "no_rc_no_baro", "imu_invalid", "imu_nan", "baro_nan"]),
    "sbus": ([], ["normal", "startup", "flags", "lost_frame", "payload_header", "partial",
                  "partial_timeout", "bad_footer", "noise", "wrap", "bounds"]),
    "imu": ([], ["moving_calibration", "stationary_calibration", "noisy_calibration",
                 "accel_motion", "zero_sample", "bus_error", "low_g", "recover_filter"]),
    "pid": ([], ["reset_no_kick", "climb_seed"]),
    "baro": ([], ["no_samples", "normal", "bus_error", "invalid_pressure", "nan"]),
}

def run(args):
    subprocess.run([str(arg) for arg in args], cwd=ROOT, check=True, timeout=60)

with tempfile.TemporaryDirectory(prefix="fc-host-tests-") as folder:
    output = Path(folder)
    compiler = os.environ.get("CXX", "c++")
    flags = ["-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter"]
    for name, (sources, cases) in SUITES.items():
        binary = output / name
        run([compiler, *flags, "-I" + str(HOST / "stubs"), "-I" + str(ROOT / "include"),
             HOST / (name + "_test.cpp"), *sources, "-o", binary])
        for case in cases:
            run([binary, case])
    binary = output / "altitude_kf_test"
    run([compiler, *flags, "-Iinclude", "src/altitude_kf.cpp", "tests/altitude_kf_test.cpp", "-o", binary])
    run([binary])
    print(f"PASS: {sum(len(cases) for _, cases in SUITES.values())} fault/control scenarios + Kalman regression")
