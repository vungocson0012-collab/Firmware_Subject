idf_component_register(
    SRCS "main.cpp" "shared_state.cpp" "control_task.cpp"
         "imu_icm20602.cpp" "baro_bmp388.cpp" "altitude_kf.cpp"
         "pid.cpp" "rc_sbus.cpp" "esc_output.cpp"
    INCLUDE_DIRS "../include"
    REQUIRES driver esp_timer esp_rom
)
