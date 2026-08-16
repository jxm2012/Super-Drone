idf_component_register(
    SRCS "main.c" "mpu6050.c" "flight_controller.c"
    INCLUDE_DIRS "."
    REQUIRES driver freertos esp_timer vfs
)
