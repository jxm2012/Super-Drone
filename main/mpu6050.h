#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define MPU6050_I2C_ADDR 0x68

typedef struct {
    float accel_x; /* m/s^2 */
    float accel_y;
    float accel_z;
    float gyro_x; /* rad/s */
    float gyro_y;
    float gyro_z;
    float temp_c;
} mpu6050_data_t;

/**
 * Initialize I2C (legacy driver) and the MPU6050.
 * Configures: wake from sleep, 1 kHz internal sampling, DLPF bandwidth ~94 Hz
 * (gyro) / ~98 Hz (accel), gyro range +/-500 dps, accel range +/-8 g.
 */
esp_err_t mpu6050_init(int sda_pin, int scl_pin, uint32_t freq_hz);

/** Read one full sample (accel + temp + gyro) in SI units. */
esp_err_t mpu6050_read(mpu6050_data_t *out);
