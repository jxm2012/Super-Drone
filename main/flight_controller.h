#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "mpu6050.h"

#define FC_ESC_PULSE_MIN_US 1000.0f
#define FC_ESC_PULSE_MAX_US 2000.0f

typedef struct {
    float kp_angle; /* outer angle loop: (rad/s) of rate setpoint per rad of tilt error */
    float kp_rate;  /* inner rate loop P: pulse-us per (rad/s) */
    float ki_rate;  /* inner rate loop I: pulse-us per rad of integrated rate error */
    float kd_rate;  /* inner rate loop D: pulse-us per (rad/s^2) */
    float kd_yaw;   /* yaw damping: pulse-us per (rad/s) of yaw rate */
    uint32_t rate_hz; /* control loop frequency */
} fc_config_t;

/** Initialize LEDC PWM (50 Hz) and reset controller state. */
esp_err_t fc_init(const fc_config_t *cfg);

/** Arm/disarm. Arming captures the current orientation as the level reference. */
void fc_set_armed(bool armed);
bool fc_get_armed(void);

/** Throttle target in pulse-us (1000..2000). Ramps to target on every update. */
void fc_set_throttle_us(float pulse_us);
float fc_get_throttle_us(void);

/** Level reference: treat the current orientation as zero roll/pitch. */
void fc_set_level_reference(void);

/** Roll/pitch angle setpoints in degrees (relative to level reference). */
void fc_set_attitude_setpoint(float roll_deg, float pitch_deg);

/** Gyro zero-offsets measured at boot (rad/s). */
void fc_set_gyro_bias(float gx, float gy, float gz);

/** Estimated roll/pitch in degrees relative to the level reference. */
void fc_get_attitude(float *roll_deg, float *pitch_deg);

/** Copy current motor commands (pulse-us, clamped 1000..2000). */
void fc_get_motors(float *out_us);

/**
 * Main control step. Call at cfg->rate_hz with fresh IMU data.
 * Pass NULL if the last IMU read failed; persistent failures trigger a disarm.
 */
void fc_update(const mpu6050_data_t *imu);

/** Print a one-line status snapshot over stdout. */
void fc_telemetry(void);
