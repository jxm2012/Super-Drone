#include "flight_controller.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"

#define TAG "flight_controller"

#define MOTOR_COUNT 4

/* ESC PWM: 50 Hz, 1-2 ms pulse, 16-bit duty */
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_TIMER_SEL LEDC_TIMER_0
#define LEDC_RESOLUTION LEDC_TIMER_16_BIT
#define LEDC_FREQ_HZ 50
#define PWM_DUTY_MAX 65535.0f
#define PWM_PERIOD_US 20000.0f

/* Complementary filter: weight given to gyro integration */
#define CF_ALPHA 0.98f
#define DEG_TO_RAD 0.017453292519943295f

#define ANGLE_RATE_MAX (300.0f * DEG_TO_RAD) /* max rate setpoint from angle loop */
#define AXIS_OUTPUT_MAX 250.0f               /* max per-axis mixer contribution, us */
#define YAW_OUTPUT_MAX 200.0f
#define INTEGRAL_LIMIT 5.0f /* integrated rate error, rad */

#define THROTTLE_RAMP_US_S 500.0f
#define SENSOR_FAIL_TIMEOUT_S 0.2f

/*
 * X-quad layout. Pins: FR=GPIO27, FL=GPIO14, BR=GPIO12, BL=GPIO13.
 * CW props on FR, BL; CCW props on FL, BR.
 * Mixer signs assume the MPU6050 is mounted X-forward (chip arrow forward).
 * Verify spin directions on the bench and flip signs if needed.
 */
enum { M_FR = 0, M_FL, M_BR, M_BL };

static const gpio_num_t motor_pins[MOTOR_COUNT] = {
    GPIO_NUM_27, GPIO_NUM_14, GPIO_NUM_12, GPIO_NUM_13,
};
static const ledc_channel_t motor_channels[MOTOR_COUNT] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3,
};

static struct {
    float kp_angle;
    float kp_rate;
    float ki_rate;
    float kd_rate;
    float kd_yaw;
    float dt;
} cfg;

static struct {
    bool armed;
    float throttle_target_us;
    float throttle_cur_us;
    float roll_sp;  /* rad, relative to level reference */
    float pitch_sp; /* rad, relative to level reference */
    float roll_ref; /* rad, level reference (captured on arm) */
    float pitch_ref;
    float gyro_bias[3];
    float roll;   /* rad, filtered attitude */
    float pitch;  /* rad, filtered attitude */
    float rate_err_prev[2];
    float integral[2];
    uint32_t out_us[4];
    uint32_t fail_count;
    int64_t last_update_us;
} state;

static void motor_set_us(int i, float us)
{
    if (us < FC_ESC_PULSE_MIN_US) {
        us = FC_ESC_PULSE_MIN_US;
    }
    if (us > FC_ESC_PULSE_MAX_US) {
        us = FC_ESC_PULSE_MAX_US;
    }
    state.out_us[i] = (uint32_t)us;
    uint32_t duty = (uint32_t)(us / PWM_PERIOD_US * PWM_DUTY_MAX);
    ledc_set_duty(LEDC_MODE, motor_channels[i], duty);
    ledc_update_duty(LEDC_MODE, motor_channels[i]);
}

static void motors_output_min(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_set_us(i, FC_ESC_PULSE_MIN_US);
    }
}

esp_err_t fc_init(const fc_config_t *cfg_in)
{
    cfg.kp_angle = cfg_in->kp_angle;
    cfg.kp_rate = cfg_in->kp_rate;
    cfg.ki_rate = cfg_in->ki_rate;
    cfg.kd_rate = cfg_in->kd_rate;
    cfg.kd_yaw = cfg_in->kd_yaw;
    cfg.dt = 1.0f / (float)cfg_in->rate_hz;

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER_SEL,
        .duty_resolution = LEDC_RESOLUTION,
        .freq_hz = LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(err));
        return err;
    }

    for (int i = 0; i < MOTOR_COUNT; i++) {
        ledc_channel_config_t ch = {
            .gpio_num = motor_pins[i],
            .speed_mode = LEDC_MODE,
            .channel = motor_channels[i],
            .timer_sel = LEDC_TIMER_SEL,
            .duty = 0,
            .hpoint = 0,
            .intr_type = LEDC_INTR_DISABLE,
            .flags.output_invert = 0,
        };
        err = ledc_channel_config(&ch);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ledc_channel_config %d failed: %s", i, esp_err_to_name(err));
            return err;
        }
    }

    memset(&state, 0, sizeof(state));
    state.throttle_cur_us = FC_ESC_PULSE_MIN_US;
    state.last_update_us = esp_timer_get_time();
    motors_output_min();
    return ESP_OK;
}

void fc_set_armed(bool armed)
{
    if (armed && !state.armed) {
        /* capture current orientation as level reference and reset loops */
        state.roll_ref = state.roll;
        state.pitch_ref = state.pitch;
        state.roll_sp = 0;
        state.pitch_sp = 0;
        state.throttle_cur_us = FC_ESC_PULSE_MIN_US;
        memset(state.integral, 0, sizeof(state.integral));
        state.rate_err_prev[0] = state.rate_err_prev[1] = 0;
    }
    state.armed = armed;
    if (!armed) {
        state.throttle_cur_us = FC_ESC_PULSE_MIN_US;
        motors_output_min();
    }
    ESP_LOGI(TAG, "armed=%d", armed);
}

bool fc_get_armed(void)
{
    return state.armed;
}

void fc_set_throttle_us(float pulse_us)
{
    if (pulse_us < FC_ESC_PULSE_MIN_US) {
        pulse_us = FC_ESC_PULSE_MIN_US;
    }
    if (pulse_us > FC_ESC_PULSE_MAX_US) {
        pulse_us = FC_ESC_PULSE_MAX_US;
    }
    state.throttle_target_us = pulse_us;
}

float fc_get_throttle_us(void)
{
    return state.throttle_target_us;
}

void fc_set_level_reference(void)
{
    state.roll_ref = state.roll;
    state.pitch_ref = state.pitch;
    state.roll_sp = 0;
    state.pitch_sp = 0;
}

void fc_set_attitude_setpoint(float roll_deg, float pitch_deg)
{
    state.roll_sp = roll_deg * DEG_TO_RAD;
    state.pitch_sp = pitch_deg * DEG_TO_RAD;
}

void fc_set_gyro_bias(float gx, float gy, float gz)
{
    state.gyro_bias[0] = gx;
    state.gyro_bias[1] = gy;
    state.gyro_bias[2] = gz;
}

void fc_get_attitude(float *roll_deg, float *pitch_deg)
{
    *roll_deg = (state.roll - state.roll_ref) / DEG_TO_RAD;
    *pitch_deg = (state.pitch - state.pitch_ref) / DEG_TO_RAD;
}

void fc_get_motors(float *out_us)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        out_us[i] = (float)state.out_us[i];
    }
}

void fc_update(const mpu6050_data_t *imu)
{
    int64_t now = esp_timer_get_time();
    float dt = (float)(now - state.last_update_us) / 1e6f;
    state.last_update_us = now;

    if (dt < 0) {
        dt = cfg.dt;
    }

    /* Sensor watchdog: persistent read failures or a stalled loop disarms. */
    bool sensor_ok = imu != NULL;
    if (sensor_ok && dt > 5.0f * cfg.dt) {
        sensor_ok = false;
        dt = cfg.dt;
    }
    if (!sensor_ok) {
        state.fail_count++;
        if (state.fail_count * cfg.dt > SENSOR_FAIL_TIMEOUT_S) {
            if (state.armed) {
                ESP_LOGW(TAG, "sensor stalled, disarming!");
            }
            state.armed = false;
            motors_output_min();
        }
        return;
    }
    state.fail_count = 0;

    float gx = imu->gyro_x - state.gyro_bias[0];
    float gy = imu->gyro_y - state.gyro_bias[1];
    float gz = imu->gyro_z - state.gyro_bias[2];

    /* Complementary filter: gyro integration fused with accelerometer tilt. */
    float pitch_accel = atan2f(-imu->accel_x,
                               sqrtf(imu->accel_y * imu->accel_y + imu->accel_z * imu->accel_z));
    float roll_accel = atan2f(imu->accel_y, imu->accel_z);
    state.pitch = CF_ALPHA * (state.pitch + gy * dt) + (1.0f - CF_ALPHA) * pitch_accel;
    state.roll = CF_ALPHA * (state.roll + gx * dt) + (1.0f - CF_ALPHA) * roll_accel;

    /* Ramp throttle toward its target. */
    float max_delta = THROTTLE_RAMP_US_S * dt;
    float d = state.throttle_target_us - state.throttle_cur_us;
    if (d > max_delta) {
        state.throttle_cur_us += max_delta;
    } else if (d < -max_delta) {
        state.throttle_cur_us -= max_delta;
    } else {
        state.throttle_cur_us += d;
    }

    if (!state.armed) {
        motors_output_min();
        return;
    }

    float rel_roll = state.roll - state.roll_ref;
    float rel_pitch = state.pitch - state.pitch_ref;

    /* Outer angle loop -> rate setpoint (clamped). */
    float rate_sp_roll = cfg.kp_angle * (state.roll_sp - rel_roll);
    float rate_sp_pitch = cfg.kp_angle * (state.pitch_sp - rel_pitch);
    if (rate_sp_roll > ANGLE_RATE_MAX) {
        rate_sp_roll = ANGLE_RATE_MAX;
    }
    if (rate_sp_roll < -ANGLE_RATE_MAX) {
        rate_sp_roll = -ANGLE_RATE_MAX;
    }
    if (rate_sp_pitch > ANGLE_RATE_MAX) {
        rate_sp_pitch = ANGLE_RATE_MAX;
    }
    if (rate_sp_pitch < -ANGLE_RATE_MAX) {
        rate_sp_pitch = -ANGLE_RATE_MAX;
    }

    /* Inner rate loop (PID, D on rate error). */
    float err[2] = { rate_sp_roll - gx, rate_sp_pitch - gy };
    state.integral[0] += err[0] * dt;
    state.integral[1] += err[1] * dt;
    for (int i = 0; i < 2; i++) {
        if (state.integral[i] > INTEGRAL_LIMIT) {
            state.integral[i] = INTEGRAL_LIMIT;
        }
        if (state.integral[i] < -INTEGRAL_LIMIT) {
            state.integral[i] = -INTEGRAL_LIMIT;
        }
    }
    float deriv[2] = {
        (err[0] - state.rate_err_prev[0]) / dt,
        (err[1] - state.rate_err_prev[1]) / dt,
    };
    state.rate_err_prev[0] = err[0];
    state.rate_err_prev[1] = err[1];

    float out_roll = cfg.kp_rate * err[0] + cfg.ki_rate * state.integral[0] + cfg.kd_rate * deriv[0];
    float out_pitch = cfg.kp_rate * err[1] + cfg.ki_rate * state.integral[1] + cfg.kd_rate * deriv[1];
    if (out_roll > AXIS_OUTPUT_MAX) {
        out_roll = AXIS_OUTPUT_MAX;
    }
    if (out_roll < -AXIS_OUTPUT_MAX) {
        out_roll = -AXIS_OUTPUT_MAX;
    }
    if (out_pitch > AXIS_OUTPUT_MAX) {
        out_pitch = AXIS_OUTPUT_MAX;
    }
    if (out_pitch < -AXIS_OUTPUT_MAX) {
        out_pitch = -AXIS_OUTPUT_MAX;
    }

    float out_yaw = -cfg.kd_yaw * gz;
    if (out_yaw > YAW_OUTPUT_MAX) {
        out_yaw = YAW_OUTPUT_MAX;
    }
    if (out_yaw < -YAW_OUTPUT_MAX) {
        out_yaw = -YAW_OUTPUT_MAX;
    }

    /* X-quad mixer (outputs in pulse-us). */
    float base = state.throttle_cur_us;
    float m[MOTOR_COUNT];
    m[M_FR] = base + out_roll - out_pitch - out_yaw;
    m[M_FL] = base - out_roll - out_pitch + out_yaw;
    m[M_BR] = base - out_roll + out_pitch + out_yaw;
    m[M_BL] = base + out_roll + out_pitch - out_yaw;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_set_us(i, m[i]);
    }
}

void fc_telemetry(void)
{
    printf("FC armed=%d roll=%+.1f pitch=%+.1f thr=%" PRIu32 " motors=%" PRIu32 "/%" PRIu32
           "/%" PRIu32 "/%" PRIu32 "\r\n",
           state.armed ? 1 : 0,
           (state.roll - state.roll_ref) / DEG_TO_RAD,
           (state.pitch - state.pitch_ref) / DEG_TO_RAD,
           (uint32_t)state.throttle_cur_us,
           state.out_us[M_FR], state.out_us[M_FL], state.out_us[M_BR], state.out_us[M_BL]);
}
