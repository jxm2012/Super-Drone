#include "mpu6050.h"

#include <math.h>

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "mpu6050"

#define I2C_PORT I2C_NUM_0

#define REG_PWR_MGMT_1 0x6B
#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_WHO_AM_I 0x75
#define REG_ACCEL_XOUT_H 0x3B

/* accel +/-8 g -> 4096 LSB/g, then convert g -> m/s^2 */
#define ACCEL_SCALE (8.0f / 32768.0f * 9.80665f)
/* gyro +/-500 dps -> 65.5 LSB/(deg/s), then deg/s -> rad/s */
#define GYRO_SCALE (500.0f / 32768.0f * 0.017453292519943295f)

static esp_err_t mpu_write_byte(uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return err;
}

static esp_err_t mpu_read_bytes(uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, &buf[len - 1], I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return err;
}

esp_err_t mpu6050_init(int sda_pin, int scl_pin, uint32_t freq_hz)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = freq_hz,
    };
    esp_err_t err = i2c_param_config(I2C_PORT, &conf);
    if (err != ESP_OK) {
        return err;
    }
    err = i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t who = 0;
    err = mpu_read_bytes(REG_WHO_AM_I, &who, 1);
    if (err != ESP_OK || (who & 0x7E) != 0x68) {
        ESP_LOGE(TAG, "MPU6050 not found (WHO_AM_I=0x%02X)", who);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "MPU6050 found (WHO_AM_I=0x%02X)", who);

    /* wake from sleep (clear SLEEP bit) */
    mpu_write_byte(REG_PWR_MGMT_1, 0x00);
    /* sample rate = 1 kHz (internal gyro rate / (1 + 0)) */
    mpu_write_byte(REG_SMPLRT_DIV, 0x00);
    /* DLPF_CFG=2 -> gyro 94 Hz / accel 98 Hz bandwidth, 1 kHz sample rate */
    mpu_write_byte(REG_CONFIG, 0x02);
    /* gyro +/-500 dps */
    mpu_write_byte(REG_GYRO_CONFIG, 0x08);
    /* accel +/-8 g */
    mpu_write_byte(REG_ACCEL_CONFIG, 0x10);

    return ESP_OK;
}

esp_err_t mpu6050_read(mpu6050_data_t *out)
{
    uint8_t buf[14];
    esp_err_t err = mpu_read_bytes(REG_ACCEL_XOUT_H, buf, sizeof(buf));
    if (err != ESP_OK) {
        return err;
    }

    int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
    int16_t t = (int16_t)((buf[6] << 8) | buf[7]);
    int16_t gx = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);

    out->accel_x = ax * ACCEL_SCALE;
    out->accel_y = ay * ACCEL_SCALE;
    out->accel_z = az * ACCEL_SCALE;
    out->gyro_x = gx * GYRO_SCALE;
    out->gyro_y = gy * GYRO_SCALE;
    out->gyro_z = gz * GYRO_SCALE;
    out->temp_c = t / 340.0f + 36.53f;

    return ESP_OK;
}
