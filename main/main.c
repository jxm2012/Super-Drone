#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_vfs_dev.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "flight_controller.h"
#include "mpu6050.h"

#define TAG "main"

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22
#define I2C_FREQ_HZ 400000

#define CONTROL_RATE_HZ 250

static void calibrate_gyro(float *bx, float *by, float *bz)
{
    const int N = 200;
    double sx = 0, sy = 0, sz = 0;
    mpu6050_data_t d;
    for (int i = 0; i < N; i++) {
        if (mpu6050_read(&d) == ESP_OK) {
            sx += d.gyro_x;
            sy += d.gyro_y;
            sz += d.gyro_z;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    *bx = (float)(sx / N);
    *by = (float)(sy / N);
    *bz = (float)(sz / N);
    ESP_LOGI(TAG, "gyro bias: x=%+.4f y=%+.4f z=%+.4f rad/s", *bx, *by, *bz);
}

static void handle_command(char *line, float *roll_sp_deg, float *pitch_sp_deg)
{
    float v;

    if (strcmp(line, "help") == 0) {
        printf("Commands:\r\n"
               "  arm / disarm         - engage/disengage motor control\r\n"
               "  level                - set current orientation as level reference\r\n"
               "  status               - print one status line\r\n"
               "  t <us>               - set throttle target (1000..2000 us)\r\n"
               "  r <deg> / p <deg>    - roll / pitch angle setpoint\r\n");
    } else if (strcmp(line, "arm") == 0) {
        fc_set_armed(true);
        printf("ARMED\r\n");
    } else if (strcmp(line, "disarm") == 0) {
        fc_set_armed(false);
        printf("DISARMED\r\n");
    } else if (strcmp(line, "level") == 0) {
        fc_set_level_reference();
        printf("LEVEL REFERENCE SET\r\n");
    } else if (strcmp(line, "status") == 0) {
        fc_telemetry();
    } else if (sscanf(line, "t %f", &v) == 1) {
        fc_set_throttle_us(v);
        printf("throttle target %u us\r\n", (unsigned)fc_get_throttle_us());
    } else if (sscanf(line, "r %f", &v) == 1) {
        *roll_sp_deg = v;
        fc_set_attitude_setpoint(*roll_sp_deg, *pitch_sp_deg);
        printf("roll sp %.1f deg\r\n", v);
    } else if (sscanf(line, "p %f", &v) == 1) {
        *pitch_sp_deg = v;
        fc_set_attitude_setpoint(*roll_sp_deg, *pitch_sp_deg);
        printf("pitch sp %.1f deg\r\n", v);
    } else {
        printf("unknown: %s (try 'help')\r\n", line);
    }
}

static void control_task(void *arg)
{
     ESP_LOGI(TAG, "Entered control task");
    TickType_t last = xTaskGetTickCount();
    TickType_t period = pdMS_TO_TICKS(1000 / CONTROL_RATE_HZ);
    if (period == 0) {
        period = 1; /* guard against 100 Hz tick config -> vTaskDelayUntil assert */
    }
    mpu6050_data_t imu;

    while (1) {
        if (mpu6050_read(&imu) == ESP_OK) {
            fc_update(&imu);
        } else {
            fc_update(NULL);
        }
        vTaskDelayUntil(&last, period);
    }
}

// Function to trim newline and trailing spaces
void trim_newline(char *str) {
    size_t len = strlen(str);
    while (len > 0 && (str[len - 1] == '\n' || isspace((unsigned char)str[len - 1]))) {
        str[--len] = '\0';
    }
}

static void command_task(void *arg)
{
    char line[64];
    float roll_sp_deg = 0;
    float pitch_sp_deg = 0;
    printf("Command task ready. Type 'help' for commands.\r\n");
    printf("> ");

    while (1) {
        if (fgets(line, sizeof(line), stdin) == NULL) {
            continue; /* non-blocking stdin returns NULL; avoid spinning */
        }
        trim_newline(line);
        if (line[0] != '\0') {
            handle_command(line, &roll_sp_deg, &pitch_sp_deg);
        }
    }
}

static void telemetry_task(void *arg)
{
    ESP_LOGI(TAG, "Entered telemetry task");
    while (1) {
        fc_telemetry();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Pre-Hal flight controller starting...");

    /* The boot console VFS reads UART0 via a non-blocking FIFO poll, so a bare
     * getchar() would return instantly and spin the command task. Install the
     * UART driver and route stdio through it so getchar() blocks properly. */
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
    esp_vfs_dev_uart_use_driver(UART_NUM_0);

    esp_err_t err = mpu6050_init(SDA_PIN, SCL_PIN, I2C_FREQ_HZ);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 init failed. Check wiring (SDA=21, SCL=22). Halting.");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    float bx, by, bz;
    calibrate_gyro(&bx, &by, &bz);
    fc_set_gyro_bias(bx, by, bz);

    fc_config_t cfg = {
        .kp_angle = 8.0f,
        .kp_rate = 320.0f,
        .ki_rate = 15.0f,
        .kd_rate = 3.0f,
        .kd_yaw = 6.0f,
        .rate_hz = CONTROL_RATE_HZ,
    };
    ESP_ERROR_CHECK(fc_init(&cfg));

    printf("Flight controller ready. Type 'help' for commands.\r\n");

    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(command_task, "commands", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(telemetry_task, "telemetry", 4096, NULL, 1, NULL, 0);
}
