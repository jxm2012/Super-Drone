#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <esp_log.h>

//Reference: https://forum.arduino.cc/t/connecting-multi-mpu6050-using-i2c-on-esp32-wroom-32d/1175063

#define SDA_PIN 21 // Custom SDA pin
#define SCL_PIN 22 // Custom SCL pin

Adafruit_MPU6050 mpu;

// Calibration biases (computed at startup)
float accel_bias_x = 0.0f;
float accel_bias_y = 0.0f;
float accel_bias_z = 0.0f;
float gyro_bias_x = 0.0f;
float gyro_bias_y = 0.0f;
float gyro_bias_z = 0.0f;

// Global filtered orientation (radians)
float g_pitch = 0.0f;
float g_roll = 0.0f;

const float GRAVITY_MS2 = 9.80665f;

// Timing / sampling
unsigned long last_sensor_micros = 0;
// target sample delay in ms (10 ms -> ~100 Hz)
const int SAMPLE_DELAY_MS = 10;

struct PID {
  float kp, ki, kd;
  float prev_error;
  float integral;
};

void TaskReadMPU(void *pvParameters) {
  // Sensor reading task for FreeRTOS
  for (;;) {
    Mpu6050Readings();
    vTaskDelay(pdMS_TO_TICKS(SAMPLE_DELAY_MS)); // target ~100 Hz
  }
}

PID rollPID = {1.0, 0.0, 0.0, 0.0, 0.0}; // Tune these values

float computePID(PID &pid, float setpoint, float measured, float dt) {
  float error = setpoint - measured;
  pid.integral += error * dt;
  float derivative = (error - pid.prev_error) / dt;
  float output = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
  pid.prev_error = error;
  return output;
}

// Telemetry publisher prototype
void publishTelemetry(float pitch_rad, float roll_rad);
void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  // Reduce noisy core logging that interleaves with our Serial output
  // Set global log level to warnings and above to keep the console readable.
  esp_log_level_set("*", ESP_LOG_WARN);


  Wire.begin(SDA_PIN, SCL_PIN);

  // Try to initialize MPU6050 at default address 0x68 using custom I2C pins
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("Failed to find MPU6050");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  delay(100);

  // Calibrate accelerometer and gyro biases while device is stationary
  // User should keep the board still and level during this calibration
  const int CAL_SAMPLES = 200; // recommended 100-200
  
  // accumulate
  double ax_sum = 0, ay_sum = 0, az_sum = 0;
  double gx_sum = 0, gy_sum = 0, gz_sum = 0;
  for (int i = 0; i < CAL_SAMPLES; ++i) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    ax_sum += a.acceleration.x;
    ay_sum += a.acceleration.y;
    az_sum += a.acceleration.z;
    gx_sum += g.gyro.x;
    gy_sum += g.gyro.y;
    gz_sum += g.gyro.z;
    delay(5);
  }
  float ax_avg = (float)(ax_sum / CAL_SAMPLES);
  float ay_avg = (float)(ay_sum / CAL_SAMPLES);
  float az_avg = (float)(az_sum / CAL_SAMPLES);
  float gx_avg = (float)(gx_sum / CAL_SAMPLES);
  float gy_avg = (float)(gy_sum / CAL_SAMPLES);
  float gz_avg = (float)(gz_sum / CAL_SAMPLES);

  // Compute biases. We want corrected static reading to be (0,0,GRAVITY_MS2)
  accel_bias_x = ax_avg;                // remove offset on X
  accel_bias_y = ay_avg;                // remove offset on Y
  accel_bias_z = az_avg - GRAVITY_MS2;  // ensure Z reads +g when still

  gyro_bias_x = gx_avg; // subtract so stationary angular rate is zero
  gyro_bias_y = gy_avg;
  gyro_bias_z = gz_avg;

  

  // Create the FreeRTOS task for sensor reading, priority 1, pin to core 1
  xTaskCreatePinnedToCore(
    TaskReadMPU,      // Task function
    "ReadMPU",        // Name
    4096,             // Stack size
    NULL,             // Parameters
    1,                // Priority
    NULL,             // Task handle
    1                 // Core (0 or 1)
  );
}

// Function to calculate roll and pitch
//void calculateRollPitch(float accelX, float accelY, float accelZ) {
  void calculateRollPitch(float accelX, float accelY, float accelZ, float gyroX, float gyroY, float gyroZ, float dt) {
  // Calculate roll (rotation around X-axis) in degrees
  // float roll = atan2(accelY, accelZ) * 180 / PI;

  // // Calculate pitch (rotation around Y-axis) in degrees
  // float pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;


  float pitchFromAccel = 0;
  float rollFromAccel  = 0;

  float _pitchGyroFavoring = 0.98f; // Favor gyroscope for pitch
  float _rollGyroFavoring = 0.98f;  // Favor gyroscope for roll
  
  pitchFromAccel = atan(-accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2)));
  rollFromAccel = atan(accelY / sqrt(pow(accelX, 2) + pow(accelZ, 2)));
  // rollFromAccel = atan2(accelerometer.y, accelerometer.z);
  
  // Complimentary Filter
  // subtract gyro bias and update global filtered pitch/roll (radians)
  float gyroX_unbiased = gyroX - gyro_bias_x;
  float gyroY_unbiased = gyroY - gyro_bias_y;

  // integrate gyro rate using measured dt, then blend with accel-derived angle
  g_pitch = (_pitchGyroFavoring) * (g_pitch + (gyroY_unbiased * dt)) + (1.0f - _pitchGyroFavoring) * (pitchFromAccel);
  g_roll  = (_rollGyroFavoring)  * (g_roll  + (gyroX_unbiased * dt))  + (1.0f - _rollGyroFavoring)  * (rollFromAccel);


  // // Print roll and pitch
  // Serial.print("Roll: ");
  // Serial.print(roll);
  // Serial.println(" degrees");

  // Serial.print("Pitch: ");
  // Serial.print(pitch);
  // Serial.println(" degrees");

    // Print roll and pitch
    //Serial.print("-----------------------Original roll and pitch data ---------------");
  // concise output only (detailed prints removed)

  // Publish calibrated telemetry for plotting (will wait until calibration completes)
  publishTelemetry(g_pitch, g_roll);

  //Calculate PID correction for roll
  // To roll the drone 45 degrees and pitch backward 20 degrees while moving forward:
  // setpointRoll should be 45 degrees (converted to radians)
  // setpointPitch should be -20 degrees (converted to radians, negative for backward pitch)
  float setpointRoll = 45.0 * PI / 180.0;    // 45 degrees in radians
  float setpointPitch = -10.0 * PI / 180.0;  // -20 degrees in radians (backward)

  float measuredRoll = g_roll;   // filtered roll value in radians
  float measuredPitch = g_pitch; // filtered pitch value in radians
  // use measured dt for PID timing
  if (dt <= 0.0f) dt = 0.01f;
  float rollCorrection = computePID(rollPID, setpointRoll, measuredRoll, dt);
  float pitchCorrection = computePID(rollPID, setpointPitch, measuredPitch, dt);
}

// Telemetry publisher: applies a simple startup calibration (average of N samples)
// and emits a lightweight telemetry line on `Serial` in degrees:
//   %PITCH,xx.x,ROLL,yy.y\n
void publishTelemetry(float pitch_rad, float roll_rad) {
  const int CAL_SAMPLES = 50; // number of samples to average at startup
  static bool calibrated = false;
  static int cal_count = 0;
  static float cal_pitch_sum = 0.0f;
  static float cal_roll_sum = 0.0f;
  static float offset_pitch = 0.0f;
  static float offset_roll = 0.0f;

  // Collect calibration samples on first calls
  if (!calibrated) {
    cal_pitch_sum += pitch_rad;
    cal_roll_sum += roll_rad;
    cal_count++;
    if (cal_count >= CAL_SAMPLES) {
      offset_pitch = cal_pitch_sum / (float)cal_count;
      offset_roll = cal_roll_sum / (float)cal_count;
      calibrated = true;
    }
    return; // don't publish telemetry until calibrated
  }

  // Apply offsets and convert to degrees
  float pitch_cal = pitch_rad - offset_pitch;
  float roll_cal = roll_rad - offset_roll;
  float pitch_deg = pitch_cal * 180.0f / PI;
  float roll_deg = roll_cal * 180.0f / PI;

  // Format and send telemetry. literal %% produces a single percent in snprintf.
  Serial.printf("%%PITCH,%.1f,ROLL,%.1f\n", pitch_deg, roll_deg);
}

void Mpu6050Readings() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Correct raw readings by subtracting startup biases
  float ax = a.acceleration.x - accel_bias_x;
  float ay = a.acceleration.y - accel_bias_y;
  float az = a.acceleration.z - accel_bias_z;
  float gx = g.gyro.x - gyro_bias_x;
  float gy = g.gyro.y - gyro_bias_y;
  float gz = g.gyro.z - gyro_bias_z;

  // Compute dt from micros() and clamp to reasonable bounds
  unsigned long now = micros();
  float dt = 0.01f; // default 10 ms
  if (last_sensor_micros != 0) {
    dt = (now - last_sensor_micros) / 1000000.0f;
  }
  last_sensor_micros = now;
  if (dt <= 0.0f) dt = 0.001f;
  if (dt > 0.5f) dt = 0.5f;

  // Calculate and print roll and pitch using bias-corrected readings and measured dt
  calculateRollPitch(ax, ay, az, gx, gy, gz, dt);

  // Compute linear accelerations by removing gravity projection
  // Using pitch (theta) and roll from complementary filter (radians)
  float lin_ax = ax - (GRAVITY_MS2 * sin(g_pitch));
  float lin_ay = ay - (GRAVITY_MS2 * sin(g_roll));

  // Concise orientation + linear-accel log for debugging/plotting
  float pitch_deg = g_pitch * 180.0f / PI;
  float roll_deg = g_roll * 180.0f / PI;
  Serial.printf("ORIENT P=%.2fdeg R=%.2fdeg | LIN_AX=%.3f m/s2 LIN_AY=%.3f\n", pitch_deg, roll_deg, lin_ax, lin_ay);
  // no blocking delay here; task controls sample rate
}

void loop(){
  // Empty: all work is done in FreeRTOS tasks
}