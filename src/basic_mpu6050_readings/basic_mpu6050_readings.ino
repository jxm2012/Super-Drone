#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

//Reference: https://forum.arduino.cc/t/connecting-multi-mpu6050-using-i2c-on-esp32-wroom-32d/1175063

#define SDA_PIN 21 // Custom SDA pin
#define SCL_PIN 22 // Custom SCL pin

Adafruit_MPU6050 mpu;

struct PID {
  float kp, ki, kd;
  float prev_error;
  float integral;
};

void TaskReadMPU(void *pvParameters) {
  // Sensor reading task for FreeRTOS
  for (;;) {
    Serial.println("Reading MPU6050 data...");
    Mpu6050Readings();
    vTaskDelay(pdMS_TO_TICKS(500)); // 500 ms delay
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

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

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
  Serial.println("");
  delay(100);

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
  void calculateRollPitch(float accelX, float accelY, float accelZ, float gyroX, float gyroY, float gyroZ) {
  // Calculate roll (rotation around X-axis) in degrees
  // float roll = atan2(accelY, accelZ) * 180 / PI;

  // // Calculate pitch (rotation around Y-axis) in degrees
  // float pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;


  float pitchFromAccel = 0;
  float rollFromAccel  = 0;

  float _filterUpdateRate = 2.0; // Update rate in Hz (match your sensor reading frequency)
  float _pitchGyroFavoring = 0.98; // Favor gyroscope for pitch
  float _rollGyroFavoring = 0.98;  // Favor gyroscope for roll
  
  pitchFromAccel = atan(-accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2)));
  rollFromAccel = atan(accelY / sqrt(pow(accelX, 2) + pow(accelZ, 2)));
  // rollFromAccel = atan2(accelerometer.y, accelerometer.z);
  
  // Complimentary Filter
  static float _pitch = 0, _roll = 0;
  _pitch = (_pitchGyroFavoring) * (_pitch + (gyroY * (1.00 / _filterUpdateRate))) + (1.00 - _pitchGyroFavoring) * (pitchFromAccel);
  _roll = (_rollGyroFavoring) * (_roll + (gyroX * (1.00 / _filterUpdateRate))) + (1.00 - _rollGyroFavoring) * (rollFromAccel);


  // // Print roll and pitch
  // Serial.print("Roll: ");
  // Serial.print(roll);
  // Serial.println(" degrees");

  // Serial.print("Pitch: ");
  // Serial.print(pitch);
  // Serial.println(" degrees");

    // Print roll and pitch
    //Serial.print("-----------------------Original roll and pitch data ---------------");
  Serial.print("Roll: ");
  Serial.print(rollFromAccel);
  Serial.println(" degrees");

  Serial.print("Pitch: ");
  Serial.print(pitchFromAccel);
  Serial.println(" degrees");

  //Serial.print("-----------------------After filtering data using complimentary filter---------------");
  Serial.print("Complimentary Roll: ");
  Serial.print(_roll);
  Serial.println(" degrees");

  Serial.print("Complimentary Pitch: ");
  Serial.print(_pitch);
  Serial.println(" degrees");

  // Log the degree angles for both roll and pitch (filtered and accel)
  Serial.print("[LOG] Roll (deg): ");
  Serial.print(rollFromAccel * 180.0 / PI);
  Serial.print(", Pitch (deg): ");
  Serial.print(pitchFromAccel * 180.0 / PI);
  Serial.print(" | Complimentary Roll (deg): ");
  Serial.print(_roll * 180.0 / PI);
  Serial.print(", Complimentary Pitch (deg): ");
  Serial.println(_pitch * 180.0 / PI);

  //Calculate PID correction for roll
  // To roll the drone 45 degrees and pitch backward 20 degrees while moving forward:
  // setpointRoll should be 45 degrees (converted to radians)
  // setpointPitch should be -20 degrees (converted to radians, negative for backward pitch)
  float setpointRoll = 45.0 * PI / 180.0;    // 45 degrees in radians
  float setpointPitch = -10.0 * PI / 180.0;  // -20 degrees in radians (backward)

  float measuredRoll = _roll;   // filtered roll value in radians
  float measuredPitch = _pitch; // filtered pitch value in radians
  float dt = 0.5; // seconds, matches your filter update rate

  float rollCorrection = computePID(rollPID, setpointRoll, measuredRoll, dt);
  Serial.print("Roll Correction: ");
  Serial.println(rollCorrection);

  float pitchCorrection = computePID(rollPID, setpointPitch, measuredPitch, dt);
  Serial.print("Pitch Correction: ");
  Serial.println(pitchCorrection);
}

void Mpu6050Readings() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.println("Reading sensor data...");

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

   // Calculate and print roll and pitch
  calculateRollPitch(a.acceleration.x, a.acceleration.y, a.acceleration.z, g.gyro.x, g.gyro.y, g.gyro.z);

  Serial.println("");
  delay(500);
}

void loop(){
  // Empty: all work is done in FreeRTOS tasks
}