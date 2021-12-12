# Test IMU MPU9250 Sketch

## platformio.ini

```
[env:esp32thing_plus]
monitor_speed = 115200
platform = espressif32
board = esp32doit-devkit-v1
framework = arduino
lib_deps = 
	sparkfun/SparkFun MPU-9250 Digital Motion Processing (DMP) Arduino Library@^1.0.0
```

## Source

```C++
#include <Arduino.h>
#include <SparkFunMPU9250-DMP.h>
#include <Wire.h>

#define R_LED_PIN 32
#define G_LED_PIN 33
#define B_LED_PIN 25

#define R_CH 0
#define G_CH 1
#define B_CH 2

#define INTERRUPT_PIN 19

MPU9250_DMP imu;

volatile bool dmpDataReady = false;

void measureCycles();

void dmpISR() {
  dmpDataReady = true;
}

void printRawAccel() {
  static long accel_counter = 0;
  static float avg_x = 0;
  static float avg_y = 0;
  static float avg_z = 0;

  avg_x += imu.ax;
  avg_y += imu.ay;
  avg_z += (imu.az - 16384);
  accel_counter++;

  if (accel_counter >= 100) {
    avg_x /= accel_counter;
    avg_y /= accel_counter;
    avg_z /= accel_counter;

    Serial.print(avg_x, 2);
    Serial.print("\t");
    Serial.print(avg_y, 2);
    Serial.print("\t");
    Serial.println(avg_z, 2);
    accel_counter = 0;
  }

}

float normalize_angle(float angle) {
  return (angle < 180) ? angle : angle - 360.0f;
}

void printIMUData(void) {
  float roll = normalize_angle(imu.roll);
  float pitch = normalize_angle(imu.pitch);
  float yaw = normalize_angle(imu.yaw);

  Serial.print("r:");
  Serial.print(roll);
  Serial.print("\tp:");  
  Serial.print(pitch);
  Serial.print("\ty:");
  Serial.println(yaw);
  
  ledcWrite(R_CH, map((int)fabs(roll), 0, 120, 0, 255));
  ledcWrite(G_CH, map((int)fabs(pitch), 0, 120, 0, 255));
  ledcWrite(B_CH, map((int)fabs(yaw), 0, 180, 0, 255));

  // float gyroX = imu.calcGyro(imu.gx);
  // float gyroY = imu.calcGyro(imu.gy);
  // float gyroZ = imu.calcGyro(imu.gz);

  // Serial.print(gyroX, 4); Serial.print("\t");
  // Serial.print(gyroY, 4); Serial.print("\t");
  // Serial.println(gyroZ, 4);
}

void setup() {
  Serial.begin(115200);

  pinMode(INTERRUPT_PIN, INPUT_PULLUP);

  if (imu.begin() != INV_SUCCESS) {
    while (1) {
      Serial.println("Failed to init IMU");
      delay(5000);
    }
  }

  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL, 100);
  // -1030.98        -186.85 -1135.15
  // long bias[] = {-1030, -186, -1135};
  // imu.dmpSetAccelBias(bias); 

  // imu.setSampleRate(100);
  // imu.configureFifo(INV_XYZ_ACCEL);   
  imu.enableInterrupt();
  imu.setIntLevel(INT_ACTIVE_LOW);
  imu.setIntLatched(INT_LATCHED);

  attachInterrupt(INTERRUPT_PIN, dmpISR, FALLING);


  ledcAttachPin(R_LED_PIN, R_CH);
  ledcAttachPin(G_LED_PIN, G_CH);
  ledcAttachPin(B_LED_PIN, B_CH);

  ledcSetup(R_CH, 1000, 8);
  ledcSetup(G_CH, 1000, 8);
  ledcSetup(B_CH, 1000, 8);
}

void loop() {
  // if (imu.dataReady()) {
  //   imu.update(UPDATE_ACCEL);
  //   printRawAccel();
  // }
  if (dmpDataReady && imu.fifoAvailable()) {
    if (imu.dmpUpdateFifo() == INV_SUCCESS) {
      dmpDataReady = false;
      imu.computeEulerAngles();
      printIMUData();
    }
  }

  // measureCycles();
}

void measureCycles() {
  static unsigned long counter = 0;
  static unsigned long timestamp = 0;
  unsigned long now = millis();
  counter++;

  if (now - timestamp >= 1000) {
    Serial.println(counter);
    counter = 0;
    timestamp = now;
  }
}
```