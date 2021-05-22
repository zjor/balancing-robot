#include <Arduino.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "imu/imu.h"
#include "util/logger.h"

Logger logger(false);

const float accel_offset[3] = {0.8864, -0.1769, -1.1098};
const float gyro_offset[3] = {-0.0160, 0.0215, -0.0261};

Adafruit_MPU6050 mpu;
IMU imu(mpu);

roll_pitch_t getAccelRollPitch(float ax, float ay, float az) {
  const float alpha = 0.2;
  static float fx = ax;
  static float fy = ay;
  static float fz = az;

  fx = alpha * ax + (1.0 - alpha) * fx;
  fy = alpha * ay + (1.0 - alpha) * fy;
  fz = alpha * az + (1.0 - alpha) * fz;

  roll_pitch_t r;
  // r.roll = atan2(-fy, fz);
  r.roll = atan2(-fy, sqrt(fx * fx + fz * fz));
  r.pitch = atan2(fx, sqrt(fy * fy + fz * fz));

  return r;
}

roll_pitch_t getRawAccelRollPitch(float ax, float ay, float az) {
  roll_pitch_t r;
  // r.roll = atan2(-ay, az);
  r.roll = atan2(-ay, sqrt(ax * ax + az * az));
  r.pitch = atan2(ax, sqrt(ay * ay + az * az));

  return r;
}

roll_pitch_t getGyroRollPitch(float gx, float gy, float gz, float roll_accel, float pitch_accel) {
  static unsigned long ts = micros();
  static float roll = roll_accel;
  static float pitch = pitch_accel;

  unsigned long now = micros();
  float dt = 1e-6 * (now - ts);
  ts = now;

  roll += -gy * dt;
  pitch += -gx * dt;

  const float a = 0.91;
  roll = a * roll + (1.0 - a) * roll_accel;
  pitch = a * pitch + (1.0 - a) * pitch_accel;

  roll_pitch_t r;
  r.roll = roll;
  r.pitch = pitch;
  return r;
}

void calibrate() {  
  Serial.println("Calibrating...");

  tuple_t<vector3d_t, vector3d_t> offsets = imu.calibrate(2000, 250);

  Serial.print("Acc offset: ");
  Serial.print(offsets.a.x, 4);

  Serial.print(", ");
  Serial.print(offsets.a.y, 4);

  Serial.print(", ");
  Serial.println(offsets.a.z, 4);

  Serial.print("Gyro offset: ");
  Serial.print(offsets.b.x, 4);

  Serial.print(", ");
  Serial.print(offsets.b.y, 4);

  Serial.print(", ");
  Serial.println(offsets.b.z, 4);  
}

void setup(void) {
  Serial.begin(115200);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(500);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

  // calibrate();
}

void loop() {
  // return;
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  logger.info("%.4f\t%.4f\n", a.acceleration.x, g.gyro.x);

  delay(250);
}