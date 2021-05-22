#include "imu.h"

IMU::IMU(Adafruit_MPU6050 mpu) {
  _mpu = mpu;
  _last_reading_us = micros();
}

tuple_t<vector3d_t, vector3d_t> IMU::calibrate(unsigned int samples_count, unsigned long delay_ms) {
  vector3d_t accel_offset = {.0, .0, .0};
  vector3d_t gyro_offset = {.0, .0, .0};

  sensors_event_t a, g, temp;

  for (int i = 0; i < samples_count; i++) {
    _mpu.getEvent(&a, &g, &temp);

    accel_offset.x += a.acceleration.x;
    accel_offset.y += a.acceleration.y;
    accel_offset.z += (a.acceleration.z - G);

    gyro_offset.x += g.gyro.x;
    gyro_offset.y += g.gyro.y;
    gyro_offset.z += g.gyro.z;

    Serial.print(a.acceleration.x);
    Serial.print("\t");
    Serial.println(g.gyro.x);

    delay(delay_ms);
  }

  accel_offset.x /= samples_count;
  accel_offset.y /= samples_count;
  accel_offset.z /= samples_count;

  gyro_offset.x /= samples_count;
  gyro_offset.y /= samples_count;
  gyro_offset.z /= samples_count;

  return { accel_offset, gyro_offset };
}