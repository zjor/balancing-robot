#include "imu.h"
#include "../util/logger.h"

Logger _imu_logger(true);

IMU::IMU(Adafruit_MPU6050 *mpu) {
  _mpu = mpu;
  _last_reading_us = micros();  
}

tuple_t<vector3d_t, vector3d_t> IMU::calibrate(unsigned int samples_count, unsigned long delay_ms) {
  vector3d_t accel_offset = {.0, .0, .0};
  vector3d_t gyro_offset = {.0, .0, .0};

  sensors_event_t a, g, temp;

  for (int i = 0; i < samples_count; i++) {
    _mpu->getEvent(&a, &g, &temp);

    accel_offset.x += a.acceleration.x;
    accel_offset.y += a.acceleration.y;
    accel_offset.z += (a.acceleration.z - G);

    gyro_offset.x += g.gyro.x;
    gyro_offset.y += g.gyro.y;
    gyro_offset.z += g.gyro.z;

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

void IMU::setAccelOffset(vector3d_t v) {
  _accel_offset = { v.x, v.y, v.z };
}

void IMU::setGyroOffset(vector3d_t v) {
  _gyro_offset = { v.x, v.y, v.z };
}

void IMU::update() {
  unsigned long now_us = micros();
  float dt = 1e-6 * (now_us - _last_reading_us);
  _last_reading_us = now_us;

  sensors_event_t a, g, temp;  
  _mpu->getEvent(&a, &g, &temp);
  _accel = {
    a.acceleration.x - _accel_offset.x,
    a.acceleration.y - _accel_offset.y,
    a.acceleration.z - _accel_offset.z,
  };

  _gyro = {
    g.gyro.x - _gyro_offset.x,
    g.gyro.y - _gyro_offset.y,
    g.gyro.z - _gyro_offset.z,
  };

  _gyro_roll += _gyro.x * dt;
  _gyro_pitch += _gyro.y * dt;  
}

roll_pitch_t IMU::getGyroRollPitch() {
  return { _gyro_roll, _gyro_pitch };
}
