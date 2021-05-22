#include "imu.h"

#define DEFAULT_ALPHA 0.98

IMU::IMU(Adafruit_MPU6050 *mpu) {
  _mpu = mpu;
  _alpha = DEFAULT_ALPHA;
  _last_reading_us = micros();  
}

IMU::IMU(Adafruit_MPU6050 *mpu, double filter_alpha) {
  _mpu = mpu;
  _alpha = filter_alpha;
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
  double dt = 1e-6 * (now_us - _last_reading_us);
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

  roll_pitch_t accel_rp = getAccelRollPitch();
  _filtered_roll = _alpha * (_filtered_roll + _gyro.x * dt) + (1.0 - _alpha) * accel_rp.roll;
  _filtered_pitch = _alpha * (_filtered_pitch + _gyro.y * dt) + (1.0 - _alpha) * accel_rp.pitch;
}

roll_pitch_t IMU::getGyroRollPitch() {
  return { _gyro_roll, _gyro_pitch };
}

roll_pitch_t IMU::getAccelRollPitch() {
  return {
    atan2(-_accel.x, _accel.z),
    atan2(_accel.y, sqrt(_accel.x * _accel.x + _accel.z * _accel.z))
  };
}

roll_pitch_t IMU::getFilteredRollPitch() {
  return { _filtered_roll, _filtered_pitch };
}
