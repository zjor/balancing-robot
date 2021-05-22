#ifndef _IMU_H
#define _IMU_H

#include <Adafruit_MPU6050.h>

#define G 9.81

template<typename A, typename B>
struct tuple_t {
 A a;
 B b;
};

typedef struct {
  float x, y, z;
} vector3d_t;

typedef struct {
  float roll, pitch;
} roll_pitch_t;

class IMU {
public:
  IMU(Adafruit_MPU6050 mpu);
  tuple_t<vector3d_t, vector3d_t> calibrate(unsigned int samples_count, unsigned long delay_ms);
  void setAccelOffset(vector3d_t v);
  void setGyroOffset(vector3d_t v);
  void update();
  roll_pitch_t getAccelRollPitch();
  roll_pitch_t getGyroRollPitch();
  roll_pitch_t getFilteredRollPitch();

private:
  Adafruit_MPU6050 _mpu;
  vector3d_t _accel_offset = {.0, .0, .0};
  vector3d_t _gyro_offset = {.0, .0, .0};
  float _gyro_roll = .0;
  float _gyro_pitch = .0;
  unsigned long _last_reading_us;
};

#endif