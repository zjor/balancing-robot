#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include "imu/imu.h"
#include "PID.h"
#include "TimedTask.h"


#define RIGHT_MOTOR_STEP_PIN  15
#define RIGHT_MOTOR_DIR_PIN   32
#define LEFT_MOTOR_STEP_PIN   17
#define LEFT_MOTOR_DIR_PIN    21

#define PPR       1600

#define TICKS_PER_SECOND 200000

#define SET_POINT 0.0


const bool shouldCalibrate = false;

Adafruit_MPU6050 mpu;
IMU imu(&mpu, 0.95);

PID pid(5.0, 10.0, 0.0, SET_POINT * DEG_TO_RAD);
bool is_balancing = false;

hw_timer_t * timer = NULL;
volatile uint32_t ticksPerPulse = TICKS_PER_SECOND;
volatile uint32_t currentTick = 0;

double angle;
double velocity;

void initMPU() {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(500);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
}

void calibrate() {  
  if (shouldCalibrate) {
    Serial.println("Calibrating...");
    tuple_t<vector3d_t, vector3d_t> offsets = imu.calibrate(2000, 10);
    Serial.printf("Accel offset: %.4f, %.4f %.4f\n", offsets.a.x, offsets.a.y, offsets.a.z);
    Serial.printf("Gyro offset: %.4f, %.4f %.4f\n", offsets.b.x, offsets.b.y, offsets.b.z);
  } else {
    imu.setAccelOffset({ -1.5999, 0.4344, -1.7590 });
    imu.setGyroOffset({ -0.0254, 0.0050, -0.0075 });
  }
}

int getTicksPerPulse(float velocity) {
  if (abs(velocity) < 1e-3) {
    return UINT32_MAX;
  } else {
    return (uint32_t)(2.0 * PI * TICKS_PER_SECOND / (abs(velocity) * PPR));
  }  
}

void setVelocity(float velocity) {
  digitalWrite(LEFT_MOTOR_DIR_PIN, velocity > 0 ? HIGH : LOW);
  digitalWrite(RIGHT_MOTOR_DIR_PIN, velocity < 0 ? HIGH : LOW);
  ticksPerPulse = getTicksPerPulse(velocity);
}

void IRAM_ATTR onTimer() {
  if (currentTick >= ticksPerPulse) {
    currentTick = 0;
  }
  if (currentTick == 0) {
    digitalWrite(LEFT_MOTOR_STEP_PIN, HIGH);
    digitalWrite(RIGHT_MOTOR_STEP_PIN, HIGH);
  } else if (currentTick == 1) {
    digitalWrite(LEFT_MOTOR_STEP_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_STEP_PIN, LOW);
  }
  currentTick++; 
}

void log(unsigned long now_millis, unsigned long dt_millis) {
  Serial.printf("%.4f\t%.4f\n", 
    angle, 
    velocity);
}

TimedTask loggerTask(log, 50);

void control(unsigned long now_millis, unsigned long dt_millis) {
  float dt = dt_millis * 1e-3;

  if (abs(angle - SET_POINT) < 5.0) {
    is_balancing = true;
  }

  if (abs(angle - SET_POINT) > 50.0) {
    is_balancing = false;
    velocity = 0.0;
    setVelocity(velocity);
  }

  if (!is_balancing) {
    return;
  }

  float u = pid.getControl(angle * DEG_TO_RAD, dt);
  velocity += u * dt;
  setVelocity(velocity);  
}

TimedTask controlTask(control, 7);

void setup(void) {
  Serial.begin(115200);

  initMPU();
  calibrate();
}

void loop() {
  if (shouldCalibrate) {
    return;
  }
  loggerTask.loop();
  controlTask.loop();
  imu.update();

  roll_pitch_t f_rp = imu.getFilteredRollPitch();
  angle = f_rp.roll * RAD_TO_DEG;

}