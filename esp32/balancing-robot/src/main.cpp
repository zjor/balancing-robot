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
#define PULSE_WIDTH      2

#define SET_POINT (-2.0 * DEG_TO_RAD)


const bool shouldCalibrate = false;

Adafruit_MPU6050 mpu;
IMU imu(&mpu, 0.95);

PID angle_pid(25.0, 75.0, 0.0, SET_POINT);
PID velocity_pid(0.001, 0.0, 0.0001, 0.0);
bool is_balancing = false;

hw_timer_t * timer = NULL;
volatile uint32_t ticksPerPulse = TICKS_PER_SECOND;
volatile uint32_t currentTick = 0;

double angle;
double angle_filtered = SET_POINT;
double lp_alpha = 0.05;
double angle_target = SET_POINT;
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
    return (uint32_t)(2.0 * PI * TICKS_PER_SECOND / (abs(velocity) * PPR)) - PULSE_WIDTH;
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
  } else if (currentTick == PULSE_WIDTH) {
    digitalWrite(LEFT_MOTOR_STEP_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_STEP_PIN, LOW);
  }
  currentTick++; 
}

void log(unsigned long now_millis, unsigned long dt_millis) {
  Serial.printf("lp_a:%.4f\tangle:%.4f\ttarget:%.4f\tvelocity:%.4f\n", 
    angle_filtered * RAD_TO_DEG,
    angle * RAD_TO_DEG, 
    angle_target * RAD_TO_DEG, 
    velocity);
}

TimedTask loggerTask(log, 25);

void control(unsigned long now_millis, unsigned long dt_millis) {
  float dt = dt_millis * 1e-3;

  if (abs(angle_filtered - angle_target) < PI / 18) {
    is_balancing = true;
  }

  if (abs(angle_filtered - angle_target) > PI / 4) {
    is_balancing = false;
    velocity = 0.0;
    setVelocity(velocity);
  }

  if (!is_balancing) {
    return;
  }
  angle_target = velocity_pid.getControl(-velocity, dt);
  angle_pid.setTarget(angle_target);

  float u = -angle_pid.getControl(angle_filtered, dt);
  velocity += u * dt;
  setVelocity(velocity);  
}

TimedTask controlTask(control, 7);

void setup(void) {
  Serial.begin(115200);

  pinMode(LEFT_MOTOR_STEP_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_STEP_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);

  digitalWrite(LEFT_MOTOR_STEP_PIN, LOW);
  digitalWrite(LEFT_MOTOR_DIR_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_STEP_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_DIR_PIN, LOW);


  initMPU();
  calibrate();

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 5, true);
  timerAlarmEnable(timer);  

  setVelocity(0.0);
}

void loop() {
  if (shouldCalibrate) {
    return;
  }
  loggerTask.loop();
  controlTask.loop();
  imu.update();

  roll_pitch_t f_rp = imu.getFilteredRollPitch();
  angle = f_rp.roll;
  angle_filtered = lp_alpha * angle + (1.0 - lp_alpha) * angle_filtered;

}