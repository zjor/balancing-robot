#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include "imu/imu.h"
#include "PID.h"
#include "TimedTask.h"


#define RIGHT_MOTOR_STEP_PIN  12
#define RIGHT_MOTOR_DIR_PIN   27
#define LEFT_MOTOR_STEP_PIN   15
#define LEFT_MOTOR_DIR_PIN    14

#define PPR       1600

#define TICKS_PER_SECOND 200000
#define PULSE_WIDTH      1

#define MAX_U     (200)
#define MAX_V     (10)
#define SET_POINT (-2.2 * DEG_TO_RAD)

#define ANGLE_Kp  800.0
#define ANGLE_Kd  10.0
#define ANGLE_Ki  0.0


const bool shouldCalibrate = false;

Adafruit_MPU6050 mpu;
IMU imu(&mpu, 0.98);

PID angle_pid(ANGLE_Kp, ANGLE_Kd, ANGLE_Ki, SET_POINT);
PID velocity_pid(0.00, 0.0, 0.00, 0.0);
bool is_balancing = false;

hw_timer_t * timer = NULL;
volatile uint32_t ticksPerPulse = TICKS_PER_SECOND;
volatile uint32_t currentTick = 0;

double angle;
double angle_target = SET_POINT;
double velocity = 0.0;
double u = 0.0;

void initMPU() {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(500);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
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

void setVelocity(float _velocity) {
  // _velocity = -_velocity;
  digitalWrite(RIGHT_MOTOR_DIR_PIN, _velocity < 0 ? HIGH : LOW);  
  digitalWrite(LEFT_MOTOR_DIR_PIN, _velocity > 0 ? HIGH : LOW);
  ticksPerPulse = getTicksPerPulse(_velocity);
}

void IRAM_ATTR onTimer() {
  if (currentTick >= ticksPerPulse) {
    currentTick = 0;
  }
  if (currentTick == 0) {
    digitalWrite(RIGHT_MOTOR_STEP_PIN, HIGH);
    digitalWrite(LEFT_MOTOR_STEP_PIN, HIGH);
  } else if (currentTick == PULSE_WIDTH) {    
    digitalWrite(RIGHT_MOTOR_STEP_PIN, LOW);
    digitalWrite(LEFT_MOTOR_STEP_PIN, LOW);    
  }
  currentTick++; 
}

void log(unsigned long now_us, unsigned long dt_us) {
  Serial.printf("angle:%.4f\ttarget:%.4f\tvelocity:%.4f\tu:%.4f\n", 
    angle * RAD_TO_DEG, 
    angle_target * RAD_TO_DEG, 
    velocity,
    u);
}

TimedTask loggerTask(log, 10 * 1000);

void control(unsigned long now_us, unsigned long dt_us) {
  float dt = dt_us * 1e-6;

  if (abs(angle - angle_target) < PI / 18) {
    is_balancing = true;
  }

  if (abs(angle - angle_target) > PI / 4) {
    is_balancing = false;
    u = 0.0;
    velocity = 0.0;
    setVelocity(velocity);
  }

  if (!is_balancing) {
    return;
  }
  // angle_target = velocity_pid.getControl(-velocity, dt);
  // angle_pid.setTarget(angle_target);

  u = angle_pid.getControl(angle, dt);
  u = constrain(u, -MAX_U, MAX_U);  
}

TimedTask controlTask(control, 1 * 1000);

void updateVelocity(unsigned long now_us, unsigned long dt_us) {
  float dt = dt_us * 1e-6;
  velocity += u * dt;
  velocity = constrain(velocity, -MAX_V, MAX_V);
  setVelocity(velocity);  
}

void setup(void) {  
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000UL);

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

  u = 0.0;
  velocity = 0.0;
  setVelocity(0.0);
}

unsigned long last_velocity_update_us = 0.0;

// unsigned long ctr = 0;
// int i = 1000;

void loop() {
  if (shouldCalibrate) {
    return;
  }

  unsigned long now = micros();
  updateVelocity(now, (now - last_velocity_update_us));
  // ctr += now - last_velocity_update_us;
  last_velocity_update_us = now;

  // i--;
  // if (i == 0) {
  //   ctr /= 1000;
  //   Serial.println(ctr);
  //   ctr = 0;
  //   i = 1000;
  // }

  loggerTask.loop();
  controlTask.loop();
  imu.update();

  roll_pitch_t f_rp = imu.getFilteredRollPitch();
  angle = -f_rp.pitch;
}