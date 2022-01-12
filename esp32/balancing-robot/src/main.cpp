/**

 * # LED Wiring

 * - RED: GPIO32
 * - GREEN: GPIO33
 * - BLUE: GPIO25
 *
 * 
 * # MPU9250 Wiring
 * 
 * - SCL: GPIO22
 * - SDA: GPIO21
 * - INT: GPIO19
 * 
 * 
 * # Motor 1 Wiring
 * 
 * - EN: GPIO15
 * - DIR: GPIO14
 * - STEP: GPIO12
 * 
 * 
 * # Motor 2 Wiring
 * 
 * - EN: GPIO16
 * - DIR: GPIO26
 * - STEP: GPIO27
 * 
 * 
 * @author Sergey Royz (zjor.se@gmail.com) 
 * @version 0.1
 * @date 2021-12-28
 * 
 * 
 */
#include <Arduino.h>
#include <Wire.h>
#include <SparkFunMPU9250-DMP.h>

#include "pid/PID.h"
#include "stepper/Stepper.h"

#define PIN_IMU_SCL 22
#define PIN_IMU_SDA 21
#define PIN_IMU_INT 19

#define PIN_MOTOR_RIGHT_EN    15
#define PIN_MOTOR_RIGHT_DIR   14
#define PIN_MOTOR_RIGHT_STEP  12

#define PIN_MOTOR_LEFT_EN    16
#define PIN_MOTOR_LEFT_DIR   26
#define PIN_MOTOR_LEFT_STEP  27

#define PPR       1600

#define TICKS_PER_SECOND 200000
#define PULSE_WIDTH      1

#define MAX_ACCEL (200)
#define ANGLE_Kp  450.0
#define ANGLE_Kd  30.0
#define ANGLE_Ki  0.0

#define VELOCITY_Kp  0.007
#define VELOCITY_Kd  0.0
#define VELOCITY_Ki  0.0005

#define WARMUP_DELAY_US (5000000UL)

#define ANGLE_SET_POINT (2.0 * DEG_TO_RAD)

// #define LOG_IMU

void initTimerInterrupt();

float normalizeAngle(float);

void updateVelocity(unsigned long);

Stepper leftStepper(PIN_MOTOR_LEFT_EN, PIN_MOTOR_LEFT_DIR, PIN_MOTOR_LEFT_STEP, TICKS_PER_SECOND, PPR, PULSE_WIDTH);
Stepper rightStepper(PIN_MOTOR_RIGHT_EN, PIN_MOTOR_RIGHT_DIR, PIN_MOTOR_RIGHT_STEP, TICKS_PER_SECOND, PPR, PULSE_WIDTH);

MPU9250_DMP imu;
volatile bool dmpDataReady = false;
float roll, pitch, yaw;

void dmpISR() {
  dmpDataReady = true;
}

void initIMU() {
  pinMode(PIN_IMU_INT, INPUT_PULLUP);

  if (imu.begin() != INV_SUCCESS) {
    while (1) {
      Serial.println("Failed to init IMU");
      delay(5000);
    }
  }

  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL, 100);
  imu.enableInterrupt();
  imu.setIntLevel(INT_ACTIVE_LOW);
  imu.setIntLatched(INT_LATCHED);

  attachInterrupt(PIN_IMU_INT, dmpISR, FALLING);
}

void readIMU() {
  if (dmpDataReady && imu.fifoAvailable()) {
    if (imu.dmpUpdateFifo() == INV_SUCCESS) {
      dmpDataReady = false;
      imu.computeEulerAngles();
      roll = imu.roll;
      pitch = imu.pitch;
      yaw = imu.yaw;
    }
  }
}

void logIMU() {
  #ifdef LOG_IMU
  static unsigned long lastTimestamp = millis();
  unsigned long now = millis();
  if (now - lastTimestamp > 50) {
    // Serial.print("r:"); Serial.print(roll); Serial.print("\t");
    // Serial.print("p:"); Serial.print(pitch); Serial.print("\t");
    // Serial.print("y:"); Serial.println(yaw);
    Serial.print("r:");Serial.print(roll);Serial.print("\t");
    Serial.println(normalizeAngle(roll));
    lastTimestamp = now;
  }
  #endif
}

PID angle_pid(ANGLE_Kp, ANGLE_Kd, ANGLE_Ki, ANGLE_SET_POINT);
PID velocity_pid(0.00, 0.0, 0.0, 0.0);
bool is_balancing = false;

hw_timer_t * timer = NULL;

void IRAM_ATTR onTimer() {
  leftStepper.tick();
  rightStepper.tick();
}

float normalizeAngle(float value) { 
  return ((value < 180) ? value : value - 360.0f) * DEG_TO_RAD;
}

void setup(void) {
  // setCpuFrequencyMhz(240);

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(1000000UL);

  initTimerInterrupt();

  initIMU();

  leftStepper.init();
  rightStepper.init();
  leftStepper.setEnabled(true);
  rightStepper.setEnabled(true);
}

void loop() {
  unsigned long nowMicros = micros();
  readIMU();
  updateVelocity(nowMicros);
  logIMU();
}

void initTimerInterrupt() {
  // 80MHz / 80 / 5 = 200kHz
  timer = timerBegin(0 /* timer ID */, 80 /* CPU frequency divider */, true /* count up */);
  timerAttachInterrupt(timer, &onTimer, true /* edge */);
  timerAlarmWrite(timer, 5 /* int at counter value */, true /* reload counter */);
  timerAlarmEnable(timer);
}

void updateVelocity(unsigned long nowMicros) {
  static unsigned long lastUpdateTimestamp = micros();
  if (nowMicros - lastUpdateTimestamp < 100 /* 10 kHz */) {
    return;
  }

  float angle = normalizeAngle(roll);
  float velocity = (abs(angle) < 0.5) ? 20.0 * angle : 0.0f;
  leftStepper.setVelocity(-velocity);
  rightStepper.setVelocity(velocity);
}