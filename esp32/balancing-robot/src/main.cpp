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
#include "indicator/Indicator.h"

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

#define CPU_FREQ_MHZ  240
#define CPU_FREQ_DIVIDER  120
#define TICKS_PER_SECOND (200000 * (CPU_FREQ_MHZ / CPU_FREQ_DIVIDER))
#define PULSE_WIDTH      1

#define MAX_ACCEL (200)
#define ANGLE_Kp  800.0
#define ANGLE_Kd  60.0
#define ANGLE_Ki  0.0

#define VELOCITY_Kp  0.005
#define VELOCITY_Kd  0.0
#define VELOCITY_Ki  0.000002

#define WARMUP_DELAY_US (7000000UL)

#define ANGLE_SET_POINT (1.5 * DEG_TO_RAD)

// #define LOG_IMU
// #define LOG_ENABLED

void initTimerInterrupt();

float normalizeAngle(float);

void updateVelocity(unsigned long);
void updateControl(unsigned long);
void log(unsigned long);

Indicator indicator;

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

bool readIMU() {
  if (dmpDataReady && imu.fifoAvailable()) {
    if (imu.dmpUpdateFifo() == INV_SUCCESS) {
      dmpDataReady = false;
      imu.computeEulerAngles();
      roll = imu.roll;
      pitch = imu.pitch;
      yaw = imu.yaw;
      return true;
    }
  }
  return false;
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

PID anglePID(ANGLE_Kp, ANGLE_Kd, ANGLE_Ki, ANGLE_SET_POINT);
PID velocityPID(VELOCITY_Kp, VELOCITY_Kd, VELOCITY_Ki, 0.0);
bool isBalancing = false;

float accel = 0.0f;
float velocity = 0.0f; 
float angle = 0.0f;
float targetAngle = ANGLE_SET_POINT;

hw_timer_t * timer = NULL;

void IRAM_ATTR onTimer() {
  leftStepper.tick();
  rightStepper.tick();
}

float normalizeAngle(float value) { 
  return ((value < 180) ? value : value - 360.0f) * DEG_TO_RAD;
}

void setup(void) {
  setCpuFrequencyMhz(CPU_FREQ_MHZ);

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(1000000UL);

  initTimerInterrupt();

  initIMU();

  leftStepper.init();
  rightStepper.init();
  leftStepper.setEnabled(true);
  rightStepper.setEnabled(true);
  
  indicator.init();
  indicator.setColor(0xFF0000);
}

void loop() {
  unsigned long nowMicros = micros();
  updateVelocity(nowMicros);
  updateControl(nowMicros);
  logIMU();
  #ifdef LOG_ENABLED
  log(nowMicros);
  #endif
}

void initTimerInterrupt() {
  // 80MHz / 80 / 5 = 200kHz  
  timer = timerBegin(0 /* timer ID */, CPU_FREQ_DIVIDER /* CPU frequency divider */, true /* count up */);
  timerAttachInterrupt(timer, &onTimer, true /* edge */);
  timerAlarmWrite(timer, 5 /* int at counter value */, true /* reload counter */);
  timerAlarmEnable(timer);
}

void updateVelocity(unsigned long nowMicros) {
  static unsigned long timestamp = micros();
  if (nowMicros - timestamp < 50 /* 20 kHz */) {
    return;
  }

  float dt = ((float) (nowMicros - timestamp)) * 1e-6;
  velocity += accel * dt;

  leftStepper.setVelocity(velocity);
  rightStepper.setVelocity(-velocity);
  
  timestamp = nowMicros;
}

void setBalancing(bool balancing) {
  if (isBalancing != balancing) {
    isBalancing = balancing;
    #ifdef LOG_ENABLED
    Serial.print("IsBalancing: ");
    Serial.println(isBalancing);
    #endif
    indicator.setColor(isBalancing ? 0x0000FF : 0x00FF00);
  }  
}

void setIMUWarmUpElapsed() {
  static bool invoked = false;
  if (!invoked) {
    invoked = true;
    indicator.setColor(0x00FF00);
    #ifdef LOG_ENABLED
    Serial.println("IMU Warm Up timeout elapsed");
    #endif
  }
}

void updateControl(unsigned long nowMicros) {
  static unsigned long timestamp = micros();
  if (nowMicros - timestamp < 1000 /* 1kHz*/) {
    return;
  }

  if (!readIMU()) {
    return;
  }

  /* Wait until IMU filter will settle */
  if (nowMicros < WARMUP_DELAY_US) {    
    return;
  } 
  setIMUWarmUpElapsed();

  angle = normalizeAngle(roll);

  float dt = ((float) (nowMicros - timestamp)) * 1e-6;

  if (abs(angle - targetAngle) < PI / 18) {
    setBalancing(true);
  }

  if (abs(angle - targetAngle) > PI / 4) {
    setBalancing(false);
    accel = 0.0;
    velocity = 0.0;    
  }

  if (!isBalancing) {
    return;
  }
  targetAngle = -velocityPID.getControl(velocity, dt);
  anglePID.setTarget(targetAngle);

  accel = anglePID.getControl(angle, dt);
  accel = constrain(accel, -MAX_ACCEL, MAX_ACCEL);

  timestamp = nowMicros;
}

void log(unsigned long nowMicros) {  
  static unsigned long timestamp = micros();  
  if (nowMicros - timestamp < 100000 /* 10Hz */) {
    return;
  }
  Serial.print("roll:");
  Serial.print(roll, 2);
  Serial.print("\t");

  Serial.print("a0:");
  Serial.print(targetAngle * RAD_TO_DEG, 4);
  Serial.print("\ta:");
  Serial.print(angle * RAD_TO_DEG, 4);
  Serial.print("\tv:");
  Serial.print(velocity, 4);
  Serial.print("\tu:");
  Serial.println(accel, 4);  
  timestamp = nowMicros;   
}