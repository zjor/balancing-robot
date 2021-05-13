#include <Arduino.h>
#include <Wire.h>

#include "MPU9250.h"
#include "TimedTask.h"

#define RIGHT_MOTOR_STEP_PIN  15
#define RIGHT_MOTOR_DIR_PIN   32
#define LEFT_MOTOR_STEP_PIN 17
#define LEFT_MOTOR_DIR_PIN 21

#define PPR       400

#define TICKS_PER_SECOND 200000

#define SET_POINT 92.5

MPU9250 mpu;

void setCalibrationValues() {
  mpu.setAccBias(39.81, 23.91, 85.44);
  mpu.setGyroBias(-1.72, 2.98, 1.44);
  mpu.setMagBias(426.18, 13.96, 457.22);
  mpu.setMagScale(1.06, 0.72, 1.52);
}

hw_timer_t * timer = NULL;
volatile uint32_t ticksPerPulse = TICKS_PER_SECOND;
volatile uint32_t currentTick = 0;
float angle;
float velocity;

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

void log(unsigned long now, unsigned long dt) {
  Serial.print(angle, 4);
  Serial.print("\t");
  Serial.println(velocity);
}

TimedTask loggerTask(log, 50);

void control(unsigned long now, unsigned long dt) {
  if (mpu.update()) {
      angle = mpu.getRoll();
      if (abs(angle - SET_POINT) < 15.0) {
        float u = radians(angle - SET_POINT) * dt * 0.1;
        velocity += u;
      } else {
        velocity = 0.0;
      }
      setVelocity(velocity);
  }
}

TimedTask controlTask(control, 10);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  if (!mpu.setup(0x68)) {
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(1000);
    }
  }
  setCalibrationValues();
  pinMode(LEFT_MOTOR_STEP_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_STEP_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);

  digitalWrite(LEFT_MOTOR_STEP_PIN, LOW);
  digitalWrite(LEFT_MOTOR_DIR_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_STEP_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_DIR_PIN, LOW);

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 5, true);
  timerAlarmEnable(timer);  

  setVelocity(0);
}

void loop() {  
  controlTask.loop();
  loggerTask.loop();
}

