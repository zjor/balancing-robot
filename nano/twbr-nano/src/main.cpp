#include <Wire.h>
#include <Arduino.h>
#include <MPU6050_light.h>

#include "pid/PID.h"

#define MOT_A_STEP  9
#define MOT_A_DIR   8
#define MOT_B_STEP  7
#define MOT_B_DIR   6

#define PPR   1600
#define TICKS_PER_SECOND  100000 // 100kHz
#define PULSE_WIDTH 1

#define MAX_ACCEL (200)
#define ANGLE_Kp  350.0
#define ANGLE_Kd  15.0
#define ANGLE_Ki  0.0

#define ANGLE_SET_POINT 0.0

MPU6050 mpu(Wire);
PID anglePID(ANGLE_Kp, ANGLE_Kd, ANGLE_Ki, ANGLE_SET_POINT);

const float accOffsets[] = { 0.02, 0.02, -0.11 };
const float gyroOffsets[] = { -0.64, 1.18, -1.40 };

volatile unsigned long currentTick = 0UL;
volatile unsigned long ticksPerPulse = UINT64_MAX;
volatile float accel = 0.0;
volatile float velocity = 0.0;

bool isBalancing = false;

float angle = 0.0;
float targetAngle = 0.0;

unsigned long lastUpdateMicros = 0;



void initMPU() {
  Wire.begin();
  Wire.setClock(1000000UL);
  byte status = mpu.begin(0, 0);
  mpu.setFilterAccCoef(0.01);
  if (status != 0) {
    Serial.println("MPU init failed");
    while (1) { delay(100); }
  }

  // mpu.calcOffsets();
  // Serial.println(mpu.getAccXoffset());
  // Serial.println(mpu.getAccYoffset());
  // Serial.println(mpu.getAccZoffset());

  // Serial.println(mpu.getGyroXoffset());
  // Serial.println(mpu.getGyroYoffset());
  // Serial.println(mpu.getGyroZoffset());
  mpu.setAccOffsets(accOffsets[0], accOffsets[1], accOffsets[2]);
  mpu.setGyroOffsets(gyroOffsets[0], gyroOffsets[1], gyroOffsets[2]);
}

void setTimer1(int ocra) {  
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; // initialize counter value to 0  
  OCR1A = ocra;
  TCCR1B |= (1 << WGM12); // turn on CTC mode
  TCCR1B |= (1 << CS11);  // set prescaler to 8  
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
}

void setTimer2(int ocra) {
  TCCR2A = 0; // set entire TCCR1A register to 0
  TCCR2B = 0; // same for TCCR1B
  TCNT2  = 0; // initialize counter value to 0  
  OCR2A = ocra;
  TCCR2A |= (1 << WGM21); // turn on CTC mode
  TCCR2B |= (1 << CS22);  // set prescaler to 64  
  TIMSK2 |= (1 << OCIE2A);  // enable timer compare interrupt
}

void setTimers() {
  cli();
  setTimer1(19); // 100kHz
  setTimer2(49); //5kHz
  sei();
}

void initMotors() {
  pinMode(MOT_A_DIR, OUTPUT);
  pinMode(MOT_A_STEP, OUTPUT);
  pinMode(MOT_B_DIR, OUTPUT);
  pinMode(MOT_B_STEP, OUTPUT);
  digitalWrite(MOT_A_STEP, LOW);
  digitalWrite(MOT_B_STEP, LOW);
}

void control(unsigned long dt_us) {
  float dt = ((float) dt_us) * 1e-6;

  if (abs(angle - targetAngle) < PI / 18) {
    isBalancing = true;
  }

  if (abs(angle - targetAngle) > PI / 4) {
    isBalancing = false;
    accel = 0.0;
    velocity = 0.0;
  }

  if (!isBalancing) {
    return;
  }
  // angle_target = velocity_pid.getControl(-velocity, dt);
  // angle_pid.setTarget(angle_target);

  accel = anglePID.getControl(angle, dt);
  accel = constrain(accel, -MAX_ACCEL, MAX_ACCEL);    
}

void log() {
  Serial.print("t:");
  Serial.print(millis());
  Serial.print("a:");
  Serial.print(angle * RAD_TO_DEG, 4);
  Serial.print("\tv:");
  Serial.print(velocity, 4);
  Serial.print("\tu:");
  Serial.println(accel, 4);
}

void setup() {
  Serial.begin(115200);
  initMPU();
  setTimers();
  initMotors();
}


void loop() {
  mpu.update();
  unsigned long now = micros();
  if (now - lastUpdateMicros > 1000) {
    angle = -mpu.getAngleY() * DEG_TO_RAD;
    control(now - lastUpdateMicros);
    lastUpdateMicros = now;
  }
  // log();
}

/**
 * Stepper control interrupt handler
 */
ISR(TIMER1_COMPA_vect) {
  if (currentTick >= ticksPerPulse) {
    currentTick = 0;
  }

  if (currentTick == 0) {
    digitalWrite(MOT_A_STEP, HIGH);
    digitalWrite(MOT_B_STEP, HIGH);
  } else if (currentTick == PULSE_WIDTH) {
    digitalWrite(MOT_A_STEP, LOW);
    digitalWrite(MOT_B_STEP, LOW);
  }
  
  currentTick++;
}

/**
 * Velocity update interrupt handler
 */
ISR(TIMER2_COMPA_vect) {
  velocity += accel * 2e-4;
  if (abs(velocity) < 1e-3) {
    ticksPerPulse = UINT64_MAX;
  } else {
    ticksPerPulse = (uint64_t)(2.0 * PI * TICKS_PER_SECOND / (abs(velocity) * PPR)) - PULSE_WIDTH;
  }
  digitalWrite(MOT_A_DIR, velocity > 0 ? HIGH : LOW);
  digitalWrite(MOT_B_DIR, -velocity > 0 ? HIGH : LOW);
}