#include <Wire.h>
#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include "pid/PID.h"

// PORTB, bit 1, PB1
#define MOT_A_STEP  9
#define MOT_A_DIR   8

// PORTD, bit 7, PD7
#define MOT_B_STEP  7
#define MOT_B_DIR   6

#define PPR   1600
#define TICKS_PER_SECOND  40000 // 40kHz
#define PULSE_WIDTH 1

#define MAX_ACCEL (200)
#define ANGLE_Kp  450.0
#define ANGLE_Kd  20.0
#define ANGLE_Ki  0.0

#define VELOCITY_Kp  0.001
#define VELOCITY_Kd  0.0
#define VELOCITY_Ki  0.007

#define ANGLE_SET_POINT (2.0 * DEG_TO_RAD)

#define OUTPUT_READABLE_YAWPITCHROLL

MPU6050 mpu;
bool dmpReady = false;  
uint8_t fifoBuffer[64];

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

PID anglePID(ANGLE_Kp, ANGLE_Kd, ANGLE_Ki, ANGLE_SET_POINT);
PID velocityPID(VELOCITY_Kp, VELOCITY_Kd, VELOCITY_Ki, 0.0);

const float accOffsets[] = { 0.02, 0.02, -0.11 };
const float gyroOffsets[] = { -0.64, 1.18, -1.40 };

volatile unsigned long currentTick = 0UL;
volatile unsigned long ticksPerPulse = UINT64_MAX;
volatile float accel = 0.0;
volatile float velocity = 0.0;

bool isBalancing = false;

float angle = 0.0;
float targetAngle = ANGLE_SET_POINT;

unsigned long lastUpdateMicros = 0;

void initMPU() {
  const int16_t accel_offset[3] = { -1262, -307, 1897 };
  const int16_t gyro_offset[3] = { 23, -41, 49 };

  Wire.begin();
  Wire.setClock(1000000UL);
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println(F("MPU6050 connection failed"));
    while(1) {}
  }

  mpu.dmpInitialize();

  mpu.setXGyroOffset(gyro_offset[0]);
  mpu.setYGyroOffset(gyro_offset[1]);
  mpu.setZGyroOffset(gyro_offset[2]);
  mpu.setXAccelOffset(accel_offset[0]);
  mpu.setYAccelOffset(accel_offset[1]);
  mpu.setZAccelOffset(accel_offset[2]);

  mpu.setDMPEnabled(true);
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

void setTimers() {
  cli();
  setTimer1(49); // 40kHz
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

void log(unsigned long nowMicros) {
  static unsigned long timestamp = micros();  
  if (nowMicros - timestamp < 10000 /* 100Hz */) {
    return;
  }
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

void mpuUpdate() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
}

void updateVelocity(unsigned long nowMicros) {
  // static unsigned long counter = 0;
  // static unsigned long sum = 0;

  static unsigned long timestamp = micros();
  if (nowMicros - timestamp < 100 /* 10kHz */) {
    return;
  }

  // sum += (nowMicros - timestamp);
  // counter++;
  // if (counter >= 1000) {
  //   Serial.println(((float)(sum)) / counter);
  //   counter = 0;
  //   sum = 0;
  // }

  float dt = ((float) (nowMicros - timestamp)) * 1e-6;
  velocity += accel * dt;
  if (abs(velocity) < 1e-3) {
    ticksPerPulse = UINT64_MAX;
  } else {
    ticksPerPulse = (uint64_t)(2.0 * PI * TICKS_PER_SECOND / (abs(velocity) * PPR)) - PULSE_WIDTH;
  }
  if (velocity > 0) {
    PORTB |= _BV(PB0);  // digitalWrite(MOT_A_DIR, HIGH);
    PORTD &= ~_BV(PD6); // digitalWrite(MOT_B_DIR, LOW);  
  } else {
    PORTB &= ~_BV(PB0);  // digitalWrite(MOT_A_DIR, LOW);
    PORTD |= _BV(PD6);  // digitalWrite(MOT_B_DIR, HIGH);  
  }

  timestamp = nowMicros;
}

void updateControl(unsigned long nowMicros) {
  static unsigned long timestamp = micros();
  if (nowMicros - timestamp < 1000 /* 1kHz */) {
    return;
  }
  mpuUpdate();
  angle = -ypr[1];

  float dt = ((float) (nowMicros - timestamp)) * 1e-6;

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
  targetAngle = -velocityPID.getControl(velocity, dt);
  anglePID.setTarget(targetAngle);

  accel = anglePID.getControl(angle, dt);
  accel = constrain(accel, -MAX_ACCEL, MAX_ACCEL);

  timestamp = nowMicros;
}

void setup() {
  Serial.begin(115200);
  initMPU();
  setTimers();
  initMotors();
}

void loop() {  
  unsigned long now = micros();
  updateVelocity(now);
  updateControl(now);
  // log(now);
}

/**
 * Stepper control interrupt handler
 */
ISR(TIMER1_COMPA_vect) {
  if (currentTick >= ticksPerPulse) {
    currentTick = 0;
  }

  if (currentTick == 0) {
    PORTD |= _BV(PD7); // digitalWrite(MOT_B_STEP, HIGH);
    PORTB |= _BV(PB1); // digitalWrite(MOT_A_STEP, HIGH);
  } else if (currentTick == PULSE_WIDTH) {
    PORTD &= ~_BV(PD7); // digitalWrite(MOT_B_STEP, LOW);
    PORTB &= ~_BV(PB1); // digitalWrite(MOT_A_STEP, LOW);    
  }
  
  currentTick++;
}