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

#define INTERRUPT_PIN 2

#define PPR   1600
#define TICKS_PER_SECOND  40000 // 40kHz
#define PULSE_WIDTH 1

#define MAX_ACCEL (200)
#define ANGLE_Kp  450.0
#define ANGLE_Kd  20.0
#define ANGLE_Ki  0.0

#define VELOCITY_Kp  0.007
#define VELOCITY_Kd  0.0
#define VELOCITY_Ki  0.0005

#define WARMUP_DELAY_US (5000000UL)

#define ANGLE_SET_POINT (2.0 * DEG_TO_RAD)

#define OUTPUT_READABLE_YAWPITCHROLL
// #define COUNT_LOOP
// #define LOGGING_ENABLED

#define NANO_BLE

/* BLE communication-related params */
#define MAX_PACKET_SIZE 96
#define DIVISOR 10000.0

char packet[MAX_PACKET_SIZE];
uint8_t packet_size = 0;

MPU6050 mpu;
bool dmpReady = false;
uint8_t mpuIntStatus;  
uint8_t fifoBuffer[64];

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

PID anglePID(ANGLE_Kp, ANGLE_Kd, ANGLE_Ki, ANGLE_SET_POINT);
PID velocityPID(VELOCITY_Kp, VELOCITY_Kd, VELOCITY_Ki, 0.0);

float pid_settings[6] = {
  ANGLE_Kp, ANGLE_Kd, ANGLE_Ki,
  VELOCITY_Kp, VELOCITY_Kd, VELOCITY_Ki
};

volatile unsigned long currentTick = 0UL;
volatile unsigned long ticksPerPulse = UINT64_MAX;
volatile float accel = 0.0;
volatile float velocity = 0.0;

bool isBalancing = false;

float angle = 0.0;
float targetAngle = ANGLE_SET_POINT;

float targetVelocity = 0.0;

unsigned long lastUpdateMicros = 0;

void send_float_array(float *a, uint8_t size);
void parse_float_array(char *p, uint8_t p_size, float *dest);
void parse_settings(char *p, uint8_t p_size);
void handle_packet(char *p, uint8_t p_size);

void initMPU() {
  const int16_t accel_offset[3] = { -1262, -307, 1897 };
  const int16_t gyro_offset[3] = { 23, -41, 49 };

  Wire.begin();
  Wire.setClock(1000000UL);
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

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
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
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

bool mpuUpdate() {
  if (mpuInterrupt && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpuInterrupt = false;
    return true;
  }
  return false;
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
  /* Wait until IMU filter will settle */
  if (nowMicros < WARMUP_DELAY_US) {
    return;
  }

  static unsigned long timestamp = micros();
  if (nowMicros - timestamp < 1000 /* 1kHz */) {
    return;
  }
  if (!mpuUpdate()) {
    return;
  }
  angle = ypr[1];

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
  #ifdef NANO_BLE
    Serial.print("AT+BAUD=4\r\n");
  #endif
  initMPU();
  setTimers();
  initMotors();
}

void loop() {  
  unsigned long now = micros();
  updateVelocity(now);
  updateControl(now);
  #ifdef LOGGING_ENABLED
    log(now);
  #endif

  #ifdef COUNT_LOOP
    static unsigned long last_ts = micros();
    static unsigned long  counter = 0;

    counter++;
    if (now - last_ts >= 1000000) {
      Serial.println(counter);
      counter = 0;
      last_ts = now;
    }
  #endif

  while (Serial.available()) {
    int c = Serial.read();
    if (c == '\n') {
        continue;
    } else if (c == '\r') {
        handle_packet(packet, packet_size);
        packet_size = 0;
    } else {
        packet[packet_size++] = (uint8_t) c;
    }
  }
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

void send_float_array(float *a, uint8_t size) {
    for (int i = 0; i < size; i++) {
        Serial.print((long)(a[i] * DIVISOR));
        if (i < size - 1) {
            Serial.print(';');
        }
    }
    Serial.print("\r\n");
}

void parse_float_array(char *p, uint8_t p_size, float *dest) {
    char buf[16];
    long value;
    uint8_t buf_size = 0;
    uint8_t index = 0;
    for (uint8_t i = 0; i < p_size; i++) {
        if ((p[i] >= '0' && p[i] <= '9') || p[i] == '+' || p[i] == '-') {
            buf[buf_size++] = p[i];
        } else if (p[i] == ';') {
            buf[buf_size] = '\0';
            buf_size = 0;
            value = atol(buf);
            dest[index++] = ((float)value) / DIVISOR;
        }
    }
    buf[buf_size] = '\0';
    value = atol(buf);
    dest[index] = ((float)value) / DIVISOR;
}

void parse_settings(char *p, uint8_t p_size) {
    parse_float_array(p, p_size, pid_settings);
    anglePID.setSettings(pid_settings[0], pid_settings[1], pid_settings[2]);
    velocityPID.setSettings(pid_settings[3], pid_settings[4], pid_settings[5]);
}

void handle_packet(char *p, uint8_t p_size) {
    switch (p[0]) {
        case 'r':
            send_float_array(pid_settings, 6);
            break;
        case 's':
            parse_settings(&p[1], p_size - 1);
            send_float_array(pid_settings, 6);
            break;
        // case 'c':
        //     parse_control(&p[1], p_size - 1);
        //     send_float_array(control, 2);
        //     break;
    }
}