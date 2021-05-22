#include <Arduino.h>
#include <Wire.h>

#include "PID.h"
#include "MPU9250.h"
#include "TimedTask.h"

#define RIGHT_MOTOR_STEP_PIN  15
#define RIGHT_MOTOR_DIR_PIN   32
#define LEFT_MOTOR_STEP_PIN   17
#define LEFT_MOTOR_DIR_PIN    21

#define PPR       1600

#define TICKS_PER_SECOND 200000

#define SET_POINT 90.0

MPU9250 mpu;
PID pid(5.0, 10.0, 0.0, SET_POINT * DEG_TO_RAD);
bool is_balancing = false;

#define ANGLE_AVG 5
float angle_buf[ANGLE_AVG];

float getAngleAvg() {
  float s = 0.0;
  for (int i = 0; i < ANGLE_AVG; i++) {
    s += angle_buf[i];
  }
  return s / ANGLE_AVG;
}

void setCalibrationValues() {
  mpu.setAccBias(41.58, 22.70, 84.88);
  mpu.setGyroBias(-1.64, 3.06, 1.40);
  mpu.setMagBias(426.18, 13.96, 457.22);
  mpu.setMagScale(1.06, 0.72, 1.52);
}

void initMPU() {
  MPU9250Setting setting;
  setting.accel_fs_sel = ACCEL_FS_SEL::A2G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G250DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

  if (!mpu.setup(0x68, setting)) {
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(1000);
    }
  }
  setCalibrationValues();
  mpu.selectFilter(QuatFilterSel::MAHONY);
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

void control(unsigned long now_millis, unsigned long dt_millis) {
  float dt = dt_millis * 1e-3;
  angle = getAngleAvg();

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

void setup() {
  Serial.begin(115200);
  Wire.begin();

  initMPU();

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

int angle_i = 0;

void loop() {  
  controlTask.loop();
  loggerTask.loop();

  if (mpu.update()) {
    angle_buf[angle_i++] = mpu.getRoll();
    angle_i = angle_i % ANGLE_AVG;
  }
}

