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

#define PIN_MPU_SCL 22
#define PIN_MPU_SDA 21
#define PIN_MPU_INT 19

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

PID angle_pid(ANGLE_Kp, ANGLE_Kd, ANGLE_Ki, ANGLE_SET_POINT);
PID velocity_pid(0.00, 0.0, 0.0, 0.0);
bool is_balancing = false;

hw_timer_t * timer = NULL;
volatile uint32_t ticksPerPulse = TICKS_PER_SECOND;
volatile uint32_t currentTick = 0;

int getTicksPerPulse(float velocity) {
  if (abs(velocity) < 1e-3) {
    return UINT32_MAX;
  } else {
    return (uint32_t)(2.0 * PI * TICKS_PER_SECOND / (abs(velocity) * PPR)) - PULSE_WIDTH;
  }  
}


void IRAM_ATTR onTimer() {
  if (currentTick >= ticksPerPulse) {
    currentTick = 0;
  }
  if (currentTick == 0) {
  } else if (currentTick == PULSE_WIDTH) {    
  }
  currentTick++; 
}






void setup(void) {
  setCpuFrequencyMhz(240);

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(1000000UL);



  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 5, true);
  timerAlarmEnable(timer);  

}



void loop() {

}