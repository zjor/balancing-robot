#include <Wire.h>
#include <Arduino.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

const float accOffsets[] = { 0.02, 0.02, -0.11 };
const float gyroOffsets[] = { -0.64, 1.18, -1.40 };

void initMPU() {
  Wire.begin();
  Wire.setClock(1000000UL);
  byte status = mpu.begin(0, 0);
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

void setup() {

  Serial.begin(115200);
  initMPU();  

}

unsigned long lastUpdateMillis = 0;

void loop() {
  mpu.update();
  unsigned long now = millis();
  if (now - lastUpdateMillis > 10) {
    Serial.println(mpu.getAngleY());
    lastUpdateMillis = now;
  }
}