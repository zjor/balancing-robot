#include <Arduino.h>

#include "Stepper.h"

int getTicksPerPulse(float velocity, int32_t ticksPerSecond, uint32_t pulsesPerRevolution, uint32_t pulseWidth) {
  if (abs(velocity) < 1e-3) {
    return UINT32_MAX;
  } else {
    return (uint32_t)(TWO_PI * ticksPerSecond / (abs(velocity) * pulsesPerRevolution)) - pulseWidth;
  }  
}

Stepper::Stepper(int enablePin, int dirPin, int stepPin, uint32_t ticksPerSecond, uint32_t pulsesPerRevolution, uint32_t pulseWidth) {
  this->enablePin = enablePin;
  this->dirPin = dirPin;
  this->stepPin = stepPin;
  this->ticksPerSecond = ticksPerSecond;
  this->pulsesPerRevolution = pulsesPerRevolution;
  this->pulseWidth = pulseWidth;  
}

void Stepper::init() {
  pinMode(enablePin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  digitalWrite(stepPin, LOW);
  
  this->currentTick = 0;
}

void Stepper::setEnabled(bool enabled) {
  digitalWrite(enablePin, enabled ? LOW : HIGH);
}

void Stepper::setVelocity(float velocity) {
  ticksPerPulse = getTicksPerPulse(velocity, ticksPerSecond, pulsesPerRevolution, pulseWidth);
  digitalWrite(dirPin, velocity > 0 ? HIGH : LOW);
}

void Stepper::tick() {
  if (currentTick >= ticksPerPulse) {
    currentTick = 0;
  }
  if (currentTick == 0) {
    digitalWrite(stepPin, HIGH);
  } else if (currentTick == pulseWidth) {    
    digitalWrite(stepPin, LOW);
  }
  currentTick++; 
}
