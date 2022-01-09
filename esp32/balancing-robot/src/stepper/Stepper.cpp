#include <Arduino.h>

#include "Stepper.h"

Stepper::Stepper(int enablePin, int dirPin, int stepPin) {
  this->enablePin = enablePin;
  this->dirPin = dirPin;
  this->stepPin = stepPin;
}

void Stepper::init() {
  pinMode(enablePin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  digitalWrite(stepPin, LOW);
}

void Stepper::setEnabled(bool enabled) {
  digitalWrite(enablePin, enabled ? LOW : HIGH);
}

void Stepper::setVelocity(float velocity) {

}
