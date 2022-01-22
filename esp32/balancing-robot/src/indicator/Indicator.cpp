#include <Arduino.h>
#include "Indicator.h"

Indicator::Indicator(int redPin, int greenPin, int bluePin) {
  this->_redPin = redPin;
  this->_greenPin = greenPin;
  this->_bluePin = bluePin;
}

void Indicator::init() {
  ledcAttachPin(_redPin, R_CH);
  ledcAttachPin(_greenPin, G_CH);
  ledcAttachPin(_bluePin, B_CH);

  ledcSetup(R_CH, 1000, 8);
  ledcSetup(G_CH, 1000, 8);
  ledcSetup(B_CH, 1000, 8);  
}

void Indicator::setColor(int color) {
  int red = (color & 0xFF0000) >> 16;
  int green = (color & 0x00FF00) >> 8;
  int blue = (color & 0x0000FF);

  ledcWrite(R_CH, 255 - red);
  ledcWrite(G_CH, 255 - green);
  ledcWrite(B_CH, 255 - blue);
}