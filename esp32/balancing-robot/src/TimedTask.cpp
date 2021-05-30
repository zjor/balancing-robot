#include <Arduino.h>

#include "TimedTask.h"

TimedTask::TimedTask(t_func func, unsigned long delta_us) {
  _delta_us = delta_us;
  _func = func;
  _last_us = micros();
}

void TimedTask::loop() {
  unsigned long now = micros();
  unsigned long dt = now - _last_us;

  if (dt >= _delta_us) {
    _func(now, dt);
    _last_us = now;
  }
}