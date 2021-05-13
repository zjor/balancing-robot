#include <Arduino.h>

#include "TimedTask.h"

TimedTask::TimedTask(t_func func, unsigned long delta_millis) {
  _delta_millis = delta_millis;
  _func = func;
  _last_millis = millis();
}

void TimedTask::loop() {
  unsigned long now = millis();
  unsigned long dt = now - _last_millis;

  if (dt >= _delta_millis) {
    _func(now, dt);
    _last_millis = now;
  }
}