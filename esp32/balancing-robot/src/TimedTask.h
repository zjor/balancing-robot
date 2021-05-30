#ifndef TIMED_TASK_H
#define TIMED_TASK_H

/**
 * Function receiving the following arguments:
 * - now - time in micros
 * - dt - elapsed micros from the prevoius invocation
 */
typedef void (* t_func)(unsigned long, unsigned long);

class TimedTask {
  public:
    TimedTask(t_func func, unsigned long delta_us);
    void loop();

  private:
    unsigned long _last_us;
    unsigned long _delta_us;
    t_func _func;
};

#endif