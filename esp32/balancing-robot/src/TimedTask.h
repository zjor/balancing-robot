#ifndef TIMED_TASK_H
#define TIMED_TASK_H

/**
 * Function receiving the following arguments:
 * - now - time in millis
 * - dt - elapsed millis from the prevoius invocation
 */
typedef void (* t_func)(unsigned long, unsigned long);

class TimedTask {
  public:
    TimedTask(t_func func, unsigned long delta_millis);    
    void loop();

  private:
    unsigned long _last_millis;
    unsigned long _delta_millis;
    t_func _func;
};

#endif