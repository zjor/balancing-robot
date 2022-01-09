#ifndef STEPPER_H
#define STEPPER_H

#include <stdint.h>

class Stepper {
  public:
    Stepper(int enablePin, int dirPin, int stepPin);
    void init();

    /**
     * @brief Sets velocity in radians per second.
     * 
     * @param velocity 
     */
    void setVelocity(float velocity);

    void setEnabled(bool enabled);

    /**
     * @brief Handles timer interrupt, increases ticks counter or performs a step.
     * 
     */
    void tick();

  private:
    int enablePin;
    int dirPin;
    int stepPin;    
    volatile uint32_t currentTick;
    volatile uint32_t ticksPerPulse;
};

#endif 