#ifndef STEPPER_H
#define STEPPER_H

#include <stdint.h>

class Stepper {
  public:

    /**
     * @brief Construct a new Stepper object
     * 
     * @param enablePin LOW - enabled
     * @param dirPin 
     * @param stepPin 
     * @param ticksPerSecond - timer interrupt ticks count per second
     * @param pulsesPerRevolution - stepper pulses count per revolution
     * @param pulseWidth - stepper pulse width in ticks count
     */
    Stepper(int enablePin, int dirPin, int stepPin, uint32_t ticksPerSecond, uint32_t pulsesPerRevolution, uint32_t pulseWidth);
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
    uint32_t ticksPerSecond;
    uint32_t pulsesPerRevolution;
    uint32_t pulseWidth;
    volatile uint32_t currentTick;
    volatile uint32_t ticksPerPulse;
};

#endif 