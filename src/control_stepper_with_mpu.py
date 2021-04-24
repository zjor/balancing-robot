import time
import RPi.GPIO as GPIO
from math import pi

from mpu import MPU
from stepper import Stepper

if __name__ == "__main__":
    mpu = MPU()
    stepper = Stepper(dir_pin=5, step_pin=6, steps_per_revolution=400)
    stepper.set_velocity(10 * pi)


    try:
        while True:
            roll, pitch = mpu.get_roll_pitch()
            print(f"{pitch:6.2f}", end='\r')
            
            target = pitch * stepper.steps_per_revolution / 2.0 / pi
            if stepper.position > target:
                stepper.set_direction(Stepper.CCW)
            else:
                stepper.set_direction(Stepper.CW)

            for i in range(0, round(abs(target - stepper.position))):
                stepper.step()
                time.sleep(stepper.step_delay)
    except KeyboardInterrupt:
        pass
        
    GPIO.cleanup()
