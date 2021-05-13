import RPi.GPIO as GPIO
from math import pi, degrees, radians

from sensor.mpu import MPU
from motor.stepper import Stepper
from util.timed_task import TimedTask

pitch_bias = radians(84.3)

if __name__ == "__main__":
    mpu = MPU()
    stepper = Stepper(dir_pin=5, step_pin=6, ppr=400)


    def read_mpu_task_handler(now, dt):
        roll, pitch = mpu.get_roll_pitch()
        pitch -= pitch_bias
        print(f"{degrees(pitch):6.2f}", end='\r')
        stepper.set_velocity(pitch * 8)


    read_mpu_task = TimedTask(delay=0.1, run=read_mpu_task_handler)

    try:
        while True:
            stepper.loop()
            read_mpu_task.loop()
    except KeyboardInterrupt:
        pass

    GPIO.cleanup()
