import time
import RPi.GPIO as GPIO
from math import pi, sin

from motor import stepper
from util import timed_task

left_stepper = None
right_stepper = None


def handler(now, dt):
    global left_stepper, right_stepper
    velocity = sin(now) * 8 * pi
    left_stepper.set_velocity(velocity)


if __name__ == "__main__":
    left_stepper = stepper.Stepper(dir_pin=5, step_pin=6, ppr=400)
    right_stepper = stepper.Stepper(dir_pin=20, step_pin=21, ppr=400)

    left_stepper.set_velocity(pi)
    right_stepper.set_velocity(0)

    tt = timed_task.TimedTask(0.01, handler)

    try:
        while True:
            left_stepper.loop()
            right_stepper.loop()
            tt.loop()
    except KeyboardInterrupt:
        print('Interrupted')
    GPIO.cleanup()
