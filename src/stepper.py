import time
import RPi.GPIO as GPIO
from math import pi


class Stepper:
    CW = GPIO.HIGH
    CCW = GPIO.LOW

    def __init__(self, dir_pin, step_pin, steps_per_revolution=400):
        self.dir_pin = dir_pin
        self.step_pin = step_pin
        self.steps_per_revolution = steps_per_revolution        

        self.position = 0
        self.dx = 1
        self.step_delay = 0

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)
        

    def set_direction(self, direction):
        GPIO.output(self.dir_pin, direction)
        if direction == Stepper.CW:
            self.dx = 1
        else:
            self.dx = -1

    def step(self):
        GPIO.output(self.step_pin, GPIO.HIGH)
        time.sleep(1e-6)
        GPIO.output(self.step_pin, GPIO.LOW)
        self.position += self.dx

    def set_velocity(self, velocity):
        self.step_delay = 2.0 * pi / self.steps_per_revolution / velocity

