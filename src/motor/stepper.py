import time
import RPi.GPIO as GPIO
from math import pi


class Stepper:
    MIN_VELOCITY_THRESHOLD = 1e-6
    MAX_STEP_DELAY = 1e6
    STEP_PULSE_WIDTH = 1e-6

    CW = GPIO.HIGH
    CCW = GPIO.LOW

    def __init__(self, dir_pin, step_pin, ppr=400):
        self.dir_pin = dir_pin
        self.step_pin = step_pin
        self.ppr = ppr

        self.position = 0
        self.dx = 1
        self.step_delay = 0
        self.last_step_ts = .0

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
        time.sleep(Stepper.STEP_PULSE_WIDTH)
        GPIO.output(self.step_pin, GPIO.LOW)
        self.position += self.dx

    def set_velocity(self, velocity):
        if abs(velocity) < Stepper.MIN_VELOCITY_THRESHOLD:
            self.step_delay = Stepper.MAX_STEP_DELAY
        else:
            self.step_delay = (2.0 * pi / self.ppr / abs(velocity)) - Stepper.STEP_PULSE_WIDTH
            self.set_direction(Stepper.CW if velocity > 0 else Stepper.CCW)

    def loop(self):
        now = time.time()
        if now - self.last_step_ts >= self.step_delay:
            self.step()
            self.last_step_ts = now


if __name__ == "__main__":
    motor = Stepper(dir_pin=5, step_pin=6, ppr=400)

    motor.set_direction(Stepper.CW)
    motor.set_velocity(-6.0 * pi)

    try:
        while True:
            motor.loop()
    except KeyboardInterrupt:
        pass

    GPIO.cleanup()
