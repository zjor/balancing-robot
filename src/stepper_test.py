import time
import RPi.GPIO as GPIO
from math import pi

DIR_PIN = 5
STEP_PIN = 6

STEPS_PER_REV = 400


def get_delay(v):
    return 2.0 * pi / v / STEPS_PER_REV


def step(step_pin):
    GPIO.output(step_pin, GPIO.HIGH)
    time.sleep(1e-6)
    GPIO.output(step_pin, GPIO.LOW)


if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(DIR_PIN, GPIO.OUT)
    GPIO.setup(STEP_PIN, GPIO.OUT)


    GPIO.output(DIR_PIN, GPIO.HIGH)
    try:
        print('Running stepper...')
        d = get_delay(10.0 * pi)

        while True:
            step(STEP_PIN)
            time.sleep(d)

    except KeyboardInterrupt:
        print('Interrupted')
    GPIO.cleanup()


