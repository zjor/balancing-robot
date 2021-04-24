import time
import RPi.GPIO as GPIO

DIR_PIN = 5
STEP_PIN = 6

def step(step_pin):
    GPIO.output(step_pin, GPIO.HIGH)
    time.sleep(1e-3)
    GPIO.output(step_pin, GPIO.LOW)


if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(DIR_PIN, GPIO.OUT)
    GPIO.setup(STEP_PIN, GPIO.OUT)


    GPIO.output(DIR_PIN, GPIO.HIGH)
    while True:
        step(STEP_PIN)
        time.sleep(0.05)


