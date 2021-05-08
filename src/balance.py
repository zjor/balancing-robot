import RPi.GPIO as GPIO
from math import pi, degrees, radians

from sensor.mpu import MPU
from motor.stepper import Stepper
from util import timed_task, pid

PITCH_BIAS = radians(84.3)


class BalancingRobot:
    def __init__(self, left_motor: Stepper, right_motor: Stepper, mpu: MPU, angle_bias):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.mpu = mpu
        self.angle_bias = angle_bias
        self.angle = 0.0
        self.velocity = 0.0
        self.pid = pid.PID(k_p=1.0, k_d=0.0, k_i=0.0, target=0.0, init_value=0.0)
        self.sensor_read_task = timed_task.TimedTask(delay=0.01, run=self.update_angle_handler)
        self.control_loop_task = timed_task.TimedTask(delay=0.05, run=self.control_loop_handler)

    def update_angle_handler(self, now, dt):
        _, pitch = self.mpu.get_roll_pitch()
        self.angle = pitch - self.angle_bias

    def control_loop_handler(self, now, dt):
        u = self.pid.get_control(self.angle, dt)
        self.velocity += u * dt
        self.left_motor.set_velocity(self.velocity)
        self.right_motor.set_velocity(-self.velocity)

    def loop(self):
        self.sensor_read_task.loop()
        self.control_loop_task.loop()
        self.left_motor.loop()
        self.right_motor.loop()


if __name__ == "__main__":
    mpu = MPU()
    left_motor = Stepper(dir_pin=5, step_pin=6, ppr=400)
    right_motor = Stepper(dir_pin=20, step_pin=21, ppr=400)

    robot = BalancingRobot(
        left_motor=left_motor,
        right_motor=right_motor,
        mpu=mpu,
        angle_bias=PITCH_BIAS)

    try:
        while True:
            robot.loop()
    except KeyboardInterrupt:
        print("Interrupted")
        pass

    GPIO.cleanup()
