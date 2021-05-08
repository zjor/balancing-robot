class PID:
    def __init__(self, k_p, k_d, k_i, target, init_value=0.0):
        self.Kp = k_p
        self.Kd = k_d
        self.Ki = k_i
        self.target = target
        self.lastValue = init_value
        self.integral_error = 0.0

    def get_control(self, value, dt):
        """
            Returns PID control.
            Derivative spike is mitigated by dError/dt = -dInput/dt
            see: http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-derivative-kick/
        """
        error = self.target - value

        derivative = -(value - self.lastValue) / dt

        self.lastValue = value
        self.integral_error += error * dt
        return self.Kp * error + self.Kd * derivative + self.Ki * self.integral_error

    def set_target(self, target):
        self.target = target
