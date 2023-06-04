"""A small, dependency-free PID controller."""
import math


def normalize_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value, low, high):
    return max(low, min(high, value))


class PID:

    def __init__(self, kp, ki=0.0, kd=0.0, output_limit=float('inf'), integral_limit=float('inf')):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_error = None

    def update(self, error, dt):
        if dt <= 0.0:
            raise ValueError('dt must be positive')
        self.integral = clamp(self.integral + error * dt, -self.integral_limit, self.integral_limit)
        derivative = 0.0 if self.prev_error is None else (error - self.prev_error) / dt
        self.prev_error = error
        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        return clamp(out, -self.output_limit, self.output_limit)
