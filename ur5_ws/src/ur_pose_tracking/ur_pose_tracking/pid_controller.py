import math

class PIDController:
    def __init__(self, kp, ki, kd, output_limit, deadband=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.deadband = deadband

        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def update(self, error, current_time_nanosec):
        if self.last_time is None:
            self.last_time = current_time_nanosec
            return 0.0

        if abs(error) < self.deadband:
            self.prev_error = error
            return 0.0

        dt = (current_time_nanosec - self.last_time) / 1e9
        if dt <= 0.0 or dt > 0.1:
            self.last_time = current_time_nanosec
            return 0.0

        # P term
        p_term = self.kp * error

        # I term
        self.integral += error * dt
        max_i_output = self.output_limit * 0.5
        i_term = self.ki * self.integral
        i_term = max(min(i_term, max_i_output), -max_i_output)

        # D term
        d_term = 0.0
        if dt > 0:
            derivative = (error - self.prev_error) / dt
            d_term = self.kd * derivative

        # Total output
        output = p_term + i_term + d_term

        # Clamp output
        output = max(min(output, self.output_limit), -self.output_limit)

        # Update state
        self.prev_error = error
        self.last_time = current_time_nanosec

        return output

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None
