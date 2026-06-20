from __future__ import annotations

from dataclasses import dataclass


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass
class PID:

    kp: float
    ki: float = 0.0
    kd: float = 0.0
    output_limits: tuple[float, float] = (-1.0, 1.0)
    integral_limits: tuple[float, float] = (-100.0, 100.0)
    integral_decay: float = 0.0

    integral: float = 0.0
    previous_error: float | None = None
    previous_output: float = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = None
        self.previous_output = 0.0

    def update(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return self.previous_output

        derivative = 0.0
        if self.previous_error is not None:
            derivative = (error - self.previous_error) / dt

        candidate_integral = clamp(
            self.integral*self.integral_decay + error * dt, # утечка интеграла против wind-up
            self.integral_limits[0],
            self.integral_limits[1],
        )
        raw_output = (
            self.kp * error
            + self.ki * candidate_integral
            + self.kd * derivative
        )

        low, high = self.output_limits
        saturated_high_in_error_direction = raw_output > high and error > 0.0 #проверка на упор в лимиты
        saturated_low_in_error_direction = raw_output < low and error < 0.0
        if not (
            saturated_high_in_error_direction
            or saturated_low_in_error_direction
        ):
            self.integral = candidate_integral

        output = clamp(
            self.kp * error + self.ki * self.integral + self.kd * derivative,
            low,
            high,
        )
        self.previous_error = error
        self.previous_output = output
        return output
