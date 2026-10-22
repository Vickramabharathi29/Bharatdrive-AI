"""
PID Controller for Autonomous Vehicles.
BharatDrive-AI - MathWorks Autonomous Driving Hackathon.

Supports Longitudinal Speed Tracking and Lateral Path Tracking with Anti-Windup.
"""

from __future__ import annotations


class PIDController:
    """
    Proportional-Integral-Derivative Controller with Anti-Windup Clamping.
    """

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        output_min: float = -1.0,
        output_max: float = 1.0,
        ktheta: float = 0.0,
    ) -> None:
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.ktheta = float(ktheta)
        self.output_min = float(output_min)
        self.output_max = float(output_max)

        self.integral_error = 0.0
        self.prev_error = 0.0
        span = self.output_max - self.output_min
        self.anti_windup_limit = (span * 0.5) / max(1e-4, abs(self.ki)) if self.ki != 0 else 100.0

    def reset(self) -> None:
        self.integral_error = 0.0
        self.prev_error = 0.0

    def compute_longitudinal(self, target_v: float, current_v: float, dt: float = 0.01) -> float:
        """
        Calculates commanded longitudinal acceleration (m/s^2).
        """
        err = float(target_v - current_v)
        dt = max(1e-5, float(dt))

        self.integral_error += err * dt
        self.integral_error = max(-self.anti_windup_limit, min(self.anti_windup_limit, self.integral_error))

        deriv = (err - self.prev_error) / dt
        self.prev_error = err

        raw = self.kp * err + self.ki * self.integral_error + self.kd * deriv
        return max(self.output_min, min(self.output_max, raw))

    def compute_lateral(self, ey: float, etheta: float, dt: float = 0.01) -> float:
        """
        Calculates commanded front steering angle (rad).
        """
        dt = max(1e-5, float(dt))
        self.integral_error += float(ey) * dt
        self.integral_error = max(-self.anti_windup_limit, min(self.anti_windup_limit, self.integral_error))

        deriv = (float(ey) - self.prev_error) / dt
        self.prev_error = float(ey)

        raw = self.kp * float(ey) + self.ki * self.integral_error + self.kd * deriv + self.ktheta * float(etheta)
        return max(self.output_min, min(self.output_max, raw))
