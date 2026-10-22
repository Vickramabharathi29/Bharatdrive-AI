"""
Stanley Front-Axle Non-Linear Tracking Controller.
BharatDrive-AI - MathWorks Autonomous Driving Hackathon.

Formulated at the center of the front axle with curvature feedforward
and high-speed yaw-rate damping.
"""

from __future__ import annotations
import math
import numpy as np


class StanleyController:
    """
    Adaptive Stanley path tracking controller.
    """

    def __init__(
        self,
        wheelbase: float = 2.498,
        max_steer: float = 0.55,
        k: float = 2.20,
        ksoft: float = 1.00,
        kd: float = 0.15,
        enable_feedforward: bool = True,
    ) -> None:
        self.wheelbase = float(wheelbase)
        self.max_steer = float(max_steer)
        self.k = float(k)
        self.ksoft = float(ksoft)
        self.kd = float(kd)
        self.enable_feedforward = bool(enable_feedforward)

    def compute_steering(
        self,
        ego_state: np.ndarray | list[float],
        waypoints: np.ndarray | list,
        yaw_rate: float = 0.0,
        kappa_ref: float = 0.0,
    ) -> tuple[float, float, float]:
        """
        Evaluates the Stanley control law.

        Args:
            ego_state: [x, y, v, yaw]
            waypoints: Nx2 or Nx3 array [x, y, (optional yaw)]
            yaw_rate: vehicle angular velocity (rad/s)
            kappa_ref: road curvature at target point (1/m)

        Returns:
            steer_cmd: commanded front steer angle (rad)
            ey_f: signed front-axle cross-track error (m)
            etheta: heading alignment error (rad)
        """
        ego = np.asarray(ego_state, dtype=np.float64)
        xr, yr = float(ego[0]), float(ego[1])
        v = max(0.1, float(ego[2]))
        yaw = float(ego[3])

        wps = np.asarray(waypoints, dtype=np.float64)

        # 1. Front Axle Coordinates
        xf = xr + self.wheelbase * math.cos(yaw)
        yf = yr + self.wheelbase * math.sin(yaw)

        # 2. Nearest Waypoint to Front Axle
        dists = np.hypot(wps[:, 0] - xf, wps[:, 1] - yf)
        nearest_idx = int(np.argmin(dists))
        target_pt = wps[nearest_idx]

        # 3. Path Heading
        if wps.shape[1] >= 3:
            path_yaw = float(target_pt[2])
        else:
            if nearest_idx < len(wps) - 1:
                dx = wps[nearest_idx + 1, 0] - wps[nearest_idx, 0]
                dy = wps[nearest_idx + 1, 1] - wps[nearest_idx, 1]
            elif nearest_idx > 0:
                dx = wps[nearest_idx, 0] - wps[nearest_idx - 1, 0]
                dy = wps[nearest_idx, 1] - wps[nearest_idx - 1, 1]
            else:
                dx, dy = 1.0, 0.0
            path_yaw = math.atan2(dy, dx)

        # 4. Heading Alignment Error: wrap to [-pi, pi]
        etheta = math.atan2(math.sin(path_yaw - yaw), math.cos(path_yaw - yaw))

        # 5. Front-Axle Signed Cross-Track Error
        dx_f = xf - target_pt[0]
        dy_f = yf - target_pt[1]
        ey_f = -dx_f * math.sin(path_yaw) + dy_f * math.cos(path_yaw)

        # 6. Non-linear Cross-Track Term with Soft Denominator (steers toward path)
        delta_e = math.atan2(-self.k * ey_f, v + self.ksoft)

        # 7. Curvature Feedforward
        delta_ff = 0.0
        if self.enable_feedforward and abs(kappa_ref) > 1e-4:
            delta_ff = math.atan(self.wheelbase * kappa_ref)

        # 8. Yaw Damping
        desired_yaw_rate = v * kappa_ref
        delta_damp = -self.kd * (yaw_rate - desired_yaw_rate)

        # 9. Total Steering Angle
        raw_steer = etheta + delta_e + delta_ff + delta_damp
        steer_cmd = max(-self.max_steer, min(self.max_steer, raw_steer))

        return steer_cmd, ey_f, etheta
