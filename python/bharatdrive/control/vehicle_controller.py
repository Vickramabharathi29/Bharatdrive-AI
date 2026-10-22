"""
Unified Vehicle Controller Engine.
BharatDrive-AI - MathWorks Autonomous Driving Hackathon.

Coordinates lateral path tracking (Stanley / Pure Pursuit / PID)
and longitudinal speed tracking (Anti-Windup PID).
"""

from __future__ import annotations
import math
from typing import Any
import numpy as np

from bharatdrive.control.pid_controller import PIDController
from bharatdrive.control.pure_pursuit import PurePursuitController
from bharatdrive.control.stanley_controller import StanleyController


class VehicleControllerEngine:
    """
    Unified Motion Controller for Autonomous Vehicles.
    """

    def __init__(self, controller_type: str = "Stanley") -> None:
        self.wheelbase = 2.498
        self.max_steer = 0.55
        self.max_accel = 3.50
        self.max_decel = -6.50

        self.stanley_ctrl = StanleyController(self.wheelbase, self.max_steer, 2.20, 1.0, 0.15)
        self.pure_pursuit_ctrl = PurePursuitController(self.wheelbase, self.max_steer, 0.20, 3.50)
        self.lateral_pid_ctrl = PIDController(0.60, 0.05, 0.20, -self.max_steer, self.max_steer, 0.50)

        self.longitudinal_ctrl = PIDController(1.20, 0.15, 0.08, self.max_decel, self.max_accel)

        self.controller_type = controller_type
        self.set_controller_type(controller_type)

    def set_controller_type(self, controller_type: str) -> None:
        c_type = controller_type.lower()
        if c_type in ("stanley", "purepursuit", "pid"):
            self.controller_type = c_type
        else:
            self.controller_type = "stanley"

    def compute_controls(
        self,
        ego_state: np.ndarray | list[float],
        waypoints: np.ndarray | list,
        target_v: float,
        dt: float = 0.01,
        kappa_ref: float = 0.0,
    ) -> dict[str, float]:
        """
        Calculates coupled steer angle and longitudinal acceleration.

        Args:
            ego_state: [x, y, v, yaw, yaw_rate]
            waypoints: Nx2 or Nx3 array
            target_v: reference cruise speed (m/s)
            dt: time step in seconds
            kappa_ref: path curvature at target point (1/m)

        Returns:
            dict containing:
                - 'steer_cmd': front steer angle (rad)
                - 'accel_cmd': drive/brake acceleration (m/s^2)
                - 'brake_pressure': estimated brake pressure (0-100 bar)
                - 'ey': cross-track error (m)
                - 'etheta': heading error (rad)
        """
        ego = np.asarray(ego_state, dtype=np.float64)
        curr_v = float(ego[2])
        yaw_rate = float(ego[4]) if len(ego) > 4 else 0.0

        # Longitudinal Acceleration Command
        accel_cmd = self.longitudinal_ctrl.compute_longitudinal(target_v, curr_v, dt)

        # Lateral Steering Command
        if self.controller_type == "stanley":
            steer_cmd, ey, etheta = self.stanley_ctrl.compute_steering(ego[:4], waypoints, yaw_rate, kappa_ref)
        elif self.controller_type == "purepursuit":
            steer_cmd, _, _, ey = self.pure_pursuit_ctrl.compute_steering(ego[:4], waypoints)
            wps = np.asarray(waypoints, dtype=np.float64)
            dists = np.hypot(wps[:, 0] - ego[0], wps[:, 1] - ego[1])
            idx = int(np.argmin(dists))
            path_yaw = float(wps[idx, 2]) if wps.shape[1] >= 3 else 0.0
            etheta = math.atan2(math.sin(path_yaw - ego[3]), math.cos(path_yaw - ego[3]))
        else:  # pid
            wps = np.asarray(waypoints, dtype=np.float64)
            dists = np.hypot(wps[:, 0] - ego[0], wps[:, 1] - ego[1])
            idx = int(np.argmin(dists))
            path_yaw = float(wps[idx, 2]) if wps.shape[1] >= 3 else 0.0
            dx = ego[0] - wps[idx, 0]
            dy = ego[1] - wps[idx, 1]
            ey = -dx * math.sin(path_yaw) + dy * math.cos(path_yaw)
            etheta = math.atan2(math.sin(path_yaw - ego[3]), math.cos(path_yaw - ego[3]))
            steer_cmd = self.lateral_pid_ctrl.compute_lateral(ey, etheta, dt)

        brake_pressure = min(100.0, abs(accel_cmd) * 15.38) if accel_cmd < 0.0 else 0.0

        return {
            "steer_cmd": float(steer_cmd),
            "accel_cmd": float(accel_cmd),
            "brake_pressure": float(brake_pressure),
            "ey": float(ey),
            "etheta": float(etheta),
        }
