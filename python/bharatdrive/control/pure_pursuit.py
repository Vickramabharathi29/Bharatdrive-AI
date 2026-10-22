"""
Pure Pursuit Geometric Path Tracking Controller.
BharatDrive-AI - MathWorks Autonomous Driving Hackathon.
"""

from __future__ import annotations
import math
import numpy as np


class PurePursuitController:
    """
    Pure Pursuit path tracking controller formulated at the rear axle center.
    """

    def __init__(
        self,
        wheelbase: float = 2.498,
        max_steer: float = 0.55,
        kdd: float = 0.20,
        l0: float = 3.50,
    ) -> None:
        self.wheelbase = float(wheelbase)
        self.max_steer = float(max_steer)
        self.kdd = float(kdd)
        self.l0 = float(l0)
        self.min_lookahead = 3.0
        self.max_lookahead = 25.0

    def compute_steering(
        self,
        ego_state: np.ndarray | list[float],
        waypoints: np.ndarray | list,
    ) -> tuple[float, np.ndarray, float, float]:
        """
        Evaluates pure pursuit steering angle.

        Args:
            ego_state: [x, y, v, yaw]
            waypoints: Nx2 or Nx3 array

        Returns:
            steer_cmd: front steering angle (rad)
            lookahead_pt: [x_la, y_la]
            ld: lookahead distance (m)
            ey: cross-track error to nearest waypoint (m)
        """
        ego = np.asarray(ego_state, dtype=np.float64)
        xr, yr = float(ego[0]), float(ego[1])
        v = max(0.1, float(ego[2]))
        yaw = float(ego[3])

        wps = np.asarray(waypoints, dtype=np.float64)

        # 1. Dynamic lookahead distance
        ld = max(self.min_lookahead, min(self.max_lookahead, self.kdd * v + self.l0))

        # 2. Nearest waypoint and cross-track error
        dists = np.hypot(wps[:, 0] - xr, wps[:, 1] - yr)
        nearest_idx = int(np.argmin(dists))
        ey = float(dists[nearest_idx])

        # 3. Find lookahead target point
        target_idx = nearest_idx
        for i in range(nearest_idx, len(wps)):
            d = math.hypot(wps[i, 0] - xr, wps[i, 1] - yr)
            if d >= ld:
                target_idx = i
                break
        if target_idx == nearest_idx and len(wps) > nearest_idx + 1:
            target_idx = len(wps) - 1

        lookahead_pt = wps[target_idx, :2]

        # 4. Angle alpha between heading and lookahead vector
        dx = lookahead_pt[0] - xr
        dy = lookahead_pt[1] - yr
        target_angle = math.atan2(dy, dx)
        alpha = math.atan2(math.sin(target_angle - yaw), math.cos(target_angle - yaw))

        # 5. Steering angle: delta = atan(2 * L * sin(alpha) / ld)
        raw_steer = math.atan2(2.0 * self.wheelbase * math.sin(alpha), ld)
        steer_cmd = max(-self.max_steer, min(self.max_steer, raw_steer))

        return steer_cmd, lookahead_pt, ld, ey
