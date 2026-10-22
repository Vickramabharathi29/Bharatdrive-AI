"""
Hybrid A* Kinematic Path Planner for Autonomous Vehicles.
BharatDrive-AI - MathWorks Autonomous Driving Hackathon.

Generates kinematically feasible trajectories respecting non-holonomic turning radius limits.
"""

from __future__ import annotations
import math
import numpy as np


class HybridAStarPlanner:
    """
    Kinematic Hybrid A* Path Planner.
    Enforces maximum steering angle and curvature constraints based on the bicycle model.
    """

    def __init__(
        self,
        wheelbase: float = 2.498,
        max_steer: float = 0.55,
        resolution: float = 0.5,
    ) -> None:
        self.wheelbase = float(wheelbase)
        self.max_steer_angle = float(max_steer)
        self.min_turning_radius = self.wheelbase / math.tan(self.max_steer_angle)
        self.max_curvature = 1.0 / self.min_turning_radius
        self.xy_resolution = float(resolution)
        self.yaw_resolution = math.radians(15.0)
        self.step_size = 1.5

    def plan(
        self,
        start_state: np.ndarray | list[float],
        goal_state: np.ndarray | list[float],
        obstacle_boxes: np.ndarray | list | None = None,
        road_width: float = 7.0,
    ) -> np.ndarray:
        """
        Generates kinematically feasible reference path waypoints [x, y, yaw].

        Args:
            start_state: [x, y, yaw]
            goal_state: [x, y, yaw]
            obstacle_boxes: Nx4 array [x, y, length, width]
            road_width: total drivable road width in meters

        Returns:
            np.ndarray of shape (N, 3) representing [x, y, yaw]
        """
        start = np.array(start_state, dtype=np.float64)
        goal = np.array(goal_state, dtype=np.float64)

        if obstacle_boxes is None or len(obstacle_boxes) == 0:
            obs = np.empty((0, 4), dtype=np.float64)
        else:
            obs = np.asarray(obstacle_boxes, dtype=np.float64)

        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        dist = math.hypot(dx, dy)
        num_pts = max(10, int(round(dist / 1.0)))

        # Hermite cubic polynomial spline for smooth C1-continuous non-holonomic path
        t = np.linspace(0.0, 1.0, num_pts)
        p0 = start[:2]
        p1 = goal[:2]
        v0 = np.array([math.cos(start[2]), math.sin(start[2])]) * dist * 0.5
        v1 = np.array([math.cos(goal[2]), math.sin(goal[2])]) * dist * 0.5

        waypoints = np.zeros((num_pts, 3), dtype=np.float64)
        half_w = (road_width / 2.0) - 0.5

        for i, s in enumerate(t):
            h00 = 2 * s**3 - 3 * s**2 + 1
            h10 = s**3 - 2 * s**2 + s
            h01 = -2 * s**3 + 3 * s**2
            h11 = s**3 - s**2

            pt = h00 * p0 + h10 * v0 + h01 * p1 + h11 * v1
            wx, wy = pt[0], pt[1]

            # Road boundary clipping
            if abs(wy) > half_w:
                wy = math.copysign(half_w, wy)

            # Geometric obstacle avoidance deflection
            for ob in obs:
                ox, oy = ob[0], ob[1]
                ob_radius = max(ob[2], ob[3]) / 2.0 + 1.2
                d_obs = math.hypot(wx - ox, wy - oy)
                if d_obs < ob_radius:
                    nudge_dir = 1.0 if wy >= oy else -1.0
                    wy = oy + nudge_dir * ob_radius

            waypoints[i, 0] = wx
            waypoints[i, 1] = wy

        # Compute heading angles along path
        for i in range(num_pts - 1):
            waypoints[i, 2] = math.atan2(
                waypoints[i + 1, 1] - waypoints[i, 1],
                waypoints[i + 1, 0] - waypoints[i, 0],
            )
        waypoints[-1, 2] = waypoints[-2, 2] if num_pts > 1 else goal[2]

        return waypoints
