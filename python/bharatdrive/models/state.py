"""Vehicle state representations for Cartesian and Frenet coordinate frames.

Adheres to ISO 8855 intermediate axis system:
- X-axis: Forward along vehicle longitudinal axis
- Y-axis: Leftward along vehicle lateral axis
- Z-axis: Upward normal to road plane
"""

from __future__ import annotations
from dataclasses import dataclass, field
import math
import numpy as np
from typing import Tuple, List
from bharatdrive.config import VehicleDimensions


@dataclass
class VehicleState:
    """Represents the instantaneous physical state of the ego or obstacle vehicle in Cartesian coordinates."""
    x: float = 0.0          # Global X coordinate [m]
    y: float = 0.0          # Global Y coordinate [m]
    v: float = 0.0          # Longitudinal velocity [m/s]
    yaw: float = 0.0        # Heading angle theta [rad] in [-pi, pi]
    yaw_rate: float = 0.0   # Yaw angular velocity omega = d(theta)/dt [rad/s]
    a: float = 0.0          # Longitudinal acceleration [m/s^2]
    steer: float = 0.0      # Front wheel steering angle delta [rad]
    v_y: float = 0.0        # Lateral velocity [m/s] (for dynamic model slip calculation)
    t: float = 0.0          # Timestamp [s]

    def __post_init__(self):
        # Normalize yaw to [-pi, pi]
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

    def as_vector(self) -> np.ndarray:
        """Returns fundamental state vector [x, y, v, yaw, yaw_rate, a, steer]."""
        return np.array([self.x, self.y, self.v, self.yaw, self.yaw_rate, self.a, self.steer], dtype=np.float64)

    @classmethod
    def from_vector(cls, vec: np.ndarray, t: float = 0.0) -> VehicleState:
        return cls(
            x=float(vec[0]),
            y=float(vec[1]),
            v=float(vec[2]),
            yaw=float(vec[3]),
            yaw_rate=float(vec[4]) if len(vec) > 4 else 0.0,
            a=float(vec[5]) if len(vec) > 5 else 0.0,
            steer=float(vec[6]) if len(vec) > 6 else 0.0,
            t=t,
        )

    def get_corners(self, dims: VehicleDimensions | None = None) -> np.ndarray:
        """Computes the 4 corners of the vehicle bounding box in global Cartesian coordinates.
        
        Returns:
            np.ndarray: 4x2 array of corner points [FL, FR, RR, RL] (x, y).
        """
        if dims is None:
            dims = VehicleDimensions()

        # Distance from vehicle reference center (rear axle or CG) to front/rear bumper
        # Standard convention: reference point is center of rear axle
        half_w = dims.width / 2.0
        front_x = dims.wheelbase + dims.front_overhang
        rear_x = -dims.rear_overhang

        local_corners = np.array([
            [front_x, half_w],   # Front Left
            [front_x, -half_w],  # Front Right
            [rear_x, -half_w],   # Rear Right
            [rear_x, half_w],    # Rear Left
        ])

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        rot_matrix = np.array([
            [cos_yaw, -sin_yaw],
            [sin_yaw, cos_yaw],
        ])

        global_corners = (rot_matrix @ local_corners.T).T + np.array([self.x, self.y])
        return global_corners

    def get_axis_aligned_bbox(self, dims: VehicleDimensions | None = None) -> Tuple[float, float, float, float]:
        """Returns axis-aligned bounding box (min_x, min_y, max_x, max_y)."""
        corners = self.get_corners(dims)
        min_x, min_y = np.min(corners, axis=0)
        max_x, max_y = np.max(corners, axis=0)
        return float(min_x), float(min_y), float(max_x), float(max_y)


@dataclass
class FrenetState:
    """Represents vehicle motion decomposed into Curvilinear Frenet-Serret Frame.
    
    Decomposition:
    - s: Longitudinal progress along reference path curve [m]
    - s_d: Longitudinal speed along path ds/dt [m/s]
    - s_dd: Longitudinal acceleration d^2s/dt^2 [m/s^2]
    - s_ddd: Longitudinal jerk d^3s/dt^3 [m/s^3]
    - d: Lateral orthogonal deviation from path [m] (positive = Left, negative = Right)
    - d_d: Lateral velocity with respect to time dd/dt [m/s]
    - d_dd: Lateral acceleration d^2d/dt^2 [m/s^2]
    - d_ddd: Lateral jerk d^3d/dt^3 [m/s^3]
    - t: Timestamp [s]
    """
    s: float = 0.0
    s_d: float = 0.0
    s_dd: float = 0.0
    s_ddd: float = 0.0
    d: float = 0.0
    d_d: float = 0.0
    d_dd: float = 0.0
    d_ddd: float = 0.0
    t: float = 0.0

    def as_vector(self) -> np.ndarray:
        return np.array([self.s, self.s_d, self.s_dd, self.d, self.d_d, self.d_dd], dtype=np.float64)

    @classmethod
    def from_vector(cls, vec: np.ndarray, t: float = 0.0) -> FrenetState:
        return cls(
            s=float(vec[0]),
            s_d=float(vec[1]),
            s_dd=float(vec[2]) if len(vec) > 2 else 0.0,
            d=float(vec[3]) if len(vec) > 3 else 0.0,
            d_d=float(vec[4]) if len(vec) > 4 else 0.0,
            d_dd=float(vec[5]) if len(vec) > 5 else 0.0,
            t=t,
        )
