"""Standardized TrackedObject representation for multi-sensor fusion outputs."""

from __future__ import annotations
from dataclasses import dataclass, field
import math
import numpy as np
from typing import List, Dict, Any


@dataclass
class TrackedObject:
    """Represents a fused tracked physical obstacle in ego vehicle body frame."""
    object_id: int
    object_class: str                          # 'Auto-Rickshaw', 'Two-Wheeler', 'Cattle', 'Pedestrian', 'Car', 'Truck-Bus', 'Pothole'
    position: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))  # [x, y, z] in meters
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))  # [vx, vy, vz] in m/s
    dimensions: np.ndarray = field(default_factory=lambda: np.array([2.0, 1.0, 1.2], dtype=np.float64)) # [L, W, H]
    distance: float = 0.0                      # range rho = sqrt(x^2 + y^2) (m)
    relative_speed: float = 0.0                # closing speed (m/s)
    confidence_score: float = 0.5              # fused detection probability in [0.0, 1.0]
    yaw: float = 0.0                           # heading in ego frame (rad)
    sensor_contributors: List[str] = field(default_factory=list) # e.g. ['Camera', 'Radar', 'LiDAR']
    track_status: str = "Tentative"            # 'Tentative', 'Confirmed', 'Coasted'
    age: int = 1
    total_hits: int = 1

    def __post_init__(self):
        self.update_metrics()

    def update_metrics(self):
        self.distance = float(math.hypot(self.position[0], self.position[1]))
        self.relative_speed = float(-self.velocity[0])

    def to_dict(self) -> Dict[str, Any]:
        return {
            "object_id": self.object_id,
            "object_class": self.object_class,
            "position": self.position.tolist(),
            "velocity": self.velocity.tolist(),
            "distance": round(self.distance, 2),
            "relative_speed": round(self.relative_speed, 2),
            "confidence_score": round(self.confidence_score, 3),
            "dimensions": self.dimensions.tolist(),
            "yaw": round(self.yaw, 3),
            "sensor_contributors": self.sensor_contributors,
            "track_status": self.track_status,
        }
