"""Multi-modal sensor suite simulation: Camera, LiDAR, and Radar."""

from __future__ import annotations
from dataclasses import dataclass, field
import math
import numpy as np
from typing import List, Dict, Any, Optional


@dataclass
class SensorDetection:
    sensor_type: str                  # 'Camera', 'LiDAR', 'Radar'
    target_id: int
    measurement: np.ndarray           # [x, y, z] or [x, y, doppler]
    measurement_noise: np.ndarray     # Covariance matrix R
    object_class: str = "Unknown"
    confidence: float = 0.8
    dimensions: np.ndarray = field(default_factory=lambda: np.array([2.0, 1.0, 1.2]))
    doppler_velocity: Optional[float] = None
    time: float = 0.0


class SensorSuite:
    """Simulates Camera, 3D LiDAR, and Doppler Radar sensors mounted on the ego vehicle."""

    def __init__(self, weather_factor: float = 0.15):
        self.weather = weather_factor
        # FOVs and Ranges
        self.cam_fov_deg = 65.0
        self.cam_max_range = 80.0
        self.lidar_fov_deg = 120.0
        self.lidar_max_range = 90.0
        self.radar_fov_deg = 45.0
        self.radar_max_range = 150.0

    def generate_measurements(
        self,
        ego_state: np.ndarray,
        actors: List[Dict[str, Any]],
        t: float,
    ) -> Dict[str, List[SensorDetection]]:
        """Generates synthetic sensor detections in ego body coordinate frame."""
        ego_x, ego_y, ego_v, ego_yaw = ego_state[:4]
        cos_y = math.cos(ego_yaw)
        sin_y = math.sin(ego_yaw)

        cam_list: List[SensorDetection] = []
        lidar_list: List[SensorDetection] = []
        radar_list: List[SensorDetection] = []

        for act in actors:
            act_pos = np.asarray(act["position"], dtype=np.float64)
            act_vel = np.asarray(act.get("velocity", [0.0, 0.0, 0.0]), dtype=np.float64)
            act_id = int(act["id"])
            act_class = str(act.get("class", "Car"))
            act_dims = np.asarray(act.get("dimensions", [2.0, 1.0, 1.2]), dtype=np.float64)

            # Global delta
            dx_g = act_pos[0] - ego_x
            dy_g = act_pos[1] - ego_y

            # Transform to ego body frame
            x_b = dx_g * cos_y + dy_g * sin_y
            y_b = -dx_g * sin_y + dy_g * cos_y
            z_b = act_pos[2] if len(act_pos) > 2 else 0.0

            range_m = math.hypot(x_b, y_b)
            azimuth_deg = math.degrees(math.atan2(y_b, x_b))

            # Relative velocity
            vx_g = act_vel[0] - ego_v * cos_y
            vy_g = act_vel[1] - ego_v * sin_y
            vx_b = vx_g * cos_y + vy_g * sin_y
            vy_b = -vx_g * sin_y + vy_g * cos_y
            radial_vel = (x_b * vx_b + y_b * vy_b) / max(range_m, 0.1)

            # 1. Camera Detection
            if range_m <= self.cam_max_range and abs(azimuth_deg) <= (self.cam_fov_deg / 2.0):
                cam_det = SensorDetection(
                    sensor_type="Camera",
                    target_id=act_id,
                    measurement=np.array([x_b, y_b, z_b], dtype=np.float64),
                    measurement_noise=np.diag([max(0.5, 0.04 * x_b)**2, 0.15**2, 0.2**2]),
                    object_class=act_class,
                    confidence=max(0.60, 0.95 - (range_m / 100.0) - self.weather * 0.2),
                    dimensions=act_dims,
                    time=t,
                )
                cam_list.append(cam_det)

            # 2. LiDAR Detection
            if range_m <= self.lidar_max_range and abs(azimuth_deg) <= (self.lidar_fov_deg / 2.0):
                lidar_det = SensorDetection(
                    sensor_type="LiDAR",
                    target_id=act_id,
                    measurement=np.array([x_b, y_b, z_b], dtype=np.float64),
                    measurement_noise=np.diag([0.06**2, 0.06**2, 0.03**2]),
                    object_class=act_class,
                    confidence=0.92,
                    dimensions=act_dims,
                    time=t,
                )
                lidar_list.append(lidar_det)

            # 3. Radar Detection
            if range_m <= self.radar_max_range and abs(azimuth_deg) <= (self.radar_fov_deg / 2.0):
                radar_det = SensorDetection(
                    sensor_type="Radar",
                    target_id=act_id,
                    measurement=np.array([x_b, y_b, radial_vel], dtype=np.float64),
                    measurement_noise=np.diag([0.25**2, 0.70**2, 0.15**2]),
                    object_class=act_class,
                    confidence=0.88,
                    dimensions=act_dims,
                    doppler_velocity=float(radial_vel),
                    time=t,
                )
                radar_list.append(radar_det)

        return {
            "Camera": cam_list,
            "LiDAR": lidar_list,
            "Radar": radar_list,
        }
