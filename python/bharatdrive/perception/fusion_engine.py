"""Sensor Fusion Engine implementing Extended Kalman Filter (EKF) and track management."""

from __future__ import annotations
import math
import numpy as np
from typing import List, Dict, Any, Optional
from bharatdrive.perception.tracked_object import TrackedObject
from bharatdrive.perception.sensor_suite import SensorDetection


class InternalTrack:
    """Internal tracking state for a single detected obstacle."""

    def __init__(self, track_id: int, initial_meas: SensorDetection):
        self.track_id = track_id
        # State vector: [x, y, vx, vy]^T
        self.state = np.array([initial_meas.measurement[0], initial_meas.measurement[1], 0.0, 0.0], dtype=np.float64)
        self.cov = np.diag([0.5**2, 0.5**2, 3.0**2, 3.0**2])
        self.age = 1
        self.total_hits = 1
        self.consecutive_misses = 0
        self.status = "Tentative"
        self.active_sensors: List[str] = [initial_meas.sensor_type]
        self.object_class = initial_meas.object_class
        self.dimensions = np.copy(initial_meas.dimensions)


class SensorFusionEngine:
    """Central multi-sensor fusion engine fusing Camera, LiDAR, and Radar."""

    def __init__(self, dt: float = 0.05):
        self.dt = dt
        self.tracks: List[InternalTrack] = []
        self.next_track_id = 1
        self.confirmation_threshold = 2
        self.deletion_threshold = 4
        self.gate_threshold = 12.0 # Chi-square gating threshold

        # Process noise covariance Q (Constant Velocity model)
        q_pos = 0.15
        q_vel = 1.20
        self.Q = np.diag([q_pos**2, q_pos**2, q_vel**2, q_vel**2]) * dt

    def step(self, detections: Dict[str, List[SensorDetection]], current_time: float) -> List[TrackedObject]:
        self.predict()
        self.update_sensor(detections.get("LiDAR", []), "LiDAR")
        self.update_sensor(detections.get("Camera", []), "Camera")
        self.update_sensor(detections.get("Radar", []), "Radar")
        self.manage_tracks()
        return self.format_output(current_time)

    def predict(self):
        dt = self.dt
        F = np.array([
            [1.0, 0.0,  dt, 0.0],
            [0.0, 1.0, 0.0,  dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ], dtype=np.float64)

        for tr in self.tracks:
            tr.state = F @ tr.state
            tr.cov = F @ tr.cov @ F.T + self.Q
            tr.age += 1
            tr.consecutive_misses += 1
            tr.active_sensors = []

    def update_sensor(self, det_list: List[SensorDetection], sensor_type: str):
        if not det_list:
            return

        for meas in det_list:
            z_pos = meas.measurement[:2]

            # Data association via Mahalanobis distance
            best_idx = -1
            min_dist = self.gate_threshold

            H_pos = np.array([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
            ])

            for idx, tr in enumerate(self.tracks):
                z_pred = H_pos @ tr.state
                y = z_pos - z_pred
                S = H_pos @ tr.cov @ H_pos.T + meas.measurement_noise[:2, :2]
                try:
                    d_mah = float(y.T @ np.linalg.solve(S, y))
                    if d_mah < min_dist:
                        min_dist = d_mah
                        best_idx = idx
                except np.linalg.LinAlgError:
                    continue

            if best_idx >= 0:
                self.apply_kalman_update(self.tracks[best_idx], meas, sensor_type)
            else:
                new_track = InternalTrack(self.next_track_id, meas)
                self.next_track_id += 1
                self.tracks.append(new_track)

    def apply_kalman_update(self, tr: InternalTrack, meas: SensorDetection, sensor_type: str):
        if sensor_type == "Radar" and meas.doppler_velocity is not None:
            # Non-linear EKF for Radar with Doppler
            x, y, vx, vy = tr.state
            r = max(math.hypot(x, y), 0.1)
            z = np.array([meas.measurement[0], meas.measurement[1], meas.doppler_velocity])
            z_pred = np.array([x, y, (x * vx + y * vy) / r])

            dr_dx = (y**2 * vx - x * y * vy) / (r**3)
            dr_dy = (x**2 * vy - x * y * vx) / (r**3)
            H = np.array([
                [1.0,   0.0,   0.0, 0.0],
                [0.0,   1.0,   0.0, 0.0],
                [dr_dx, dr_dy, x/r, y/r],
            ])
            R = meas.measurement_noise
        else:
            # Linear position update for Camera and LiDAR
            z = meas.measurement[:2]
            H = np.array([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
            ])
            z_pred = H @ tr.state
            R = meas.measurement_noise[:2, :2]

        y = z - z_pred
        S = H @ tr.cov @ H.T + R
        K = tr.cov @ H.T @ np.linalg.inv(S)

        tr.state = tr.state + K @ y
        tr.cov = (np.eye(4) - K @ H) @ tr.cov
        tr.total_hits += 1
        tr.consecutive_misses = 0

        if sensor_type not in tr.active_sensors:
            tr.active_sensors.append(sensor_type)

        if sensor_type == "Camera" and meas.object_class != "Unknown":
            tr.object_class = meas.object_class
        if meas.dimensions is not None:
            tr.dimensions = np.copy(meas.dimensions)

    def manage_tracks(self):
        surviving: List[InternalTrack] = []
        for tr in self.tracks:
            if tr.status == "Tentative" and tr.total_hits >= self.confirmation_threshold:
                tr.status = "Confirmed"
            if tr.consecutive_misses > 0 and tr.status == "Confirmed":
                tr.status = "Coasted"
            if tr.consecutive_misses <= self.deletion_threshold:
                surviving.append(tr)
        self.tracks = surviving

    def format_output(self, current_time: float) -> List[TrackedObject]:
        output_list: List[TrackedObject] = []
        for tr in self.tracks:
            if tr.consecutive_misses > 2:
                continue

            pos = np.array([tr.state[0], tr.state[1], 0.0], dtype=np.float64)
            vel = np.array([tr.state[2], tr.state[3], 0.0], dtype=np.float64)

            hit_ratio = min(1.0, tr.total_hits / max(1, tr.age))
            sensor_bonus = min(0.30, 0.10 * len(tr.active_sensors))
            cov_factor = 1.0 / (1.0 + np.trace(tr.cov[:2, :2]))
            conf = min(0.99, max(0.20, 0.50 * hit_ratio + sensor_bonus + 0.20 * cov_factor))

            out = TrackedObject(
                object_id=tr.track_id,
                object_class=tr.object_class,
                position=pos,
                velocity=vel,
                dimensions=tr.dimensions,
                confidence_score=float(conf),
                sensor_contributors=list(tr.active_sensors),
                track_status=tr.status,
                age=tr.age,
                total_hits=tr.total_hits,
            )
            output_list.append(out)
        return output_list
