"""Physics-based Constant Turn Rate and Acceleration (CTRA) Trajectory Predictor.
Serves as an analytical benchmark and robust real-time fallback.
"""

from __future__ import annotations
import math
import numpy as np
from typing import Tuple
from trajectory_prediction.models.base_predictor import BaseTrajectoryPredictor


class CTRAPhysicsPredictor(BaseTrajectoryPredictor):
    """Constant Turn Rate and Acceleration (CTRA) kinematic predictor."""

    def predict(
        self,
        observed_trajectory: np.ndarray,
        agent_class: str = "Car",
    ) -> np.ndarray:
        obs = np.asarray(observed_trajectory, dtype=np.float64)
        if len(obs) < 2:
            # Stationary fallback
            last_pt = obs[-1, :2] if len(obs) > 0 else np.zeros(2)
            return np.tile(last_pt, (self.pred_steps, 1))

        # Estimate instantaneous kinematic state from last observations
        dt = self.dt
        p_curr = obs[-1, :2]
        p_prev = obs[-2, :2]

        v_vec = (p_curr - p_prev) / dt
        speed = float(np.linalg.norm(v_vec))

        if speed < 0.2 or agent_class in ["Cattle", "Cow"]:
            # Stationary or near-stationary obstacle
            if agent_class in ["Cattle", "Cow"]:
                return np.tile(p_curr, (self.pred_steps, 1))

        yaw = math.atan2(v_vec[1], v_vec[0])

        # Estimate yaw rate from last 3 points if available
        if len(obs) >= 3:
            p_prev2 = obs[-3, :2]
            v_prev = (p_prev - p_prev2) / dt
            yaw_prev = math.atan2(v_prev[1], v_prev[0])
            yaw_diff = math.atan2(math.sin(yaw - yaw_prev), math.cos(yaw - yaw_prev))
            yaw_rate = float(np.clip(yaw_diff / dt, -0.6, 0.6))
        else:
            yaw_rate = 0.0

        # Acceleration estimate
        if len(obs) >= 3:
            speed_prev = float(np.linalg.norm(v_prev))
            accel = float(np.clip((speed - speed_prev) / dt, -4.0, 2.5))
        else:
            accel = 0.0

        # Rollout forward 50 steps
        future_traj = np.zeros((self.pred_steps, 2), dtype=np.float64)
        curr_x, curr_y = p_curr[0], p_curr[1]
        curr_v = speed
        curr_yaw = yaw

        for t in range(self.pred_steps):
            curr_v = max(0.0, curr_v + accel * dt)
            curr_yaw = curr_yaw + yaw_rate * dt

            curr_x += curr_v * math.cos(curr_yaw) * dt
            curr_y += curr_v * math.sin(curr_yaw) * dt
            future_traj[t] = [curr_x, curr_y]

        return future_traj

    def predict_multimodal(
        self,
        observed_trajectory: np.ndarray,
        num_modes: int = 3,
        agent_class: str = "Car",
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Generates 3 kinematic modes: [Center Cruise, Left Swerve/Cut-in, Right Swerve]."""
        base_traj = self.predict(observed_trajectory, agent_class)
        trajectories = np.zeros((num_modes, self.pred_steps, 2), dtype=np.float64)
        trajectories[0] = base_traj

        # Lateral deviation offsets over 5 seconds
        t_ramp = (np.arange(self.pred_steps) / float(self.pred_steps))**2

        if num_modes >= 3:
            # Mode 1: Leftward deviation (+1.5m max at 5s)
            trajectories[1, :, 0] = base_traj[:, 0]
            trajectories[1, :, 1] = base_traj[:, 1] + 1.50 * t_ramp

            # Mode 2: Rightward deviation (-1.5m max at 5s)
            trajectories[2, :, 0] = base_traj[:, 0]
            trajectories[2, :, 1] = base_traj[:, 1] - 1.50 * t_ramp

            probs = np.array([0.65, 0.20, 0.15], dtype=np.float64)
        else:
            probs = np.ones(num_modes) / num_modes

        return trajectories, probs
