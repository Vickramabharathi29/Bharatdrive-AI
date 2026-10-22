"""LSTM-based Trajectory Predictor with Cell and Hidden State Encoder-Decoder."""

from __future__ import annotations
import math
import numpy as np
from typing import Tuple
from trajectory_prediction.models.base_predictor import BaseTrajectoryPredictor
from trajectory_prediction.models.physics_kalman_predictor import CTRAPhysicsPredictor


class LSTMTrajectoryPredictor(BaseTrajectoryPredictor):
    """LSTM Encoder-Decoder network for sequence-to-sequence trajectory forecasting."""

    def __init__(
        self,
        obs_horizon_s: float = 2.0,
        pred_horizon_s: float = 5.0,
        dt: float = 0.1,
        hidden_dim: int = 64,
    ):
        super().__init__(obs_horizon_s, pred_horizon_s, dt)
        self.hidden_dim = hidden_dim
        self.physics_fallback = CTRAPhysicsPredictor(obs_horizon_s, pred_horizon_s, dt)

        rng = np.random.default_rng(101)
        # 4 gates: Forget (f), Input (i), Candidate (c), Output (o)
        self.W_f = rng.normal(0, 0.1, (hidden_dim, 4 + hidden_dim))
        self.W_i = rng.normal(0, 0.1, (hidden_dim, 4 + hidden_dim))
        self.W_c = rng.normal(0, 0.1, (hidden_dim, 4 + hidden_dim))
        self.W_o = rng.normal(0, 0.1, (hidden_dim, 4 + hidden_dim))
        self.W_proj = rng.normal(0, 0.1, (2, hidden_dim))

    def _lstm_cell(self, x: np.ndarray, h: np.ndarray, c: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        xh = np.concatenate([x, h])
        f = 1.0 / (1.0 + np.exp(-np.clip(self.W_f @ xh, -15.0, 15.0)))
        i = 1.0 / (1.0 + np.exp(-np.clip(self.W_i @ xh, -15.0, 15.0)))
        c_cand = np.tanh(self.W_c @ xh)
        c_next = f * c + i * c_cand
        o = 1.0 / (1.0 + np.exp(-np.clip(self.W_o @ xh, -15.0, 15.0)))
        h_next = o * np.tanh(c_next)
        return h_next, c_next

    def predict(
        self,
        observed_trajectory: np.ndarray,
        agent_class: str = "Car",
    ) -> np.ndarray:
        obs = np.asarray(observed_trajectory, dtype=np.float64)
        if len(obs) < 2:
            return self.physics_fallback.predict(obs, agent_class)

        deltas = np.diff(obs[:, :2], axis=0)
        h = np.zeros(self.hidden_dim, dtype=np.float64)
        c = np.zeros(self.hidden_dim, dtype=np.float64)

        for t in range(len(deltas)):
            dx, dy = deltas[t]
            feat = np.array([dx, dy, dx / self.dt, dy / self.dt], dtype=np.float64)
            h, c = self._lstm_cell(feat, h, c)

        curr_pos = np.copy(obs[-1, :2])
        future_coords = np.zeros((self.pred_steps, 2), dtype=np.float64)
        physics_traj = self.physics_fallback.predict(obs, agent_class)
        physics_deltas = np.diff(np.vstack([curr_pos, physics_traj]), axis=0)

        for step in range(self.pred_steps):
            d_pred = self.W_proj @ h
            d_phys = physics_deltas[step]
            alpha = min(0.85, 0.35 + 0.50 * (step / float(self.pred_steps)))
            d_final = (1.0 - alpha) * d_pred + alpha * d_phys

            curr_pos += d_final
            future_coords[step] = curr_pos

            feat = np.array([d_final[0], d_final[1], d_final[0] / self.dt, d_final[1] / self.dt])
            h, c = self._lstm_cell(feat, h, c)

        return future_coords

    def predict_multimodal(
        self,
        observed_trajectory: np.ndarray,
        num_modes: int = 3,
        agent_class: str = "Car",
    ) -> Tuple[np.ndarray, np.ndarray]:
        base = self.predict(observed_trajectory, agent_class)
        trajectories = np.zeros((num_modes, self.pred_steps, 2), dtype=np.float64)
        trajectories[0] = base

        t_ramp = (np.arange(self.pred_steps) / float(self.pred_steps))**1.5
        if num_modes >= 3:
            trajectories[1, :, 0] = base[:, 0]
            trajectories[1, :, 1] = base[:, 1] + 1.30 * t_ramp
            trajectories[2, :, 0] = base[:, 0]
            trajectories[2, :, 1] = base[:, 1] - 1.30 * t_ramp
            probs = np.array([0.65, 0.20, 0.15], dtype=np.float64)
        else:
            probs = np.ones(num_modes) / num_modes

        return trajectories, probs
