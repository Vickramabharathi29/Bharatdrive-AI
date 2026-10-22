"""Abstract Base Class for Multi-Agent Trajectory Predictors.
Adheres to SOLID principles (Interface Segregation and Liskov Substitution).
"""

from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Tuple, List, Dict, Any
import numpy as np


class BaseTrajectoryPredictor(ABC):
    """Abstract interface for autonomous driving trajectory prediction."""

    def __init__(self, obs_horizon_s: float = 2.0, pred_horizon_s: float = 5.0, dt: float = 0.1):
        self.obs_horizon_s = obs_horizon_s
        self.pred_horizon_s = pred_horizon_s
        self.dt = dt
        self.obs_steps = int(round(obs_horizon_s / dt))   # e.g. 20 steps
        self.pred_steps = int(round(pred_horizon_s / dt)) # e.g. 50 steps

    @abstractmethod
    def predict(
        self,
        observed_trajectory: np.ndarray,
        agent_class: str = "Car",
    ) -> np.ndarray:
        """Predicts the most probable single trajectory over the next 5 seconds.
        
        Args:
            observed_trajectory: Array of shape (T_obs, 2) or (T_obs, 4) [x, y, vx, vy].
            agent_class: Semantic category string.
            
        Returns:
            np.ndarray: Array of shape (T_pred, 2) predicted [x, y] coordinates.
        """
        pass

    @abstractmethod
    def predict_multimodal(
        self,
        observed_trajectory: np.ndarray,
        num_modes: int = 3,
        agent_class: str = "Car",
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Predicts top-K multimodal trajectory hypotheses with probability scores.
        
        Returns:
            trajectories: Array of shape (K, T_pred, 2).
            probabilities: Array of shape (K,) summing to 1.0.
        """
        pass
