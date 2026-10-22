"""Spatio-Temporal Dynamic Costmap Grid generation in Python."""

from __future__ import annotations
import numpy as np
from typing import List, Tuple, Dict, Any


class SpatioTemporalCostmap:
    """Generates 2D risk potential fields around detected obstacles in vehicle body frame."""

    def __init__(
        self,
        x_range: Tuple[float, float] = (-10.0, 70.0),
        y_range: Tuple[float, float] = (-10.0, 10.0),
        resolution: float = 0.25,
    ):
        self.x_range = x_range
        self.y_range = y_range
        self.resolution = resolution

        x_vec = np.arange(x_range[0], x_range[1] + resolution, resolution)
        y_vec = np.arange(y_range[0], y_range[1] + resolution, resolution)

        self.grid_w = len(x_vec)
        self.grid_h = len(y_vec)
        self.X, self.Y = np.meshgrid(x_vec, y_vec)
        self.cost_grid = np.zeros((self.grid_h, self.grid_w), dtype=np.float64)

    def reset(self):
        self.cost_grid.fill(0.0)

    def update_costmap(self, risk_results: List[Any], road_width: float = 7.0):
        self.reset()
        half_w = road_width / 2.0

        # 1. Road Boundary Confinement Cost
        dist_to_left = half_w - self.Y
        dist_to_right = self.Y - (-half_w)

        boundary_cost = 25.0 * (
            np.exp(-np.maximum(0.0, dist_to_left) / 0.5) +
            np.exp(-np.maximum(0.0, dist_to_right) / 0.5)
        )
        off_road = (self.Y > half_w) | (self.Y < -half_w)
        boundary_cost[off_road] = 150.0
        self.cost_grid += boundary_cost

        # 2. Obstacle Risk Potential Fields
        for res in risk_results:
            pos = getattr(res, "position", np.array([20.0, 0.0, 0.0]))
            x_obs, y_obs = pos[0], pos[1]
            risk = res.risk_score
            cname = getattr(res, "object_class", "Car")

            sigma_x = 2.0 + 0.3 * max(0.0, res.factors.r_kin * 10.0)
            sigma_y = 1.8 if cname in ["Two-Wheeler", "Cattle", "Cow"] else 1.2

            peak_cost = 250.0 * risk
            obstacle_pot = peak_cost * np.exp(
                -(((self.X - x_obs)**2) / (2 * sigma_x**2) + ((self.Y - y_obs)**2) / (2 * sigma_y**2))
            )
            self.cost_grid += obstacle_pot

        np.clip(self.cost_grid, 0.0, 300.0, out=self.cost_grid)
