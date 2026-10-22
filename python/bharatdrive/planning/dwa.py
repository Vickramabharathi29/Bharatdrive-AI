"""
Dynamic Window Approach (DWA) Local Trajectory Planner.
BharatDrive-AI - MathWorks Autonomous Driving Hackathon.

Operates directly in vehicle velocity space (v, omega), sampling achievable trajectories
within powertrain acceleration and braking limits, subject to dynamic risk modulation.
"""

from __future__ import annotations
import math
from typing import Any
import numpy as np


class DynamicWindowApproach:
    """
    Dynamic Window Approach for high-frequency continuous local replanning.
    """

    def __init__(
        self,
        max_speed: float = 22.22,
        max_accel: float = 3.5,
        max_decel: float = 6.5,
    ) -> None:
        self.max_speed = float(max_speed)
        self.min_speed = 0.0
        self.max_yaw_rate = 0.85
        self.max_accel = float(max_accel)
        self.max_decel = float(max_decel)
        self.max_yaw_accel = 1.50
        self.predict_time = 2.0
        self.dt = 0.1

        self.v_samples = 15
        self.omega_samples = 21

        self.weight_heading = 0.15
        self.weight_clearance = 0.40
        self.weight_velocity = 0.25
        self.weight_risk_cost = 0.50

    def plan(
        self,
        current_state: np.ndarray | list[float],
        target_goal: np.ndarray | list[float],
        obstacle_list: list[Any] | None = None,
        costmap_engine: Any | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Selects optimal [v, omega] maximizing the DWA objective over the dynamic window.

        Args:
            current_state: [x, y, v, yaw, yaw_rate]
            target_goal: [x_goal, y_goal]
            obstacle_list: list of obstacles with position/Position
            costmap_engine: optional costmap

        Returns:
            best_cmd: [v, omega]
            best_traj: np.ndarray of shape (N, 3) representing [x, y, yaw]
        """
        curr = np.asarray(current_state, dtype=np.float64)
        curr_x = curr[0]
        curr_y = curr[1]
        curr_v = curr[2]
        curr_yaw = curr[3] if len(curr) > 3 else 0.0
        curr_omega = curr[4] if len(curr) > 4 else 0.0

        target = np.asarray(target_goal, dtype=np.float64)[:2]
        obstacles = obstacle_list or []

        # 1. Dynamic Window: Achievable velocities within time step dt
        v_min_dyn = max(self.min_speed, curr_v - self.max_decel * self.dt)
        v_max_dyn = min(self.max_speed, curr_v + self.max_accel * self.dt)
        w_min_dyn = max(-self.max_yaw_rate, curr_omega - self.max_yaw_accel * self.dt)
        w_max_dyn = min(self.max_yaw_rate, curr_omega + self.max_yaw_accel * self.dt)

        v_candidates = np.linspace(v_min_dyn, v_max_dyn, self.v_samples)
        w_candidates = np.linspace(w_min_dyn, w_max_dyn, self.omega_samples)

        best_score = -float("inf")
        best_cmd = np.array([0.0, 0.0], dtype=np.float64)
        num_steps = int(round(self.predict_time / self.dt))
        best_traj = np.tile([curr_x, curr_y, curr_yaw], (num_steps, 1))

        for v in v_candidates:
            for w in w_candidates:
                # Forward rollout arc
                traj = np.zeros((num_steps, 3), dtype=np.float64)
                sim_x, sim_y, sim_yaw = curr_x, curr_y, curr_yaw
                min_dist_to_obs = float("inf")

                for step in range(num_steps):
                    sim_yaw += w * self.dt
                    sim_x += v * math.cos(sim_yaw) * self.dt
                    sim_y += v * math.sin(sim_yaw) * self.dt
                    traj[step] = [sim_x, sim_y, sim_yaw]

                    for ob in obstacles:
                        if isinstance(ob, dict):
                            ox, oy = ob.get("position", ob.get("Position", [0.0, 0.0]))[:2]
                        else:
                            pos = getattr(ob, "Position", getattr(ob, "position", [0.0, 0.0]))
                            ox, oy = pos[0], pos[1]
                        dist = math.hypot(sim_x - ox, sim_y - oy)
                        if dist < min_dist_to_obs:
                            min_dist_to_obs = dist

                # Admissibility check: can stop before colliding?
                stopping_dist = (v**2) / (2.0 * self.max_decel)
                if min_dist_to_obs <= stopping_dist and v > 1.0:
                    continue  # Inadmissible trajectory

                # 1. Heading Alignment Objective
                final_pt = traj[-1, :2]
                goal_heading = math.atan2(target[1] - final_pt[1], target[0] - final_pt[0])
                heading_err = abs(math.atan2(math.sin(goal_heading - traj[-1, 2]), math.cos(goal_heading - traj[-1, 2])))
                score_heading = (math.pi - heading_err) / math.pi

                # 2. Clearance Objective
                score_clearance = min(1.0, min_dist_to_obs / 8.0)

                # 3. Velocity Progress Objective
                score_velocity = v / max(1.0, self.max_speed)

                # 4. Risk Cost Penalty
                score_risk = min(1.0, 1.0 / (min_dist_to_obs + 0.1))

                total_score = (
                    self.weight_heading * score_heading
                    + self.weight_clearance * score_clearance
                    + self.weight_velocity * score_velocity
                    - self.weight_risk_cost * score_risk
                )

                if total_score > best_score:
                    best_score = total_score
                    best_cmd = np.array([v, w], dtype=np.float64)
                    best_traj = traj

        return best_cmd, best_traj
