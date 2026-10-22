"""
Adaptive Planner Coordinator: Continuous Replanning Engine.
BharatDrive-AI - MathWorks Autonomous Driving Hackathon.

Fuses Hybrid A*, Dynamic Window Approach (DWA), and PPO Reinforcement Learning
into a unified, high-frequency continuous replanning architecture (10-20 Hz).
"""

from __future__ import annotations
import math
from typing import Any
import numpy as np

from bharatdrive.planning.hybrid_astar import HybridAStarPlanner
from bharatdrive.planning.dwa import DynamicWindowApproach
from bharatdrive.planning.ppo_policy import PPOAgentNegotiator


class AdaptivePlannerCoordinator:
    """
    Coordinates continuous replanning across global corridor, tactical negotiation, and local dynamic window.
    """

    def __init__(self, target_speed: float = 13.89) -> None:
        self.target_speed = float(target_speed)
        self.replanning_rate_hz = 10.0  # 100ms cycle
        self.dt = 1.0 / self.replanning_rate_hz

        self.hybrid_astar = HybridAStarPlanner()
        self.dwa = DynamicWindowApproach(max_speed=target_speed)
        self.ppo_agent = PPOAgentNegotiator()
        self.last_trajectory: np.ndarray | None = None

    def replan_cycle(
        self,
        ego_state: np.ndarray | list[float],
        detected_obstacles: list[Any] | None = None,
        risk_results: list[Any] | None = None,
        road_width: float = 7.0,
        weather: dict[str, float] | None = None,
        global_goal: np.ndarray | list[float] | None = None,
    ) -> dict[str, Any]:
        """
        Executes a single continuous replanning iteration (10 Hz).

        Args:
            ego_state: [x, y, v, yaw, yaw_rate]
            detected_obstacles: list of detected obstacle objects/dicts
            risk_results: list of risk evaluation results
            road_width: total drivable road width (m)
            weather: dict with 'friction_mu' and 'visibility'
            global_goal: [x_goal, y_goal, yaw_goal]

        Returns:
            plan_output: dict containing:
                - 'command': {'target_velocity': float, 'target_yaw_rate': float}
                - 'local_trajectory': np.ndarray (N, 3)
                - 'tactical_nudge': float
                - 'speed_factor': float
                - 'dwa_risk_weight': float
                - 'state_value': float
                - 'guide_path': np.ndarray
        """
        ego = np.asarray(ego_state, dtype=np.float64)
        obstacles = detected_obstacles or []
        risks = risk_results or []
        w_info = weather or {"friction_mu": 0.85, "visibility": 1.0}

        if global_goal is None:
            goal = np.array([ego[0] + 60.0, 0.0, 0.0], dtype=np.float64)
        else:
            goal = np.asarray(global_goal, dtype=np.float64)

        # 1. Identify primary threatening obstacle from risk assessments
        highest_risk_obs = None
        max_risk = 0.0
        for r in risks:
            if isinstance(r, dict):
                score = r.get("risk_score", r.get("RiskScore", 0.0))
            else:
                score = getattr(r, "RiskScore", getattr(r, "risk_score", 0.0))
            if score > max_risk:
                max_risk = float(score)
                highest_risk_obs = r

        # 2. Level 2: Evaluate PPO Tactical Policy
        traffic_density = len(obstacles)
        obs_vec = self.ppo_agent.extract_observation(
            ego, highest_risk_obs, road_width, w_info.get("friction_mu", 0.85), traffic_density
        )
        ppo_action, state_value = self.ppo_agent.evaluate_policy(obs_vec)

        # 3. Level 1: Modulate Mid-Term Goal using PPO Tactical Nudge
        nudged_goal = goal.copy()
        half_w = (road_width / 2.0) - 0.6
        nudged_goal[1] = np.clip(goal[1] + ppo_action["nudge_offset"], -half_w, half_w)

        # Format obstacle bounding boxes for Hybrid A*
        obs_boxes = []
        for ob in obstacles:
            if isinstance(ob, dict):
                pos = ob.get("position", ob.get("Position", [0.0, 0.0]))
                dim = ob.get("dimensions", ob.get("Dimensions", [2.0, 1.5]))
            else:
                pos = getattr(ob, "Position", getattr(ob, "position", [0.0, 0.0]))
                dim = getattr(ob, "Dimensions", getattr(ob, "dimensions", [2.0, 1.5]))
            obs_boxes.append([pos[0], pos[1], dim[0], dim[1]])

        hybrid_guide = self.hybrid_astar.plan(
            start_state=ego[:3],
            goal_state=nudged_goal,
            obstacle_boxes=obs_boxes,
            road_width=road_width,
        )

        # 4. Level 3: Dynamically Configure DWA from PPO Directives
        lookahead_idx = min(15, len(hybrid_guide) - 1)
        target_pt = hybrid_guide[lookahead_idx, :2]

        self.dwa.max_speed = self.target_speed * ppo_action["speed_factor"]
        self.dwa.weight_risk_cost = ppo_action["risk_weight"]

        # Emergency override if risk exceeds critical threshold
        if max_risk >= 0.85:
            self.dwa.max_speed = 0.0  # Force braking rollout
            self.dwa.weight_clearance = 0.80

        # Execute DWA local velocity sampling
        best_cmd, best_traj = self.dwa.plan(
            current_state=ego,
            target_goal=target_pt,
            obstacle_list=obstacles,
            costmap_engine=None,
        )

        plan_output = {
            "command": {
                "target_velocity": float(best_cmd[0]),
                "target_yaw_rate": float(best_cmd[1]),
            },
            "local_trajectory": best_traj,
            "tactical_nudge": ppo_action["nudge_offset"],
            "speed_factor": ppo_action["speed_factor"],
            "dwa_risk_weight": ppo_action["risk_weight"],
            "state_value": state_value,
            "guide_path": hybrid_guide,
        }

        self.last_trajectory = best_traj
        return plan_output
