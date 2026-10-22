"""
Unit Tests for Adaptive Path Planning Module (Hybrid A* + PPO + DWA).
BharatDrive-AI - MathWorks Autonomous Driving Hackathon.
"""

import math
import numpy as np
import pytest

from bharatdrive.planning.hybrid_astar import HybridAStarPlanner
from bharatdrive.planning.dwa import DynamicWindowApproach
from bharatdrive.planning.ppo_policy import PPOAgentNegotiator
from bharatdrive.planning.adaptive_planner import AdaptivePlannerCoordinator


class TestHybridAStar:
    """Tests for Level 1 Kinematic Hybrid A* Corridor Planner."""

    def test_curvature_and_road_bounds(self):
        planner = HybridAStarPlanner(wheelbase=2.498, max_steer=0.55)
        start = [0.0, 0.0, 0.0]
        goal = [40.0, 1.5, 0.0]
        road_width = 7.0

        path = planner.plan(start, goal, obstacle_boxes=[], road_width=road_width)
        assert len(path) >= 10
        assert path.shape[1] == 3

        # Path starts near start and ends near goal
        assert math.isclose(path[0, 0], 0.0, abs_tol=0.1)
        assert math.isclose(path[0, 1], 0.0, abs_tol=0.1)
        assert math.isclose(path[-1, 0], 40.0, abs_tol=0.5)

        # Path strictly respects road boundaries (half_width = 3.5m)
        half_w = road_width / 2.0
        assert np.all(np.abs(path[:, 1]) <= half_w)

    def test_obstacle_deflection(self):
        planner = HybridAStarPlanner()
        start = [0.0, 0.0, 0.0]
        goal = [30.0, 0.0, 0.0]
        # Obstacle directly on centerline at x = 15.0m
        obstacles = [[15.0, 0.0, 2.0, 1.2]]

        path = planner.plan(start, goal, obstacle_boxes=obstacles, road_width=7.0)

        # Waypoints near x = 15m must have non-zero lateral deflection
        idx_near_obs = np.where((path[:, 0] >= 13.0) & (path[:, 0] <= 17.0))[0]
        assert len(idx_near_obs) > 0
        deflections = np.abs(path[idx_near_obs, 1])
        assert np.max(deflections) >= 1.0, "Path should deflect around obstacle on centerline"


class TestPPOPolicy:
    """Tests for Level 2 PPO Tactical Negotiation Policy."""

    def test_action_bounds(self):
        ppo = PPOAgentNegotiator()
        dummy_obs = np.zeros(12)
        action, state_value = ppo.evaluate_policy(dummy_obs)

        assert -1.50 <= action["nudge_offset"] <= 1.50
        assert 0.25 <= action["speed_factor"] <= 1.15
        assert 0.30 <= action["risk_weight"] <= 2.50
        assert isinstance(state_value, float)

    def test_observation_extraction(self):
        ppo = PPOAgentNegotiator()
        ego_state = [10.0, -0.5, 12.0, 0.05, 0.01]
        threat = {
            "distance": 25.0,
            "relative_speed": 4.5,
            "risk_score": 0.65,
            "position": [35.0, 0.2],
        }

        obs = ppo.extract_observation(
            ego_state=ego_state,
            threat_obstacle=threat,
            road_width=7.0,
            weather_mu=0.80,
            traffic_density=3,
        )

        assert len(obs) == 12
        assert np.all(np.isfinite(obs))
        assert 0.0 <= obs[0] <= 1.0  # Speed norm
        assert 0.0 <= obs[5] <= 1.0  # Distance norm
        assert math.isclose(obs[7], 0.65, abs_tol=1e-3)  # Risk score


class TestDynamicWindowApproach:
    """Tests for Level 3 Dynamic Window Approach (DWA)."""

    def test_dynamic_window_acceleration_limits(self):
        dwa = DynamicWindowApproach(max_speed=20.0, max_accel=3.5, max_decel=6.5)
        # Ego moving at 10.0 m/s
        current_state = [0.0, 0.0, 10.0, 0.0, 0.0]
        target_goal = [20.0, 0.0]

        best_cmd, best_traj = dwa.plan(current_state, target_goal, obstacle_list=[])

        v_cmd, w_cmd = best_cmd[0], best_cmd[1]
        dt = dwa.dt
        # Commanded velocity must be reachable within acceleration/braking limits
        min_v = 10.0 - 6.5 * dt - 1e-4
        max_v = 10.0 + 3.5 * dt + 1e-4
        assert min_v <= v_cmd <= max_v, f"v_cmd {v_cmd} outside dynamic window [{min_v}, {max_v}]"

    def test_stopping_distance_admissibility(self):
        dwa = DynamicWindowApproach(max_speed=15.0, max_decel=6.0)
        # Obstacle immediately ahead (4.0m)
        current_state = [0.0, 0.0, 8.0, 0.0, 0.0]
        target_goal = [20.0, 0.0]
        obstacles = [{"position": [4.0, 0.0]}]

        best_cmd, _ = dwa.plan(current_state, target_goal, obstacle_list=obstacles)
        # Stopping distance for 8 m/s with 6 m/s^2 decel is 8^2 / 12 = 5.33m > 4m
        # DWA must decelerate strongly towards safe speed
        assert best_cmd[0] < current_state[2], "DWA should decelerate when obstacle is within stopping distance"


class TestAdaptivePlannerCoordinator:
    """Tests for continuous replanning coordinator combining Hybrid A*, PPO, and DWA."""

    def test_continuous_replan_cycle(self):
        coordinator = AdaptivePlannerCoordinator(target_speed=13.89)
        ego_state = [0.0, 0.0, 10.0, 0.0, 0.0]
        obstacles = [
            {"position": [20.0, 0.5], "dimensions": [2.0, 1.2], "risk_score": 0.45},
            {"position": [35.0, -1.5], "dimensions": [2.5, 1.3], "risk_score": 0.30},
        ]
        risks = [
            {"RiskScore": 0.45, "Distance": 20.0, "RelativeSpeed": 5.0},
            {"RiskScore": 0.30, "Distance": 35.0, "RelativeSpeed": 2.0},
        ]

        plan_out = coordinator.replan_cycle(
            ego_state=ego_state,
            detected_obstacles=obstacles,
            risk_results=risks,
            road_width=7.0,
        )

        assert "command" in plan_out
        assert "local_trajectory" in plan_out
        assert "tactical_nudge" in plan_out
        assert -1.5 <= plan_out["tactical_nudge"] <= 1.5
        assert 0.25 <= plan_out["speed_factor"] <= 1.15
        assert plan_out["local_trajectory"].shape[1] == 3

    def test_emergency_override_trigger(self):
        coordinator = AdaptivePlannerCoordinator(target_speed=13.89)
        ego_state = [0.0, 0.0, 12.0, 0.0, 0.0]
        # Critical threat: sudden animal/pedestrian jump-in
        critical_obs = [{"position": [5.0, 0.0], "dimensions": [1.5, 1.0], "risk_score": 0.92}]
        critical_risk = [{"RiskScore": 0.92, "Distance": 5.0, "RelativeSpeed": 12.0}]

        plan_out = coordinator.replan_cycle(
            ego_state=ego_state,
            detected_obstacles=critical_obs,
            risk_results=critical_risk,
            road_width=7.0,
        )

        # Under critical risk >= 0.85, DWA max speed must collapse to zero
        assert coordinator.dwa.max_speed == 0.0
        assert plan_out["command"]["target_velocity"] <= ego_state[2]
