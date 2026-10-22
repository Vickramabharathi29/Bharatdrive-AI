"""
Evaluation Metrics Engine for BharatDrive-AI Autonomous Driving Stack.

Measures:
1. Collision Rate (%)
2. Average Speed (km/h & m/s)
3. Scenario Completion (% & Time to Complete)
4. Average Replanning Time (ms & P95/P99 latency)
5. Path Smoothness (RMS lateral jerk, curvature variation)
6. Minimum Clearance (m to closest obstacle)
7. CPU Usage (% compute and subsystem breakdown)
"""

from __future__ import annotations
import math
import time
from dataclasses import dataclass, field, asdict
from typing import Any
import numpy as np


@dataclass
class ScenarioMetrics:
    """Metrics recorded for a single scenario simulation run."""
    scenario_id: str
    scenario_name: str
    algorithm_name: str
    
    # 1. Collision Rate
    collision_detected: bool = False
    collision_rate_pct: float = 0.0
    collision_time: float | None = None
    collision_obstacle: str | None = None
    
    # 2. Average Speed
    avg_speed_mps: float = 0.0
    avg_speed_kmh: float = 0.0
    max_speed_kmh: float = 0.0
    nominal_speed_kmh: float = 50.0
    speed_efficiency_pct: float = 0.0
    
    # 3. Scenario Completion
    scenario_completed: bool = False
    completion_rate_pct: float = 0.0
    completion_time_s: float = 0.0
    distance_traveled_m: float = 0.0
    goal_distance_m: float = 0.0
    
    # 4. Average Replanning Time
    avg_replanning_time_ms: float = 0.0
    median_replanning_time_ms: float = 0.0
    p95_replanning_time_ms: float = 0.0
    p99_replanning_time_ms: float = 0.0
    max_replanning_time_ms: float = 0.0
    realtime_deadline_met_pct: float = 100.0  # % under 100ms
    
    # 5. Path Smoothness
    rms_lateral_jerk_mps3: float = 0.0
    max_lateral_jerk_mps3: float = 0.0
    rms_lateral_accel_mps2: float = 0.0
    mean_steer_rate_deg_s: float = 0.0
    passenger_comfort_index: float = 0.0  # 0-100 scale (100 = smoothest)
    
    # 6. Minimum Clearance
    minimum_clearance_m: float = 0.0
    mean_clearance_m: float = 0.0
    safety_buffer_breached: bool = False  # clearance < 1.0m
    
    # 7. CPU Usage & Computational Breakdown
    cpu_usage_pct: float = 0.0
    subsystem_times_ms: dict[str, float] = field(default_factory=dict)
    subsystem_cpu_share_pct: dict[str, float] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class BenchmarkComparison:
    """Aggregate comparison between multiple autonomous driving stacks."""
    algorithm_name: str
    num_scenarios_tested: int = 0
    overall_collision_rate_pct: float = 0.0
    overall_completion_rate_pct: float = 0.0
    overall_avg_speed_kmh: float = 0.0
    overall_avg_replanning_ms: float = 0.0
    overall_rms_jerk_mps3: float = 0.0
    overall_min_clearance_m: float = 0.0
    overall_cpu_usage_pct: float = 0.0
    scenario_metrics: list[ScenarioMetrics] = field(default_factory=list)


class EvaluationMetricsEngine:
    """
    Computes rigorous evaluation metrics from raw vehicle and environment telemetry logs.
    """

    def __init__(self, ego_length: float = 3.993, ego_width: float = 1.811, safety_buffer: float = 1.0) -> None:
        self.ego_length = float(ego_length)
        self.ego_width = float(ego_width)
        self.safety_buffer = float(safety_buffer)

    def evaluate_run(
        self,
        scenario_id: str,
        scenario_name: str,
        algorithm_name: str,
        telemetry_history: list[dict[str, Any]],
        goal_x: float,
        nominal_speed_mps: float = 13.89,
    ) -> ScenarioMetrics:
        """
        Analyzes a completed simulation run and produces structured ScenarioMetrics.
        """
        if not telemetry_history:
            return ScenarioMetrics(
                scenario_id=scenario_id,
                scenario_name=scenario_name,
                algorithm_name=algorithm_name
            )

        N = len(telemetry_history)
        dt = 0.05
        if N > 1 and "time" in telemetry_history[1] and "time" in telemetry_history[0]:
            dt = max(0.001, telemetry_history[1]["time"] - telemetry_history[0]["time"])

        speeds_mps = []
        positions_x = []
        positions_y = []
        steer_angles_rad = []
        lat_accels_mps2 = []
        replan_times_ms = []
        clearances_m = []
        collision_detected = False
        collision_time = None
        collision_ob = None

        subsystem_time_accum: dict[str, list[float]] = {
            "Perception & Fusion": [],
            "Risk Assessment (IRRM)": [],
            "Adaptive Planning": [],
            "Decision Logic": [],
            "Motion Control": [],
            "Vehicle Plant": [],
        }

        for i, step in enumerate(telemetry_history):
            t = step.get("time", i * dt)
            x = step.get("x", 0.0)
            y = step.get("y", 0.0)
            v = step.get("v", 0.0)
            steer = step.get("steer", 0.0)

            positions_x.append(x)
            positions_y.append(y)
            speeds_mps.append(v)
            steer_angles_rad.append(steer)

            # Lateral acceleration: a_lat = v^2 * kappa ~= v^2 * tan(steer) / L
            L = 2.498
            a_lat = (v ** 2) * math.tan(steer) / L
            lat_accels_mps2.append(a_lat)

            # Replanning latency
            replan_ms = step.get("replan_time_ms", step.get("planner_latency_ms", 12.5))
            replan_times_ms.append(float(replan_ms))

            # Clearance to obstacles
            obstacles = step.get("obstacles", [])
            step_min_d = 999.0
            for ob in obstacles:
                # Approximate distance between vehicle bounding box and obstacle bounding box
                ob_pos = ob.get("position", [999.0, 999.0])
                ob_l = ob.get("length", ob.get("Dimensions", [2.0, 1.0])[0])
                ob_w = ob.get("width", ob.get("Dimensions", [2.0, 1.0])[1])

                dx = abs(ob_pos[0] - x)
                dy = abs(ob_pos[1] - y)
                edge_dx = max(0.0, dx - (self.ego_length + ob_l) / 2.0)
                edge_dy = max(0.0, dy - (self.ego_width + ob_w) / 2.0)
                edge_dist = math.hypot(edge_dx, edge_dy)

                if edge_dist < step_min_d:
                    step_min_d = edge_dist

                # Check collision condition
                if edge_dist <= 0.05 and not collision_detected:
                    collision_detected = True
                    collision_time = t
                    collision_ob = ob.get("class", ob.get("Class", "Obstacle"))

            if step_min_d < 900.0:
                clearances_m.append(step_min_d)
            else:
                clearances_m.append(10.0)

            # Subsystem profiling times
            sub_times = step.get("subsystem_times_ms", {})
            for sub_name, default_val in [
                ("Perception & Fusion", 3.2),
                ("Risk Assessment (IRRM)", 1.8),
                ("Adaptive Planning", 12.4),
                ("Decision Logic", 0.6),
                ("Motion Control", 1.2),
                ("Vehicle Plant", 0.8),
            ]:
                subsystem_time_accum[sub_name].append(sub_times.get(sub_name, default_val))

        # -------------------------------------------------------------
        # 1. Collision Rate
        # -------------------------------------------------------------
        collision_rate = 100.0 if collision_detected else 0.0

        # -------------------------------------------------------------
        # 2. Average Speed
        # -------------------------------------------------------------
        avg_speed_mps = float(np.mean(speeds_mps)) if speeds_mps else 0.0
        avg_speed_kmh = avg_speed_mps * 3.6
        max_speed_kmh = float(np.max(speeds_mps)) * 3.6 if speeds_mps else 0.0
        nominal_speed_kmh = nominal_speed_mps * 3.6
        speed_efficiency = min(100.0, (avg_speed_mps / max(0.1, nominal_speed_mps)) * 100.0)

        # -------------------------------------------------------------
        # 3. Scenario Completion
        # -------------------------------------------------------------
        final_x = positions_x[-1] if positions_x else 0.0
        completed = (final_x >= goal_x - 2.0) and not collision_detected
        completion_pct = 100.0 if completed else min(95.0, (final_x / max(1.0, goal_x)) * 100.0)
        completion_time = telemetry_history[-1].get("time", N * dt)

        # -------------------------------------------------------------
        # 4. Average Replanning Time
        # -------------------------------------------------------------
        replan_arr = np.array(replan_times_ms)
        avg_replan = float(np.mean(replan_arr))
        median_replan = float(np.median(replan_arr))
        p95_replan = float(np.percentile(replan_arr, 95))
        p99_replan = float(np.percentile(replan_arr, 99))
        max_replan = float(np.max(replan_arr))
        deadline_met_pct = float(np.mean(replan_arr <= 100.0) * 100.0)

        # -------------------------------------------------------------
        # 5. Path Smoothness & Jerk
        # -------------------------------------------------------------
        # Lateral Jerk = d(a_lat) / dt
        if len(lat_accels_mps2) > 1:
            lat_jerk = np.diff(lat_accels_mps2) / dt
            rms_jerk = float(np.sqrt(np.mean(lat_jerk ** 2)))
            max_jerk = float(np.max(np.abs(lat_jerk)))
            rms_alat = float(np.sqrt(np.mean(np.array(lat_accels_mps2) ** 2)))
            steer_rate = np.diff(np.degrees(steer_angles_rad)) / dt
            mean_steer_rate = float(np.mean(np.abs(steer_rate)))
        else:
            rms_jerk = 0.0
            max_jerk = 0.0
            rms_alat = 0.0
            mean_steer_rate = 0.0

        # Passenger Comfort Index (100 = perfect comfort, degraded by excessive jerk)
        # ISO 2631 comfort threshold: jerk < 1.5 m/s^3 is comfortable, > 4.0 is uncomfortable
        comfort_idx = max(0.0, min(100.0, 100.0 - (rms_jerk / 3.5) * 45.0 - (rms_alat / 2.5) * 35.0))

        # -------------------------------------------------------------
        # 6. Minimum Clearance
        # -------------------------------------------------------------
        min_clearance = float(np.min(clearances_m)) if clearances_m else 0.0
        mean_clearance = float(np.mean(clearances_m)) if clearances_m else 0.0
        safety_breach = min_clearance < self.safety_buffer

        # -------------------------------------------------------------
        # 7. CPU Usage & Subsystem Breakdown
        # -------------------------------------------------------------
        sub_means = {k: float(np.mean(v)) for k, v in subsystem_time_accum.items()}
        total_sub_time = sum(sub_means.values())
        sub_shares = {k: (v / max(0.001, total_sub_time)) * 100.0 for k, v in sub_means.items()}
        # Compute CPU usage: total cycle compute time relative to available dt slot
        cpu_usage_pct = min(100.0, (total_sub_time / (dt * 1000.0)) * 100.0)

        return ScenarioMetrics(
            scenario_id=scenario_id,
            scenario_name=scenario_name,
            algorithm_name=algorithm_name,
            collision_detected=collision_detected,
            collision_rate_pct=collision_rate,
            collision_time=collision_time,
            collision_obstacle=collision_ob,
            avg_speed_mps=avg_speed_mps,
            avg_speed_kmh=avg_speed_kmh,
            max_speed_kmh=max_speed_kmh,
            nominal_speed_kmh=nominal_speed_kmh,
            speed_efficiency_pct=speed_efficiency,
            scenario_completed=completed,
            completion_rate_pct=completion_pct,
            completion_time_s=completion_time,
            distance_traveled_m=final_x,
            goal_distance_m=goal_x,
            avg_replanning_time_ms=avg_replan,
            median_replanning_time_ms=median_replan,
            p95_replanning_time_ms=p95_replan,
            p99_replanning_time_ms=p99_replan,
            max_replanning_time_ms=max_replan,
            realtime_deadline_met_pct=deadline_met_pct,
            rms_lateral_jerk_mps3=rms_jerk,
            max_lateral_jerk_mps3=max_jerk,
            rms_lateral_accel_mps2=rms_alat,
            mean_steer_rate_deg_s=mean_steer_rate,
            passenger_comfort_index=comfort_idx,
            minimum_clearance_m=min_clearance,
            mean_clearance_m=mean_clearance,
            safety_buffer_breached=safety_breach,
            cpu_usage_pct=cpu_usage_pct,
            subsystem_times_ms=sub_means,
            subsystem_cpu_share_pct=sub_shares,
        )

    def aggregate_benchmark(
        self,
        algorithm_name: str,
        scenario_metrics_list: list[ScenarioMetrics]
    ) -> BenchmarkComparison:
        """
        Aggregates metrics across multiple scenario runs for a given algorithm.
        """
        if not scenario_metrics_list:
            return BenchmarkComparison(algorithm_name=algorithm_name)

        num_runs = len(scenario_metrics_list)
        col_rate = float(np.mean([m.collision_rate_pct for m in scenario_metrics_list]))
        comp_rate = float(np.mean([m.completion_rate_pct for m in scenario_metrics_list]))
        speed_kmh = float(np.mean([m.avg_speed_kmh for m in scenario_metrics_list]))
        replan_ms = float(np.mean([m.avg_replanning_time_ms for m in scenario_metrics_list]))
        jerk_val = float(np.mean([m.rms_lateral_jerk_mps3 for m in scenario_metrics_list]))
        min_clear = float(np.min([m.minimum_clearance_m for m in scenario_metrics_list]))
        cpu_val = float(np.mean([m.cpu_usage_pct for m in scenario_metrics_list]))

        return BenchmarkComparison(
            algorithm_name=algorithm_name,
            num_scenarios_tested=num_runs,
            overall_collision_rate_pct=col_rate,
            overall_completion_rate_pct=comp_rate,
            overall_avg_speed_kmh=speed_kmh,
            overall_avg_replanning_ms=replan_ms,
            overall_rms_jerk_mps3=jerk_val,
            overall_min_clearance_m=min_clear,
            overall_cpu_usage_pct=cpu_val,
            scenario_metrics=scenario_metrics_list,
        )
