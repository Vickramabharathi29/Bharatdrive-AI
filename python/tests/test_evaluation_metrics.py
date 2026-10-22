"""
Unit tests for BharatDrive-AI Evaluation Metrics Engine and Graph Generator.
"""

from __future__ import annotations
import os
import tempfile
import numpy as np
import pytest

from bharatdrive.evaluation.metrics_engine import (
    EvaluationMetricsEngine,
    ScenarioMetrics,
    BenchmarkComparison,
)
from bharatdrive.evaluation.graph_generator import GraphGenerator


def test_metrics_engine_initialization():
    engine = EvaluationMetricsEngine(ego_length=4.0, ego_width=1.8, safety_buffer=1.0)
    assert engine.ego_length == 4.0
    assert engine.ego_width == 1.8
    assert engine.safety_buffer == 1.0


def test_synthetic_telemetry_evaluation():
    engine = EvaluationMetricsEngine()
    
    # Generate 5-second synthetic telemetry history
    n_steps = 100
    telemetry_history = []
    
    for i in range(n_steps):
        t = i * 0.05
        step = {
            "time": t,
            "x": i * 0.5,
            "y": 0.0,
            "v": 10.0,  # 10 m/s = 36 km/h
            "yaw": 0.0,
            "steer": np.sin(t) * 0.05,
            "collision": False,
            "replan_time_ms": 11.5,
            "subsystem_times": {
                "Perception & Fusion": 6.5,
                "Risk Assessment (IRRM)": 3.8,
                "Adaptive Planning": 12.0,
                "Decision Logic": 1.5,
                "Motion Control": 1.2,
                "Vehicle Plant": 0.5,
            },
            "obstacles": [
                {"id": 1, "type": "auto_rickshaw", "x": 20.0, "y": 3.0, "length": 2.6, "width": 1.3}
            ]
        }
        telemetry_history.append(step)
    
    metrics = engine.evaluate_run(
        scenario_id="scenario_test",
        scenario_name="Synthetic Test",
        algorithm_name="BharatDrive-AI",
        telemetry_history=telemetry_history,
        goal_x=50.0,
        nominal_speed_mps=13.89,
    )
    
    assert isinstance(metrics, ScenarioMetrics)
    assert not metrics.collision_detected
    assert metrics.collision_rate_pct == 0.0
    assert pytest.approx(metrics.avg_speed_kmh, 0.1) == 36.0
    assert metrics.scenario_completed is True
    assert metrics.completion_rate_pct == 100.0
    assert metrics.avg_replanning_time_ms > 0.0
    assert metrics.rms_lateral_jerk_mps3 >= 0.0
    assert metrics.minimum_clearance_m > 0.0
    assert metrics.cpu_usage_pct > 0.0
    assert "Adaptive Planning" in metrics.subsystem_times_ms


def test_aggregate_benchmarks():
    engine = EvaluationMetricsEngine()
    
    m1 = ScenarioMetrics(
        scenario_id="s1",
        scenario_name="Scenario 1",
        algorithm_name="TestStack",
        collision_detected=False,
        collision_rate_pct=0.0,
        avg_speed_kmh=30.0,
        scenario_completed=True,
        completion_rate_pct=100.0,
        avg_replanning_time_ms=10.0,
        rms_lateral_jerk_mps3=0.8,
        minimum_clearance_m=1.5,
        cpu_usage_pct=40.0
    )
    m2 = ScenarioMetrics(
        scenario_id="s2",
        scenario_name="Scenario 2",
        algorithm_name="TestStack",
        collision_detected=True,
        collision_rate_pct=100.0,
        avg_speed_kmh=20.0,
        scenario_completed=False,
        completion_rate_pct=50.0,
        avg_replanning_time_ms=12.0,
        rms_lateral_jerk_mps3=1.2,
        minimum_clearance_m=0.0,
        cpu_usage_pct=42.0
    )
    
    comp = engine.aggregate_benchmark("TestStack", [m1, m2])
    assert isinstance(comp, BenchmarkComparison)
    assert comp.num_scenarios_tested == 2
    assert comp.overall_collision_rate_pct == 50.0
    assert comp.overall_completion_rate_pct == 75.0
    assert pytest.approx(comp.overall_avg_speed_kmh, 0.1) == 25.0
    assert pytest.approx(comp.overall_avg_replanning_ms, 0.1) == 11.0
    assert pytest.approx(comp.overall_cpu_usage_pct, 0.1) == 41.0


def test_graph_generator_renders_files():
    with tempfile.TemporaryDirectory() as tmpdir:
        generator = GraphGenerator(output_dir=tmpdir)
        
        # Create mock benchmarks
        m = ScenarioMetrics(
            scenario_id="s1",
            scenario_name="Test Scene",
            algorithm_name="BharatDrive-AI",
            collision_detected=False,
            collision_rate_pct=0.0,
            avg_speed_kmh=35.0,
            scenario_completed=True,
            completion_rate_pct=100.0,
            avg_replanning_time_ms=12.0,
            p95_replanning_time_ms=16.0,
            p99_replanning_time_ms=19.0,
            rms_lateral_jerk_mps3=0.85,
            passenger_comfort_index=93.0,
            minimum_clearance_m=1.45,
            cpu_usage_pct=38.0,
            subsystem_times_ms={
                "Perception": 6.5,
                "Fusion": 2.2,
                "IRRM Risk": 3.8,
                "Stateflow": 1.5,
                "Hybrid A*": 12.0,
                "Controller": 1.2
            }
        )
        
        benchmarks = {
            "BharatDrive-AI": BenchmarkComparison(
                algorithm_name="BharatDrive-AI",
                num_scenarios_tested=1,
                overall_collision_rate_pct=0.0,
                overall_completion_rate_pct=100.0,
                overall_avg_speed_kmh=35.0,
                overall_avg_replanning_ms=12.0,
                overall_rms_jerk_mps3=0.85,
                overall_min_clearance_m=1.45,
                overall_cpu_usage_pct=38.0,
                scenario_metrics=[m]
            )
        }
        
        generated_files = generator.generate_all_graphs(benchmarks)
        assert len(generated_files) > 0
        for fpath in generated_files:
            assert os.path.exists(fpath)
            assert os.path.getsize(fpath) > 0
