"""Unit tests for configuration loaders and parameter schemas."""

import json
from pathlib import Path
import pytest
from bharatdrive.config import (
    VehicleConfig,
    VehicleDimensions,
    RoadSpecsConfig,
    PlannerConfig,
    load_all_configs,
    get_default_config_dir,
)


def test_vehicle_config_defaults():
    cfg = VehicleConfig()
    assert cfg.name == "BharatDrive_Ego_EV"
    assert cfg.dimensions.wheelbase == pytest.approx(2.498, rel=1e-3)
    assert cfg.mass.mass == 1400.0
    assert cfg.limits.max_velocity_mps == pytest.approx(22.22, rel=1e-3)
    assert cfg.limits.max_steer_angle_rad == pytest.approx(0.55, rel=1e-3)


def test_vehicle_config_yaml_loading():
    config_dir = get_default_config_dir()
    yaml_file = config_dir / "vehicle_params.yaml"
    assert yaml_file.exists(), f"Missing config file at {yaml_file}"

    cfg = VehicleConfig.from_yaml(yaml_file)
    assert cfg.name == "BharatDrive_Ego_EV"
    assert cfg.dimensions.length == pytest.approx(3.993, rel=1e-3)
    assert cfg.mass.cg_to_front_axle == pytest.approx(1.150, rel=1e-3)
    assert cfg.tires.cornering_stiffness_front == 80000.0


def test_vehicle_config_serialization():
    cfg = VehicleConfig()
    d = cfg.to_dict()
    assert "dimensions" in d
    assert "mass" in d
    json_str = cfg.to_json()
    parsed = json.loads(json_str)
    assert parsed["name"] == "BharatDrive_Ego_EV"


def test_road_specs_yaml():
    config_dir = get_default_config_dir()
    yaml_file = config_dir / "indian_road_specs.yaml"
    assert yaml_file.exists()

    road_cfg = RoadSpecsConfig.from_yaml(yaml_file)
    assert road_cfg.get_road_width("two_lane_undivided") == 7.00
    assert road_cfg.get_road_width("single_lane") == 3.75
    assert road_cfg.get_agent_buffer("two_wheeler") == 1.50
    assert road_cfg.get_agent_buffer("stray_animal") == 1.80


def test_planner_config_yaml():
    config_dir = get_default_config_dir()
    yaml_file = config_dir / "planner_config.yaml"
    assert yaml_file.exists()

    planner_cfg = PlannerConfig.from_yaml(yaml_file)
    assert planner_cfg.d_step_m == 0.25
    assert planner_cfg.static_obstacle_proximity == 80.0
    assert planner_cfg.boundary_violation == 500.0


def test_load_all_configs():
    veh, road, plan = load_all_configs()
    assert isinstance(veh, VehicleConfig)
    assert isinstance(road, RoadSpecsConfig)
    assert isinstance(plan, PlannerConfig)
