"""Configuration management module for BharatDrive-AI.

Provides type-safe configuration classes and YAML/JSON loader interfaces
for vehicle parameters, Indian road specifications, and trajectory planner parameters.
"""

from __future__ import annotations
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Dict, Any, Optional
import json
import yaml


@dataclass(frozen=True)
class VehicleDimensions:
    """Vehicle geometric dimensions in meters."""
    length: float = 3.993
    width: float = 1.811
    height: float = 1.606
    wheelbase: float = 2.498
    front_overhang: float = 0.780
    rear_overhang: float = 0.715
    track_width: float = 1.520


@dataclass(frozen=True)
class VehicleMassProperties:
    """Vehicle inertial and mass properties."""
    mass: float = 1400.0            # kg
    inertia_z: float = 2250.0       # kg*m^2
    cg_to_front_axle: float = 1.150 # lf (m)
    cg_to_rear_axle: float = 1.348  # lr (m)


@dataclass(frozen=True)
class VehicleKinematicLimits:
    """Kinematic, comfort, and safety saturation boundaries."""
    max_steer_angle_rad: float = 0.55
    max_steer_rate_rad_s: float = 0.60
    max_velocity_mps: float = 22.22
    min_velocity_mps: float = 0.0
    max_accel_mps2: float = 3.5
    max_decel_mps2: float = -6.5
    comfort_accel_mps2: float = 1.8
    comfort_decel_mps2: float = -2.5
    max_lat_accel_mps2: float = 3.5
    comfort_lat_accel_mps2: float = 2.0
    max_lat_jerk_mps3: float = 2.0
    max_lon_jerk_mps3: float = 2.5


@dataclass(frozen=True)
class VehicleTireParameters:
    """Linear and dynamic tire characteristics."""
    cornering_stiffness_front: float = 80000.0  # N/rad
    cornering_stiffness_rear: float = 85000.0   # N/rad
    friction_coefficient: float = 0.85
    friction_coefficient_wet: float = 0.50


@dataclass(frozen=True)
class VehicleConfig:
    """Comprehensive vehicle configuration parameter aggregate."""
    name: str = "BharatDrive_Ego_EV"
    dimensions: VehicleDimensions = field(default_factory=VehicleDimensions)
    mass: VehicleMassProperties = field(default_factory=VehicleMassProperties)
    limits: VehicleKinematicLimits = field(default_factory=VehicleKinematicLimits)
    tires: VehicleTireParameters = field(default_factory=VehicleTireParameters)

    @classmethod
    def from_yaml(cls, path: str | Path) -> VehicleConfig:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)["vehicle"]
        return cls(
            name=data.get("name", "BharatDrive_Ego_EV"),
            dimensions=VehicleDimensions(**data["dimensions"]),
            mass=VehicleMassProperties(**data["mass_properties"]),
            limits=VehicleKinematicLimits(**data["kinematic_limits"]),
            tires=VehicleTireParameters(**data["tire_parameters"]),
        )

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2)


@dataclass(frozen=True)
class RoadSpecsConfig:
    """Indian road types and hazard thresholds."""
    data: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_yaml(cls, path: str | Path) -> RoadSpecsConfig:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        return cls(data=data)

    def get_road_width(self, road_type: str = "two_lane_undivided") -> float:
        return float(self.data.get("road_types", {}).get(road_type, {}).get("carriageway_width_m", 7.0))

    def get_agent_buffer(self, agent_type: str) -> float:
        return float(self.data.get("heterogeneous_agents", {}).get(agent_type, {}).get("lateral_safety_buffer_m", 1.2))


@dataclass(frozen=True)
class PlannerConfig:
    """Frenet trajectory planner sampling intervals and optimization weights."""
    d_step_m: float = 0.25
    max_d_offset_m: float = 2.25
    time_horizon_min_s: float = 2.0
    time_horizon_max_s: float = 4.5
    time_horizon_step_s: float = 0.5
    delta_t_s: float = 0.1
    speed_sampling_step_mps: float = 1.39
    num_speed_samples: int = 5

    # Cost weights
    jerk_longitudinal: float = 0.15
    jerk_lateral: float = 0.20
    acceleration_longitudinal: float = 0.10
    acceleration_lateral: float = 0.25
    time_efficiency: float = 0.10
    target_speed_deviation: float = 1.20
    corridor_center_offset: float = 0.80
    static_obstacle_proximity: float = 80.0
    dynamic_obstacle_proximity: float = 120.0
    pothole_penalty: float = 50.0
    boundary_violation: float = 500.0

    # Margins
    static_obstacle_buffer_m: float = 0.60
    dynamic_obstacle_buffer_m: float = 1.00
    pothole_clearance_m: float = 0.40
    road_edge_margin_m: float = 0.35

    @classmethod
    def from_yaml(cls, path: str | Path) -> PlannerConfig:
        with open(path, "r", encoding="utf-8") as f:
            raw = yaml.safe_load(f)
        fs = raw.get("frenet_sampling", {})
        cw = raw.get("cost_weights", {})
        sm = raw.get("safety_margins", {})
        return cls(
            d_step_m=float(fs.get("d_step_m", 0.25)),
            max_d_offset_m=float(fs.get("max_d_offset_m", 2.25)),
            time_horizon_min_s=float(fs.get("time_horizon_min_s", 2.0)),
            time_horizon_max_s=float(fs.get("time_horizon_max_s", 4.5)),
            time_horizon_step_s=float(fs.get("time_horizon_step_s", 0.5)),
            delta_t_s=float(fs.get("delta_t_s", 0.1)),
            speed_sampling_step_mps=float(fs.get("speed_sampling_step_mps", 1.39)),
            num_speed_samples=int(fs.get("num_speed_samples", 5)),
            jerk_longitudinal=float(cw.get("jerk_longitudinal", 0.15)),
            jerk_lateral=float(cw.get("jerk_lateral", 0.20)),
            acceleration_longitudinal=float(cw.get("acceleration_longitudinal", 0.10)),
            acceleration_lateral=float(cw.get("acceleration_lateral", 0.25)),
            time_efficiency=float(cw.get("time_efficiency", 0.10)),
            target_speed_deviation=float(cw.get("target_speed_deviation", 1.20)),
            corridor_center_offset=float(cw.get("corridor_center_offset", 0.80)),
            static_obstacle_proximity=float(cw.get("static_obstacle_proximity", 80.0)),
            dynamic_obstacle_proximity=float(cw.get("dynamic_obstacle_proximity", 120.0)),
            pothole_penalty=float(cw.get("pothole_penalty", 50.0)),
            boundary_violation=float(cw.get("boundary_violation", 500.0)),
            static_obstacle_buffer_m=float(sm.get("static_obstacle_buffer_m", 0.60)),
            dynamic_obstacle_buffer_m=float(sm.get("dynamic_obstacle_buffer_m", 1.00)),
            pothole_clearance_m=float(sm.get("pothole_clearance_m", 0.40)),
            road_edge_margin_m=float(sm.get("road_edge_margin_m", 0.35)),
        )


def get_default_config_dir() -> Path:
    """Resolves project root config directory reliably."""
    current = Path(__file__).resolve().parent
    for candidate in [current.parent.parent / "config", current.parent / "config", Path("config")]:
        if candidate.exists() and (candidate / "vehicle_params.yaml").exists():
            return candidate
    return Path("config")


def load_all_configs(config_dir: Optional[str | Path] = None) -> tuple[VehicleConfig, RoadSpecsConfig, PlannerConfig]:
    """Loads all 3 YAML configs with automatic fallback to defaults if files are absent."""
    base = Path(config_dir) if config_dir else get_default_config_dir()
    veh_file = base / "vehicle_params.yaml"
    road_file = base / "indian_road_specs.yaml"
    plan_file = base / "planner_config.yaml"

    vehicle_cfg = VehicleConfig.from_yaml(veh_file) if veh_file.exists() else VehicleConfig()
    road_cfg = RoadSpecsConfig.from_yaml(road_file) if road_file.exists() else RoadSpecsConfig()
    planner_cfg = PlannerConfig.from_yaml(plan_file) if plan_file.exists() else PlannerConfig()

    return vehicle_cfg, road_cfg, planner_cfg
