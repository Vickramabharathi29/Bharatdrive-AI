"""BharatDrive-AI: Adaptive Path Planning and Collision Avoidance
for Autonomous Vehicles on Unstructured Indian Roads.

Module 1: Core Mathematical Foundations, Kinematic Models & Workspace Architecture.
"""

__version__ = "1.0.0"
__author__ = "BharatDrive-AI Team"

from bharatdrive.config import VehicleConfig, RoadSpecsConfig, PlannerConfig, load_all_configs
from bharatdrive.models.state import VehicleState, FrenetState, VehicleDimensions
from bharatdrive.models.bicycle_model import KinematicBicycleModel, DynamicBicycleModel
from bharatdrive.geometry.transforms import (
    cartesian_to_body,
    body_to_cartesian,
    compute_vehicle_corners,
    check_polygon_collision_sat,
)
from bharatdrive.geometry.frenet import CubicSpline2D, cartesian_to_frenet, frenet_to_cartesian

__all__ = [
    "VehicleConfig",
    "RoadSpecsConfig",
    "PlannerConfig",
    "load_all_configs",
    "VehicleState",
    "FrenetState",
    "VehicleDimensions",
    "KinematicBicycleModel",
    "DynamicBicycleModel",
    "cartesian_to_body",
    "body_to_cartesian",
    "compute_vehicle_corners",
    "check_polygon_collision_sat",
    "CubicSpline2D",
    "cartesian_to_frenet",
    "frenet_to_cartesian",
]
