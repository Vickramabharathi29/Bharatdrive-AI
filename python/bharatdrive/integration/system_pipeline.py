"""
BharatDrive-AI Master End-to-End Autonomous Driving Pipeline.
MathWorks Autonomous Driving Hackathon.

Connects all 8 subsystems in a unified closed-loop architecture:
Sensor -> Detection -> Prediction -> Risk Assessment -> Path Planner ->
Decision Engine -> Vehicle Controller -> Vehicle Dynamics Plant.
"""

from __future__ import annotations
import math
from typing import Any
import numpy as np

from bharatdrive.perception.sensor_suite import SensorSuite, SensorDetection
from bharatdrive.perception.fusion_engine import SensorFusionEngine
from bharatdrive.risk.risk_engine import RiskAssessmentEngine
from bharatdrive.planning.adaptive_planner import AdaptivePlannerCoordinator
from bharatdrive.decision.stateflow_supervisor import StateflowDecisionSupervisor
from bharatdrive.control.vehicle_controller import VehicleControllerEngine
from bharatdrive.models.bicycle_model import DynamicBicycleModel
from bharatdrive.models.state import VehicleState


class BharatDrivePipeline:
    """
    Master closed-loop autonomous driving pipeline connecting all 8 modules.
    """

    def __init__(self, cruise_speed: float = 13.89, road_width: float = 7.00, friction_mu: float = 0.85) -> None:
        self.cruise_speed = float(cruise_speed)
        self.road_width = float(road_width)
        self.friction_mu = float(friction_mu)
        self.dt = 0.05  # Master simulation step: 20 Hz loop

        # 1. Perception & Fusion Engine
        self.fusion_engine = SensorFusionEngine()

        # 2. Dynamic Risk Assessment Engine (7-Factor IRRM)
        self.risk_engine = RiskAssessmentEngine()

        # 3. Adaptive Path Planner (Hybrid A* + PPO + DWA)
        self.planner = AdaptivePlannerCoordinator(target_speed=cruise_speed)

        # 4. Stateflow Decision Supervisor (8 States)
        self.decision_supervisor = StateflowDecisionSupervisor(cruise_speed=cruise_speed)

        # 5. Vehicle Motion Controller (Adaptive Stanley + PID)
        self.controller = VehicleControllerEngine(controller_type="Stanley")

        # 6. Tata Nexon EV 2-DOF Dynamic Plant
        self.plant = DynamicBicycleModel()
        self.ego_state = VehicleState(x=0.0, y=0.0, v=8.0, yaw=0.0, yaw_rate=0.0)

        # Telemetry logging
        self.history: list[dict[str, Any]] = []

    def step(self, sim_time: float, obstacles: list[dict[str, Any]]) -> dict[str, Any]:
        """
        Executes a single end-to-end closed-loop cycle.

        Args:
            sim_time: current simulation time (s)
            obstacles: list of world obstacles with position, velocity, and class

        Returns:
            step_record: full telemetry snapshot of the step
        """
        ego_pos = np.array([self.ego_state.x, self.ego_state.y])
        ego_v = float(self.ego_state.v)
        ego_yaw = float(self.ego_state.yaw)

        # -----------------------------------------------------------------
        # 1. Sensor & Detection Layer
        # -----------------------------------------------------------------
        fused_tracks = []
        for ob in obstacles:
            ox, oy = ob["position"][0], ob["position"][1]
            ovx, ovy = ob["velocity"][0], ob["velocity"][1]
            # Transform to vehicle frame
            dx, dy = ox - ego_pos[0], oy - ego_pos[1]
            dist = math.hypot(dx, dy)
            vrel = math.hypot(ego_v - ovx, -ovy)
            angle_deg = math.degrees(math.atan2(dy, dx) - ego_yaw)

            fused_tracks.append({
                "id": ob["id"],
                "class": ob["class"],
                "position": [ox, oy],
                "distance": dist,
                "relative_speed": vrel,
                "angle_deg": angle_deg,
                "velocity": [ovx, ovy],
            })

        # -----------------------------------------------------------------
        # 2. Prediction Layer (5.0s Horizon)
        # -----------------------------------------------------------------
        predicted_trajs = []
        for trk in fused_tracks:
            pred_pts = []
            for t_h in [1.0, 2.0, 3.0, 4.0, 5.0]:
                px = trk["position"][0] + trk["velocity"][0] * t_h
                py = trk["position"][1] + trk["velocity"][1] * t_h
                pred_pts.append([px, py])
            predicted_trajs.append(pred_pts)

        # -----------------------------------------------------------------
        # 3. Risk Assessment Layer (7-Factor IRRM)
        # -----------------------------------------------------------------
        weather = {"friction_mu": self.friction_mu, "visibility": 1.0}
        risk_results = []
        highest_threat = None
        max_risk = 0.0

        for trk in fused_tracks:
            r_eval = self.risk_engine.evaluate_risk(
                distance=trk["distance"],
                relative_speed=trk["relative_speed"],
                approach_angle_deg=trk["angle_deg"],
                object_type=trk["class"],
                road_width=self.road_width,
                weather_friction_mu=self.friction_mu,
                weather_visibility=1.0,
                traffic_density=len(fused_tracks),
                ego_speed=ego_v,
            )
            trk["risk_score"] = r_eval.risk_score
            trk["ttc"] = r_eval.time_to_collision
            risk_results.append(r_eval)

            if r_eval.risk_score > max_risk:
                max_risk = r_eval.risk_score
                highest_threat = {
                    "Distance": trk["distance"],
                    "VRel": trk["relative_speed"],
                    "Position": trk["position"],
                    "Class": trk["class"],
                    "RiskScore": r_eval.risk_score,
                    "TTC": r_eval.time_to_collision,
                }

        # -----------------------------------------------------------------
        # 4. Path Planning Layer (Hybrid A* + PPO + DWA)
        # -----------------------------------------------------------------
        ego_state_arr = np.array([self.ego_state.x, self.ego_state.y, self.ego_state.v, self.ego_state.yaw, self.ego_state.yaw_rate])
        plan_out = self.planner.replan_cycle(
            ego_state=ego_state_arr,
            detected_obstacles=fused_tracks,
            risk_results=risk_results,
            road_width=self.road_width,
            weather=weather,
        )

        # -----------------------------------------------------------------
        # 5. Decision Engine Layer (Stateflow 8-State Supervisor)
        # -----------------------------------------------------------------
        env_ctx = {
            "RoadWidth": self.road_width,
            "LeftClearance": 2.5,
            "RightClearance": 2.5,
            "OncomingDist": 120.0,
            "SignalState": "None",
        }
        decision = self.decision_supervisor.step(
            ego_state=ego_state_arr,
            threat_obs=highest_threat,
            env_context=env_ctx,
            dt=self.dt,
        )

        # -----------------------------------------------------------------
        # 6. Vehicle Controller Layer (Adaptive Stanley + PID)
        # -----------------------------------------------------------------
        target_v = decision["target_speed"]
        if decision["state_name"] == "Emergency Stop":
            target_v = 0.0

        guide_wps = plan_out["guide_path"]
        ctrl_cmd = self.controller.compute_controls(
            ego_state=ego_state_arr,
            waypoints=guide_wps,
            target_v=target_v,
            dt=self.dt,
        )

        # If Stateflow emergency braking, clamp acceleration to -6.5 m/s^2
        if decision["state_name"] == "Emergency Stop":
            ctrl_cmd["accel_cmd"] = -6.50
            ctrl_cmd["brake_pressure"] = 100.0

        # -----------------------------------------------------------------
        # 7. Vehicle Dynamics Plant Layer (Tata Nexon EV 2-DOF)
        # -----------------------------------------------------------------
        a_cmd = float(ctrl_cmd["accel_cmd"])
        steer_cmd = float(ctrl_cmd["steer_cmd"])

        # Update plant state
        self.ego_state = self.plant.step(
            current_state=self.ego_state,
            accel_cmd=a_cmd,
            steer_cmd=steer_cmd,
            dt=self.dt,
        )

        step_record = {
            "time": sim_time,
            "ego_x": self.ego_state.x,
            "ego_y": self.ego_state.y,
            "ego_v": self.ego_state.v,
            "ego_yaw": self.ego_state.yaw,
            "steer_cmd": steer_cmd,
            "accel_cmd": a_cmd,
            "state_name": decision["state_name"],
            "max_risk": max_risk,
            "nudge": plan_out["tactical_nudge"],
            "horn": decision["horn"],
            "hazards": decision["hazard_lights"],
        }
        self.history.append(step_record)
        return step_record
