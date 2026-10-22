"""
Stateflow Tactical Decision Supervisor.
BharatDrive-AI - MathWorks Autonomous Driving Hackathon.

Evaluates multi-agent road context and selects among 8 discrete driving states:
  1. Accelerate
  2. Brake
  3. Overtake
  4. Wait
  5. Stop
  6. Emergency Stop
  7. Move Left
  8. Move Right
"""

from __future__ import annotations
import math
from typing import Any
import numpy as np

from bharatdrive.decision.maneuver_state import ManeuverState


class StateflowDecisionSupervisor:
    """
    Tactical decision supervisor executing at 10 Hz with transition guards,
    hysteresis timing, and fail-safe preemption.
    """

    def __init__(self, cruise_speed: float = 13.89) -> None:
        self.nominal_cruise_speed = float(cruise_speed)
        self.current_state = ManeuverState.ACCELERATE
        self.previous_state = ManeuverState.ACCELERATE
        self.state_time = 0.0
        self.overtake_phase = 0  # 0: None, 1: ChangeRight, 2: Passing, 3: CutBack
        self.min_follow_dist = 15.0
        self.critical_dist = 3.8
        self.hysteresis_time = 0.40
        self.horn_active = False
        self.hazard_lights_active = False

    def step(
        self,
        ego_state: np.ndarray | list[float],
        threat_obs: dict[str, Any] | Any | None = None,
        env_context: dict[str, Any] | None = None,
        dt: float = 0.1,
    ) -> dict[str, Any]:
        """
        Executes a single Stateflow decision cycle (10 Hz).

        Args:
            ego_state: [x, y, v] or [x, y, v, yaw, yaw_rate, ...]
            threat_obs: Threat obstacle dict or object
            env_context: Environmental road context dict
            dt: time step in seconds

        Returns:
            decision: dict containing active state, commands, and actuator flags
        """
        ego = np.asarray(ego_state, dtype=np.float64)
        ego_x, ego_y, ego_v = float(ego[0]), float(ego[1]), float(ego[2])

        # Extract threat obstacle information
        if threat_obs is None:
            obs_d = 100.0
            obs_vrel = 0.0
            obs_pos = [ego_x + 100.0, 0.0]
            obs_class = "None"
            obs_risk = 0.0
            obs_ttc = float("inf")
        elif isinstance(threat_obs, dict):
            obs_d = float(threat_obs.get("Distance", threat_obs.get("distance", 100.0)))
            obs_vrel = float(threat_obs.get("VRel", threat_obs.get("vrel", 0.0)))
            obs_pos = threat_obs.get("Position", threat_obs.get("position", [ego_x + 100.0, 0.0]))
            obs_class = str(threat_obs.get("Class", threat_obs.get("class", "None")))
            obs_risk = float(threat_obs.get("RiskScore", threat_obs.get("risk_score", 0.0)))
            obs_ttc = float(threat_obs.get("TTC", threat_obs.get("ttc", float("inf"))))
        else:
            obs_d = float(getattr(threat_obs, "Distance", 100.0))
            obs_vrel = float(getattr(threat_obs, "VRel", 0.0))
            obs_pos = getattr(threat_obs, "Position", [ego_x + 100.0, 0.0])
            obs_class = str(getattr(threat_obs, "Class", "None"))
            obs_risk = float(getattr(threat_obs, "RiskScore", 0.0))
            obs_ttc = float(getattr(threat_obs, "TTC", float("inf")))

        # Extract environment context
        env = env_context or {}
        road_w = float(env.get("RoadWidth", env.get("road_width", 7.0)))
        left_clear = float(env.get("LeftClearance", env.get("left_clearance", 2.5)))
        right_clear = float(env.get("RightClearance", env.get("right_clearance", 2.5)))
        oncoming_d = float(env.get("OncomingDist", env.get("oncoming_dist", 150.0)))
        signal = str(env.get("SignalState", env.get("signal_state", "None")))

        self.state_time += dt
        next_state = self.current_state
        self.horn_active = False
        self.hazard_lights_active = False

        # -----------------------------------------------------------------
        # 1. EMERGENCY STOP (Priority 1 Preemptive Interrupt)
        # -----------------------------------------------------------------
        is_emergency = (
            (0.0 < obs_ttc <= 1.10)
            or (obs_risk >= 0.88)
            or (obs_d <= self.critical_dist and ego_v > 0.5 and abs(obs_pos[1] - ego_y) < 1.4)
        )

        if is_emergency:
            next_state = ManeuverState.EMERGENCY_STOP

        elif self.current_state == ManeuverState.EMERGENCY_STOP:
            if ego_v < 0.2 and obs_risk < 0.55:
                if obs_d < 10.0:
                    next_state = ManeuverState.WAIT
                else:
                    next_state = ManeuverState.ACCELERATE
            else:
                next_state = ManeuverState.EMERGENCY_STOP

        # -----------------------------------------------------------------
        # 2. CONTROLLED STOP (Traffic Signal / Stop Line)
        # -----------------------------------------------------------------
        elif signal in ("Red", "Stop") or obs_class == "Roadblock":
            next_state = ManeuverState.STOP

        elif self.current_state == ManeuverState.STOP:
            if signal not in ("Red", "Stop") and obs_class != "Roadblock":
                next_state = ManeuverState.ACCELERATE

        # -----------------------------------------------------------------
        # 3. WAIT (Crossing Cattle, Pedestrian on Roadway)
        # -----------------------------------------------------------------
        elif (
            obs_class in ("Cattle", "Pedestrian")
            and (obs_d < 12.0)
            and (ego_v < 2.5 or obs_d < 6.0)
            and abs(obs_pos[1] - ego_y) < 1.6
        ):
            next_state = ManeuverState.WAIT
            if self.state_time > 1.5:
                self.horn_active = True

        elif self.current_state == ManeuverState.WAIT:
            if obs_d > 14.0 or abs(obs_pos[1] - ego_y) > 2.0 or obs_class == "None":
                next_state = ManeuverState.ACCELERATE
            else:
                next_state = ManeuverState.WAIT
                if self.state_time > 1.5:
                    self.horn_active = True

        # -----------------------------------------------------------------
        # 4. OVERTAKE (Multi-Phase Maneuver)
        # -----------------------------------------------------------------
        elif self.current_state == ManeuverState.OVERTAKE:
            next_state = self._evaluate_overtake_phases(ego_x, obs_pos[0], oncoming_d)

        elif (
            (self.state_time >= self.hysteresis_time)
            and obs_class in ("Auto-Rickshaw", "Cycle", "Truck-Bus")
            and (10.0 <= obs_d <= 25.0)
            and (obs_vrel > 2.0 or (ego_v - obs_vrel) < 0.6 * self.nominal_cruise_speed)
            and (road_w >= 6.5)
            and (oncoming_d >= 75.0)
            and (right_clear >= 2.2)
        ):
            next_state = ManeuverState.OVERTAKE
            self.overtake_phase = 1

        # -----------------------------------------------------------------
        # 5. MOVE LEFT (Obstacle on Center/Right: Cattle, Truck, etc.)
        # -----------------------------------------------------------------
        elif (
            (self.state_time >= self.hysteresis_time)
            and (obs_d < 30.0)
            and (obs_pos[1] >= -0.2)
            and (left_clear >= 1.6)
            and (obs_class in ("Cattle", "Truck-Bus") or obs_pos[1] > 0.0)
        ):
            next_state = ManeuverState.MOVE_LEFT

        elif self.current_state == ManeuverState.MOVE_LEFT:
            if (obs_pos[0] < ego_x - 3.0) or (obs_d > 35.0):
                next_state = ManeuverState.ACCELERATE

        # -----------------------------------------------------------------
        # 6. MOVE RIGHT (Obstacle on Left: Pothole, Cart, Pedestrian)
        # -----------------------------------------------------------------
        elif (
            (self.state_time >= self.hysteresis_time)
            and (obs_d < 30.0)
            and (obs_pos[1] <= 0.2)
            and (right_clear >= 1.6)
            and (oncoming_d >= 45.0)
            and (obs_class in ("Pothole", "Push-Cart", "Pedestrian") or obs_pos[1] < 0.0)
        ):
            next_state = ManeuverState.MOVE_RIGHT

        elif self.current_state == ManeuverState.MOVE_RIGHT:
            if (obs_pos[0] < ego_x - 3.0) or (obs_d > 35.0):
                next_state = ManeuverState.ACCELERATE

        # -----------------------------------------------------------------
        # 7. BRAKE (Slower Lead or Elevated Risk)
        # -----------------------------------------------------------------
        elif (obs_d < 24.0 and obs_vrel > 0.5) or (0.42 <= obs_risk < 0.88):
            next_state = ManeuverState.BRAKE

        elif self.current_state == ManeuverState.BRAKE:
            if obs_d >= 28.0 and obs_risk < 0.35:
                next_state = ManeuverState.ACCELERATE
            elif ego_v < 0.5 and obs_d < 8.0:
                next_state = ManeuverState.WAIT

        # -----------------------------------------------------------------
        # 8. ACCELERATE (Default Nominal Free-Flow State)
        # -----------------------------------------------------------------
        else:
            next_state = ManeuverState.ACCELERATE

        # Transition handling
        if next_state != self.current_state:
            self.previous_state = self.current_state
            self.current_state = next_state
            self.state_time = 0.0
            if next_state != ManeuverState.OVERTAKE:
                self.overtake_phase = 0

        return self._generate_state_commands(ego_v, obs_d, obs_vrel)

    def _evaluate_overtake_phases(self, ego_x: float, lead_x: float, oncoming_d: float) -> ManeuverState:
        # Abort if oncoming vehicle gets too close
        if oncoming_d < 45.0 and self.overtake_phase < 3:
            self.overtake_phase = 0
            return ManeuverState.BRAKE

        if self.overtake_phase == 1:
            if self.state_time >= 1.2 or (ego_x >= lead_x - 8.0):
                self.overtake_phase = 2  # Passing
            return ManeuverState.OVERTAKE
        elif self.overtake_phase == 2:
            if ego_x > (lead_x + 6.0):
                self.overtake_phase = 3  # Cut-back
            return ManeuverState.OVERTAKE
        elif self.overtake_phase == 3:
            if self.state_time >= 1.5 or (ego_x > lead_x + 14.0):
                self.overtake_phase = 0
                return ManeuverState.ACCELERATE
            return ManeuverState.OVERTAKE
        else:
            self.overtake_phase = 0
            return ManeuverState.ACCELERATE

    def _generate_state_commands(self, ego_v: float, obs_d: float, obs_vrel: float) -> dict[str, Any]:
        horn = self.horn_active
        hazards = self.hazard_lights_active

        if self.current_state == ManeuverState.EMERGENCY_STOP:
            target_speed = 0.0
            target_accel = -6.50
            target_lat = 0.0
            hazards = True
        elif self.current_state == ManeuverState.STOP:
            target_speed = 0.0
            target_accel = -2.50
            target_lat = 0.0
        elif self.current_state == ManeuverState.WAIT:
            target_speed = 0.0
            target_accel = -1.50
            target_lat = 0.0
            hazards = True
        elif self.current_state == ManeuverState.BRAKE:
            lead_speed = max(0.0, ego_v - obs_vrel)
            target_speed = max(0.0, lead_speed * 0.90)
            target_accel = max(-4.50, (lead_speed - ego_v) / 1.20)
            target_lat = 0.0
        elif self.current_state == ManeuverState.ACCELERATE:
            target_speed = self.nominal_cruise_speed
            target_accel = min(2.50, (self.nominal_cruise_speed - ego_v) / 2.0)
            target_lat = 0.0
        elif self.current_state == ManeuverState.MOVE_LEFT:
            target_speed = min(ego_v, self.nominal_cruise_speed * 0.85)
            target_accel = 0.0
            target_lat = -1.25
        elif self.current_state == ManeuverState.MOVE_RIGHT:
            target_speed = min(ego_v, self.nominal_cruise_speed * 0.85)
            target_accel = 0.0
            target_lat = +1.25
        elif self.current_state == ManeuverState.OVERTAKE:
            if self.overtake_phase == 1:
                target_speed = min(self.nominal_cruise_speed, ego_v + 1.0)
                target_accel = 1.0
                target_lat = +1.80
            elif self.overtake_phase == 2:
                target_speed = min(self.nominal_cruise_speed * 1.15, ego_v + 2.5)
                target_accel = 2.0
                target_lat = +1.80
            else:
                target_speed = self.nominal_cruise_speed
                target_accel = 0.0
                target_lat = 0.0
        else:
            target_speed = self.nominal_cruise_speed
            target_accel = 0.0
            target_lat = 0.0

        return {
            "state": self.current_state,
            "state_name": self.current_state.readable_name,
            "target_speed": float(target_speed),
            "target_accel": float(target_accel),
            "target_lateral_offset": float(target_lat),
            "overtake_phase": int(self.overtake_phase),
            "horn": bool(horn),
            "hazard_lights": bool(hazards),
        }
