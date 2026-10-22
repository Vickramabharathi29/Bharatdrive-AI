"""Dynamic Risk Assessment Engine evaluating the 7-Factor Indian Road Risk Metric (IRRM)."""

from __future__ import annotations
from dataclasses import dataclass, field
import math
import numpy as np
from typing import Dict, Any, Optional


@dataclass
class RiskFactors:
    r_kin: float
    phi_dir: float
    m_type: float
    c_width: float
    omega_weather: float
    psi_density: float


@dataclass
class RiskAssessmentResult:
    risk_score: float                  # [0.0, 1.0]
    criticality: str                   # 'LOW', 'MEDIUM', 'HIGH', 'CRITICAL'
    time_to_collision: float           # seconds (or inf)
    stopping_distance: float           # meters
    action_recommendation: str         # 'MAINTAIN_CRUISE', 'PREPARE_NUDGE', 'ACTIVE_DECELERATION', 'EMERGENCY_BRAKE'
    factors: RiskFactors
    object_id: Optional[int] = None
    object_class: Optional[str] = None


class RiskAssessmentEngine:
    """Evaluates multi-factor dynamic risk on unstructured Indian roads."""

    def __init__(
        self,
        tau0: float = 3.5,
        d0: float = 3.0,
        t_react: float = 1.0,
        a_comfort: float = 2.5,
        nominal_road_width: float = 7.0,
        mu_dry: float = 0.85,
    ):
        self.tau0 = tau0
        self.d0 = d0
        self.t_react = t_react
        self.a_comfort = a_comfort
        self.nominal_road_width = nominal_road_width
        self.mu_dry = mu_dry
        self.gravity = 9.81

        self.type_vulnerability = {
            "Pedestrian": 2.50,
            "Cattle": 2.20,
            "Cow": 2.20,
            "Two-Wheeler": 2.00,
            "Bike": 2.00,
            "Cycle": 2.00,
            "Auto-Rickshaw": 1.80,
            "Truck-Bus": 2.40,
            "Bus": 2.40,
            "Truck": 2.40,
            "Pothole": 1.60,
            "Push-Cart": 1.70,
            "Car": 1.00,
        }

    def evaluate_risk(
        self,
        distance: float,
        relative_speed: float,
        approach_angle_deg: float = 0.0,
        object_type: str = "Car",
        road_width: float = 7.0,
        weather_friction_mu: float = 0.85,
        weather_visibility: float = 1.0,
        traffic_density: int = 0,
        ego_speed: float = 13.89,
    ) -> RiskAssessmentResult:
        """Evaluates dynamic risk across all 7 physical and contextual dimensions."""
        d = max(0.1, float(distance))
        v_rel = float(relative_speed)

        # 1. Dimension 1 & 2: Distance & Relative Speed (Kinetic Risk)
        if v_rel > 0.05:
            ttc = d / v_rel
            r_ttc = math.exp(-ttc / self.tau0)
        else:
            ttc = float("inf")
            r_ttc = 0.0

        d_safe = self.d0 + ego_speed * self.t_react + (ego_speed**2) / (2.0 * self.a_comfort)
        r_prox = math.exp(-d / max(5.0, d_safe))
        r_kin = 0.65 * r_ttc + 0.35 * r_prox

        # 2. Dimension 3: Direction
        rad = math.radians(approach_angle_deg)
        phi_dir = 1.0 + 0.60 * max(0.0, -math.cos(rad)) + 0.45 * abs(math.sin(rad))

        # 3. Dimension 4: Object Type Vulnerability
        m_type = self.type_vulnerability.get(object_type, 1.00)

        # 4. Dimension 5: Road Width (Lateral Escape Bottleneck)
        eff_w = max(2.5, float(road_width))
        c_width = (self.nominal_road_width / eff_w)**1.4

        # 5. Dimension 6: Weather & Friction
        mu = max(0.20, min(1.0, float(weather_friction_mu)))
        vis = max(0.20, min(1.0, float(weather_visibility)))
        omega_weather = ((self.mu_dry / mu)**0.85) * (1.0 + 0.35 * (1.0 - vis))
        d_stop_emergency = (ego_speed**2) / (2.0 * mu * self.gravity)

        # 6. Dimension 7: Traffic Density
        psi_density = 1.0 + 0.12 * min(5.0, float(traffic_density))

        # 7. Total Unified Indian Road Risk Metric
        raw_risk = r_kin * (0.35 * phi_dir + 0.25 * m_type + 0.20 * c_width + 0.15 * omega_weather + 0.05 * psi_density)
        risk_score = float(np.clip(raw_risk, 0.0, 1.0))

        # Hard safety override
        if d <= d_stop_emergency and v_rel > 0.0:
            risk_score = max(risk_score, 0.90)

        # Categorize
        if risk_score < 0.35:
            criticality = "LOW"
            action = "MAINTAIN_CRUISE"
        elif risk_score < 0.65:
            criticality = "MEDIUM"
            action = "PREPARE_NUDGE"
        elif risk_score < 0.85:
            criticality = "HIGH"
            action = "ACTIVE_DECELERATION"
        else:
            criticality = "CRITICAL"
            action = "EMERGENCY_BRAKE"

        factors = RiskFactors(
            r_kin=r_kin,
            phi_dir=phi_dir,
            m_type=m_type,
            c_width=c_width,
            omega_weather=omega_weather,
            psi_density=psi_density,
        )

        return RiskAssessmentResult(
            risk_score=risk_score,
            criticality=criticality,
            time_to_collision=ttc,
            stopping_distance=d_stop_emergency,
            action_recommendation=action,
            factors=factors,
        )
