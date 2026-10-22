"""Unit tests for Dynamic Risk Assessment Engine and Spatio-Temporal Costmap."""

import math
import numpy as np
import pytest

from bharatdrive.risk.risk_engine import RiskAssessmentEngine
from bharatdrive.risk.costmap import SpatioTemporalCostmap


@pytest.fixture
def risk_engine():
    return RiskAssessmentEngine()


def test_distance_sensitivity(risk_engine):
    # Closer distance should yield strictly higher risk score
    res_close = risk_engine.evaluate_risk(distance=15.0, relative_speed=10.0)
    res_far = risk_engine.evaluate_risk(distance=60.0, relative_speed=10.0)
    assert res_close.risk_score > res_far.risk_score


def test_relative_speed_sensitivity(risk_engine):
    # Fast closing speed should yield higher risk than slow approach
    res_fast = risk_engine.evaluate_risk(distance=30.0, relative_speed=20.0)
    res_slow = risk_engine.evaluate_risk(distance=30.0, relative_speed=2.0)
    assert res_fast.risk_score > res_slow.risk_score
    assert res_fast.time_to_collision < res_slow.time_to_collision


def test_direction_sensitivity(risk_engine):
    # Oncoming (180 deg) and orthogonal cut-in (90 deg) should have higher Phi_dir than preceding (0 deg)
    res_oncoming = risk_engine.evaluate_risk(distance=35.0, relative_speed=15.0, approach_angle_deg=180.0)
    res_cutin = risk_engine.evaluate_risk(distance=35.0, relative_speed=15.0, approach_angle_deg=90.0)
    res_lead = risk_engine.evaluate_risk(distance=35.0, relative_speed=15.0, approach_angle_deg=0.0)

    assert res_oncoming.factors.phi_dir > res_lead.factors.phi_dir
    assert res_cutin.factors.phi_dir > res_lead.factors.phi_dir


def test_object_type_vulnerability(risk_engine):
    # Vulnerable Pedestrian and massive Cattle should have higher risk than standard Car
    res_ped = risk_engine.evaluate_risk(distance=25.0, relative_speed=8.0, object_type="Pedestrian")
    res_cow = risk_engine.evaluate_risk(distance=25.0, relative_speed=8.0, object_type="Cattle")
    res_car = risk_engine.evaluate_risk(distance=25.0, relative_speed=8.0, object_type="Car")

    assert res_ped.factors.m_type > res_car.factors.m_type
    assert res_cow.factors.m_type > res_car.factors.m_type
    assert res_ped.risk_score > res_car.risk_score


def test_road_width_bottleneck_factor(risk_engine):
    # Narrow village road (3.75m) should amplify C_width risk compared to wide highway (14.0m)
    res_narrow = risk_engine.evaluate_risk(distance=25.0, relative_speed=10.0, road_width=3.75)
    res_wide = risk_engine.evaluate_risk(distance=25.0, relative_speed=10.0, road_width=14.0)

    assert res_narrow.factors.c_width > res_wide.factors.c_width
    assert res_narrow.factors.c_width == pytest.approx((7.0 / 3.75)**1.4, rel=1e-2)
    assert res_narrow.risk_score > res_wide.risk_score


def test_weather_and_friction_penalty(risk_engine):
    # Monsoon wet road (mu=0.50, vis=0.60) should increase risk and stopping distance vs dry (mu=0.85, vis=1.0)
    res_dry = risk_engine.evaluate_risk(
        distance=25.0, relative_speed=10.0, weather_friction_mu=0.85, weather_visibility=1.0
    )
    res_wet = risk_engine.evaluate_risk(
        distance=25.0, relative_speed=10.0, weather_friction_mu=0.50, weather_visibility=0.60
    )

    assert res_wet.factors.omega_weather > res_dry.factors.omega_weather
    assert res_wet.stopping_distance > res_dry.stopping_distance
    assert res_wet.risk_score > res_dry.risk_score


def test_traffic_density_pinch(risk_engine):
    # Crowded scene (5 neighbors) should amplify psi_density
    res_crowded = risk_engine.evaluate_risk(distance=25.0, relative_speed=10.0, traffic_density=5)
    res_empty = risk_engine.evaluate_risk(distance=25.0, relative_speed=10.0, traffic_density=0)

    assert res_crowded.factors.psi_density > res_empty.factors.psi_density
    assert res_crowded.risk_score > res_empty.risk_score


def test_criticality_and_action_recommendation(risk_engine):
    # Imminent critical cut-in (d = 5m, v_rel = 15m/s)
    res_crit = risk_engine.evaluate_risk(distance=5.0, relative_speed=15.0, object_type="Cattle")
    assert res_crit.criticality == "CRITICAL"
    assert res_crit.action_recommendation == "EMERGENCY_BRAKE"

    # Distant low risk (d = 70m, v_rel = 1m/s)
    res_low = risk_engine.evaluate_risk(distance=70.0, relative_speed=1.0, object_type="Car")
    assert res_low.criticality in ["LOW", "MEDIUM"]


def test_spatiotemporal_costmap_generation(risk_engine):
    costmap = SpatioTemporalCostmap(x_range=(-10, 60), y_range=(-8, 8), resolution=0.5)

    res = risk_engine.evaluate_risk(distance=20.0, relative_speed=8.0, object_type="Auto-Rickshaw")
    setattr(res, "position", np.array([20.0, -1.0, 0.0]))
    setattr(res, "object_class", "Auto-Rickshaw")

    costmap.update_costmap([res], road_width=7.0)
    assert costmap.cost_grid.shape == (costmap.grid_h, costmap.grid_w)
    # The cost near obstacle at (20, -1) should be significantly higher than far away
    # Check max cost near obstacle center
    assert np.max(costmap.cost_grid) > 50.0
