"""
Unit and Integration Tests for BharatDrive-AI Vercel Flask Serverless Application.
BharatDrive-AI - MathWorks Autonomous Driving Hackathon.
"""

import json
import pytest
from app import app, handler


@pytest.fixture
def client():
    """Create a test client for the Flask app."""
    app.config["TESTING"] = True
    with app.test_client() as client:
        yield client


def test_handler_alias_exists():
    """Verifies that handler is exported for Vercel Serverless Functions."""
    assert handler is not None
    assert handler == app


def test_api_index_exports():
    """Verifies that api/index.py imports and re-exports app and handler."""
    from api.index import app as api_app, handler as api_handler
    assert api_app is not None
    assert api_handler is not None
    assert api_app == app


def test_frontend_dashboard_serving(client):
    """Verifies that GET / serves the master dashboard index.html."""
    response = client.get("/")
    assert response.status_code == 200
    assert b"BHARATDRIVE-AI" in response.data or b"Dashboard" in response.data


def test_health_endpoint(client):
    """Verifies that GET /api/health returns operational status."""
    response = client.get("/api/health")
    assert response.status_code == 200
    data = response.get_json()
    assert data["status"] == "healthy"
    assert data["version"] == "2.5.0"
    assert data["subsystems"]["perception_fusion"] == "OPERATIONAL"
    assert data["subsystems"]["risk_assessment_irrm"] == "OPERATIONAL"
    assert data["subsystems"]["adaptive_planner"] == "OPERATIONAL"


def test_scenarios_endpoint(client):
    """Verifies that GET /api/scenarios returns all 6 scenarios."""
    response = client.get("/api/scenarios")
    assert response.status_code == 200
    data = response.get_json()
    assert data["count"] == 6
    assert len(data["scenarios"]) == 6

    # Test single scenario detail
    sc5 = client.get("/api/scenarios/5")
    assert sc5.status_code == 200
    sc5_data = sc5.get_json()
    assert "Cattle Crossing" in sc5_data["scenario"]["name"]


def test_vehicle_config_endpoint(client):
    """Verifies that GET /api/vehicle-config returns Tata Nexon EV specifications."""
    response = client.get("/api/vehicle-config")
    assert response.status_code == 200
    data = response.get_json()
    assert data["vehicle"]["make_model"] == "Tata Nexon EV"
    assert data["vehicle"]["dimensions"]["length_m"] > 0
    assert data["vehicle"]["dimensions"]["wheelbase_m"] == 2.498


def test_risk_assessment_endpoint(client):
    """Verifies that POST /api/risk-assessment calculates 7-factor IRRM risk."""
    payload = {
        "distance": 14.0,
        "relative_speed": 4.0,
        "approach_angle_deg": 5.0,
        "object_type": "cow",
        "road_width": 7.0,
        "friction_mu": 0.85,
        "ego_speed": 11.0
    }
    response = client.post("/api/risk-assessment", json=payload)
    assert response.status_code == 200
    data = response.get_json()
    assert 0.0 <= data["risk_score"] <= 1.0
    assert "criticality" in data
    assert "action_recommendation" in data
    assert "factors" in data
    assert "kinetic_risk" in data["factors"]


def test_plan_endpoint(client):
    """Verifies that POST /api/plan returns waypoints and commands."""
    payload = {
        "target_speed": 13.89,
        "ego_x": 0.0,
        "ego_y": 0.0,
        "ego_v": 10.0,
        "obstacles": [
            {"position": [24.0, 0.5], "velocity": [0.0, 0.0], "class": "cow", "risk_score": 0.65}
        ]
    }
    response = client.post("/api/plan", json=payload)
    assert response.status_code == 200
    data = response.get_json()
    assert "target_velocity" in data
    assert "tactical_nudge_m" in data
    assert "trajectory_points" in data
    assert len(data["trajectory_points"]) > 0


def test_simulate_step_endpoint(client):
    """Verifies that POST /api/simulate-step advances closed-loop pipeline."""
    payload = {
        "sim_time": 0.05,
        "cruise_speed": 13.89,
        "obstacles": [
            {"id": 1, "class": "cow", "position": [24.0, 0.5], "velocity": [0.0, 0.0]}
        ]
    }
    response = client.post("/api/simulate-step", json=payload)
    assert response.status_code == 200
    data = response.get_json()
    assert data["success"] is True
    assert "telemetry" in data
    assert "ego_v" in data["telemetry"]
    assert "state_name" in data["telemetry"]


def test_metrics_endpoint(client):
    """Verifies that GET /api/metrics returns benchmark evaluation data."""
    response = client.get("/api/metrics")
    assert response.status_code == 200
    data = response.get_json()
    assert data["benchmarks"]["collision_rate_percent"] == 0.00
    assert data["validation_status"] == "PASSED_ALL_BENCHMARKS"


def test_404_handler(client):
    """Verifies structured 404 response for non-existent API routes."""
    response = client.get("/api/non-existent-endpoint")
    assert response.status_code == 404
    data = response.get_json()
    assert "error" in data
