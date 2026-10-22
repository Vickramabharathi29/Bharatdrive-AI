"""Unit tests to validate RoadRunner Indian road scenario definition YAML files."""

from pathlib import Path
import yaml
import pytest


def get_scenario_files():
    root = Path(__file__).resolve().parent.parent.parent
    rr_dir = root / "roadrunner_scenes"
    return list(rr_dir.glob("scenario_*.yaml"))


def test_scenario_files_exist():
    scenarios = get_scenario_files()
    assert len(scenarios) == 5, f"Expected 5 scenario files, found {len(scenarios)}"


@pytest.mark.parametrize("scenario_path", get_scenario_files())
def test_scenario_yaml_structure(scenario_path):
    with open(scenario_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    # Core required keys
    assert "scenario_id" in data
    assert "title" in data
    assert "road_signs" in data
    assert "road_boundaries" in data
    assert "actors" in data

    actors = data["actors"]
    assert "ego" in actors
    assert "moving_vehicles" in actors

    # Verify heterogeneous actor entities
    if "pedestrians" in actors:
        assert isinstance(actors["pedestrians"], list)
    if "animals" in actors:
        assert isinstance(actors["animals"], list)
