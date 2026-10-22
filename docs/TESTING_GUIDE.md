# BharatDrive-AI: Comprehensive Testing & Validation Guide

This document details the automated testing framework, test catalog, regression validation procedures, and code coverage metrics for **BharatDrive-AI**.

---

## 1. Test Suite Overview & Quality Metrics

The test suite enforces rigorous quality assurance across all perception, planning, risk, decision, control, and evaluation modules:
- **Total Test Cases**: **83 automated unit and integration tests**
- **Pass Rate**: **100% (83 / 83 Passed)**
- **Execution Time**: **~7.5 seconds**
- **Flakiness**: **0% (Zero intermittent test failures)**
- **Warning Count**: **0 compiler, deprecation, or runtime warnings**

---

## 2. Test Catalog & Module Breakdown

| Test File | Test Cases | Subsystems Validated | Key Invariants Tested |
|---|:---:|---|---|
| [`test_config.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/tests/test_config.py) | 6 | YAML Parsers & Catalogs | Validates Tata Nexon EV parameters, IRC:73 Indian road specs, and planner cost weights |
| [`test_state_and_transforms.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/tests/test_state_and_transforms.py) | 5 | Geometric Transforms | 2D Oriented Bounding Box (OBB) generation, Separating Axis Theorem (SAT) collision checking |
| [`test_bicycle_model.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/tests/test_bicycle_model.py) | 6 | Vehicle Dynamics Plants | Kinematic RK4 bicycle model, Dynamic bicycle model with Pacejka tire slip angles ($\alpha_f, \alpha_r$) |
| [`test_frenet.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/tests/test_frenet.py) | 6 | Curvilinear Splines | Natural cubic spline interpolation, Cartesian-to-Frenet and Frenet-to-Cartesian invertible transforms |
| [`test_yolo_detection.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/tests/test_yolo_detection.py) | 5 | Perception & YOLO | Multi-class detection for auto-rickshaws, cattle, two-wheelers, pedestrians, and potholes |
| [`test_sensor_fusion.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/tests/test_sensor_fusion.py) | 6 | Multi-Sensor Fusion | Hungarian data association, Mahalanobis distance gating, EKF state updates for fused tracks |
| [`test_trajectory_prediction.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/tests/test_trajectory_prediction.py) | 6 | Multi-Agent Prediction | Constant Turn Rate and Velocity (CTRV) EKF, Physics Kalman predictor, GRU/LSTM models |
| [`test_risk_assessment.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/tests/test_risk_assessment.py) | 7 | 7-Factor IRRM Engine | Composite risk score calculation, individual factor weighting, and criticality zone boundaries |
| [`test_adaptive_path_planning.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/tests/test_adaptive_path_planning.py) | 8 | Dual-Horizon Motion Planner | Hybrid A* kinematic search, Frenet quintic/quartic sampling, multi-objective cost optimization |
| [`test_stateflow_decision.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/tests/test_stateflow_decision.py) | 8 | Tactical Decision Engine | 8 operational states: Cruise, Nudge Left, Nudge Right, Follow Lead, Yield, Straddle, AEB |
| [`test_vehicle_controller.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/tests/test_vehicle_controller.py) | 8 | Motion Controllers | Adaptive Stanley lateral path tracker, curvature feedforward, longitudinal PID/AEB deceleration |
| [`test_scenarios.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/tests/test_scenarios.py) | 5 | Indian Scenario Harness | Validates actor geometries and corridor configurations across all 6 Indian road scenarios |
| [`test_end_to_end_integration.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/tests/test_end_to_end_integration.py) | 3 | Master Closed-Loop Pipeline | Executes 50-step end-to-end simulation linking Sensors -> Perception -> IRRM -> Planner -> Controller -> Plant |
| [`test_evaluation_metrics.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/tests/test_evaluation_metrics.py) | 4 | Evaluation Metrics Engine | Validates all 7 metrics (Collision Rate, Speed, Completion, Replan, Jerk, Clearance, CPU) & Graph Generator |

---

## 3. Running the Test Suite

### Run All Unit Tests:
```bash
python -m pytest python/tests -v
```

### Run with Execution Duration Timing (Slowest 10 Tests):
```bash
python -m pytest python/tests --durations=10
```

### Run a Specific Test Module:
```bash
# Test the 7-Factor IRRM Risk Assessment Engine
python -m pytest python/tests/test_risk_assessment.py -v

# Test the Evaluation Metrics & Graph Generator
python -m pytest python/tests/test_evaluation_metrics.py -v

# Test the Master End-to-End Closed-Loop Pipeline
python -m pytest python/tests/test_end_to_end_integration.py -v
```

### Generate Code Coverage Report:
```bash
python -m pytest python/tests --cov=python/bharatdrive --cov-report=term-missing --cov-report=html
# View HTML report at: htmlcov/index.html
```

---

## 4. MATLAB Verification Scripts

In addition to pytest, the MATLAB environment provides dedicated validation scripts:

```matlab
% In the MATLAB Command Window:
startup

% 1. Run Master End-to-End Simulation Harness
run_end_to_end_simulation

% 2. Run Lateral Controller Comparison (Stanley vs Pure Pursuit vs NMPC)
run_controller_comparison

% 3. Run Dynamic Risk Assessment Simulation
run_risk_assessment_simulation

% 4. Run Evaluation Metrics Visualizer
run_evaluation_metrics
```
