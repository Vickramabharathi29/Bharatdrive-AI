# BharatDrive-AI: Complete Repository File Catalog & Module Architecture

This document catalogs and explains **every file and module** across the entire **BharatDrive-AI** repository, detailing its functional responsibility, algorithmic role, inputs, outputs, and dependencies.

---

## 1. Top-Level Root Configuration & Entrypoints

| File Path | Functional Description & Role |
|---|---|
| [`README.md`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/README.md) | Master repository landing page; executive summary, system architecture diagram, benchmark comparison table, quickstart runbooks, and IEEE citations. |
| [`pyproject.toml`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/pyproject.toml) | Modern PEP 518/621 build specification; configures package metadata, dependencies, entrypoints, and pytest configurations. |
| [`requirements.txt`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/requirements.txt) | Pinned Python package dependencies: `numpy`, `scipy`, `pyyaml`, `pytest`, `matplotlib`, `opencv-python-headless`. |
| [`LICENSE`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/LICENSE) | Formal Apache License, Version 2.0 terms and conditions for open-source distribution. |
| [`CONTRIBUTING.md`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/CONTRIBUTING.md) | Open-source contribution guidelines, code style standards, PR workflow, and test verification requirements. |
| [`.gitignore`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/.gitignore) | Git exclusion patterns covering Python bytecode, virtual environments, MATLAB `.asv`/`.slxc` cache files, and OS artifacts. |
| [`.github/workflows/ci.yml`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/.github/workflows/ci.yml) | GitHub Actions Continuous Integration pipeline; automates testing across Python 3.10, 3.11, and 3.12 matrices. |
| [`conftest.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/conftest.py) | Pytest root configuration; sets up Python system paths so test suites cleanly import `bharatdrive`. |

---

## 2. Configuration Subsystem (`config/`)

| File Path | Functional Description & Role |
|---|---|
| [`config/vehicle_params.yaml`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/config/vehicle_params.yaml) | Defines geometric, inertial, and dynamic parameters for the **Tata Nexon EV**: mass ($1400\text{ kg}$), wheelbase ($2.498\text{ m}$), cornering stiffnesses, and steering limits. |
| [`config/indian_road_specs.yaml`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/config/indian_road_specs.yaml) | Indian Roads Congress (IRC:73 / IRC:SP:84) geometric specifications: lane widths, shoulder envelopes, camber, and heterogeneous actor catalog. |
| [`config/planner_config.yaml`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/config/planner_config.yaml) | Trajectory planner hyperparameter configuration: Frenet sampling horizons ($T \in [3.0, 6.0]\text{ s}$), lateral offsets, cost weights, and SAT safety margins. |

---

## 3. Core Python AI & Algorithms Package (`python/bharatdrive/`)

### 3.1 Subsystem Root & Configurations
- [`python/bharatdrive/__init__.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/bharatdrive/__init__.py): Root package initialization, version metadata (`v2.5.0`), and public API exports.
- [`python/bharatdrive/config.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/bharatdrive/config.py): Strongly-typed dataclass loaders (`VehicleConfig`, `RoadConfig`, `PlannerConfig`) that parse YAML files with validation fallbacks.

### 3.2 Vehicle Plant Models (`python/bharatdrive/models/`)
- [`python/bharatdrive/models/state.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/bharatdrive/models/state.py): Implements [`VehicleState`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/bharatdrive/models/state.py), encapsulating 7-state vehicle dynamics $[x, y, v, \theta, \omega, a, \delta]$ adhering to ISO 8855 standards.
- [`python/bharatdrive/models/bicycle_model.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/bharatdrive/models/bicycle_model.py): Implements both [`KinematicBicycleModel`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/bharatdrive/models/bicycle_model.py) (with 4th-Order Runge-Kutta numerical integration) and [`DynamicBicycleModel`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/bharatdrive/models/bicycle_model.py) (with Pacejka tire slip angles $\alpha_f, \alpha_r$ and cornering stiffnesses).

### 3.3 Geometry & Transformations (`python/bharatdrive/geometry/`)
- [`python/bharatdrive/geometry/transforms.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/bharatdrive/geometry/transforms.py): Implements 2D Oriented Bounding Box (OBB) calculations, coordinate frame transforms, and polygon intersection testing using the Separating Axis Theorem (SAT).
- [`python/bharatdrive/geometry/frenet.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/bharatdrive/geometry/frenet.py): Implements 2D natural cubic splines with continuous curvature $\kappa(s)$, bidirectional Cartesian-to-Frenet and Frenet-to-Cartesian transformations.

### 3.4 Perception & Multi-Sensor Fusion (`python/bharatdrive/perception/`)
- [`python/bharatdrive/perception/sensor_suite.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/bharatdrive/perception/sensor_suite.py): Models the multi-sensor hardware suite (Stereo Camera, 128-beam 3D LiDAR, 77 GHz Radar, RTK GNSS/IMU) with realistic sensor noise and field-of-view limits.
- [`python/bharatdrive/perception/fusion_engine.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/bharatdrive/perception/fusion_engine.py): Multi-sensor tracking engine using Hungarian data association and Extended Kalman Filters (EKF) to maintain continuous object tracks.

### 3.5 Dynamic Risk Assessment (`python/bharatdrive/risk/`)
- [`python/bharatdrive/risk/risk_engine.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/bharatdrive/risk/risk_engine.py): Computes the **7-Factor Indian Road Risk Metric (IRRM)**, synthesizing kinetic threat, approach heading, obstacle vulnerability, road bottlenecks, surface friction, multi-agent density, and crest occlusions into a continuous risk field $[0.0, 1.0]$.

### 3.6 Dual-Horizon Motion Planning (`python/bharatdrive/planning/`)
- [`python/bharatdrive/planning/hybrid_astar.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/bharatdrive/planning/hybrid_astar.py): Non-holonomic continuous-state $(x, y, \theta)$ global corridor search using Reeds-Shepp heuristics and discrete steering primitives.
- [`python/bharatdrive/planning/frenet_optimal_trajectory.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/bharatdrive/planning/frenet_optimal_trajectory.py): Local trajectory generation using Werling quintic and quartic polynomials sampled over time and lateral offsets.
- [`python/bharatdrive/planning/adaptive_planner.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/bharatdrive/planning/adaptive_planner.py): Unified coordinator coupling Hybrid A* corridor guidance with Frenet polynomial rollouts and tactical nudging offsets ($\pm 1.25\text{ m}$).

### 3.7 Tactical Decision Engine (`python/bharatdrive/decision/`)
- [`python/bharatdrive/decision/stateflow_supervisor.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/bharatdrive/decision/stateflow_supervisor.py): 8-state deterministic behavioral finite state machine (Cruise, Follow Lead, Nudge Left, Nudge Right, Yield, Pothole Straddle, Active Deceleration, AEB).

### 3.8 Motion Control & Path Tracking (`python/bharatdrive/control/`)
- [`python/bharatdrive/control/stanley_controller.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/bharatdrive/control/stanley_controller.py): Adaptive Stanley path tracking controller combining front-axle cross-track error with feedforward road curvature compensation and slew rate limiting.
- [`python/bharatdrive/control/vehicle_controller.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/bharatdrive/control/vehicle_controller.py): Unified lateral and longitudinal control coordinator managing steering actuation, torque/throttle, and friction braking.

### 3.9 End-to-End Master Pipeline (`python/bharatdrive/integration/`)
- [`python/bharatdrive/integration/system_pipeline.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/bharatdrive/integration/system_pipeline.py): Master closed-loop pipeline executing the complete 20 Hz cycle connecting Sensors $\to$ Perception $\to$ Fusion $\to$ IRRM $\to$ Decision $\to$ Planner $\to$ Controller $\to$ Plant.

### 3.10 Evaluation & Benchmarking Suite (`python/bharatdrive/evaluation/`)
- [`python/bharatdrive/evaluation/metrics_engine.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/bharatdrive/evaluation/metrics_engine.py): Calculates all 7 required performance indicators: Collision Rate, Average Speed, Scenario Completion, Average Replanning Time, Path Smoothness, Minimum Clearance, and CPU Usage.
- [`python/bharatdrive/evaluation/graph_generator.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/bharatdrive/evaluation/graph_generator.py): Renders publication-grade dark-themed vector SVGs and high-resolution PNGs for all metrics and comparative dashboards.

---

## 4. Native MATLAB & Simulink Architecture (`matlab/` & `simulink/`)

### 4.1 Native MATLAB Object-Oriented Package (`matlab/+bharatdrive/`)
- `+bharatdrive/+config/`: Vehicle parameter and Indian road specification loaders in native MATLAB.
- `+bharatdrive/+models/`: Native MATLAB classes for [`KinematicBicycle.m`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/matlab/+bharatdrive/+models/KinematicBicycle.m) and [`DynamicBicycle.m`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/matlab/+bharatdrive/+models/DynamicBicycle.m).
- `+bharatdrive/+frenet/`: 2D natural spline class [`Spline2D.m`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/matlab/+bharatdrive/+frenet/Spline2D.m) and Frenet transformation functions.
- `+bharatdrive/+perception/`: Sensor and TrackedObject data classes.
- `+bharatdrive/+risk/`: Native MATLAB 7-factor IRRM risk calculator.
- `+bharatdrive/+decision/`: Stateflow transition rules and state constants.
- `+bharatdrive/+planning/`: Hybrid A* corridor search and Frenet polynomial trajectory generators.
- `+bharatdrive/+control/`: Native Stanley and longitudinal speed controller classes.

### 4.2 MATLAB App Designer Telemetry Interface
- [`matlab/BharatDriveDashboard.m`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/matlab/BharatDriveDashboard.m): Native MATLAB `matlab.apps.AppBase` class with circular speed gauge, 7-factor risk bar chart, 2D BEV axes, warning HUD, sensor status lamps, and detected objects table.
- [`matlab/scripts/run_dashboard.m`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/matlab/scripts/run_dashboard.m): Entrypoint script initializing paths and instantiating `BharatDriveDashboard`.
- [`matlab/startup.m`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/matlab/startup.m): Environment initialization script configuring paths and printing startup banner.

### 4.3 MATLAB Simulation & Benchmark Scripts (`matlab/scripts/`)
- [`matlab/scripts/run_evaluation_metrics.m`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/matlab/scripts/run_evaluation_metrics.m): Native MATLAB benchmark evaluation visualizer rendering tiled layout figures.
- [`matlab/scripts/run_end_to_end_simulation.m`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/matlab/scripts/run_end_to_end_simulation.m): Closed-loop simulation script running all 6 scenarios in MATLAB.
- [`matlab/scripts/run_controller_comparison.m`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/matlab/scripts/run_controller_comparison.m): Quantitative comparison of Stanley, Pure Pursuit, and NMPC lateral controllers.
- [`matlab/scripts/run_adaptive_planning_simulation.m`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/matlab/scripts/run_adaptive_planning_simulation.m): Path planning simulation visualizing Hybrid A* corridor search and Frenet polynomial bundles.

### 4.4 Simulink Co-Simulation Harness (`simulink/`)
- `simulink/models/`: Simulink block diagrams for vehicle dynamics, controllers, and sensor buses.
- `simulink/buses/`: Signal dictionaries defining `EgoStateBus`, `PerceptionBus`, `RiskAssessmentBus`, and `ControlCommandBus`.
- `simulink/models/blocks/block_hybrid_astar.m`: Simulink MATLAB Function block for online Hybrid A* corridor generation.

---

## 5. Interactive Web Dashboard (`dashboard/`)

| File Path | Functional Description & Role |
|---|---|
| [`dashboard/index.html`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/dashboard/index.html) | Semantic HTML5 master application featuring 2D Canvas BEV, SVG speedometer, scenario selector, and the **Evaluation Metrics** modal. |
| [`dashboard/css/dashboard.css`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/dashboard/css/dashboard.css) | Custom glassmorphic dark-mode design system with responsive multi-column grid, typography, and modal animations. |
| [`dashboard/js/simulation_engine.js`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/dashboard/js/simulation_engine.js) | 20 Hz JavaScript simulation engine implementing the 2-DOF bicycle plant, 7-factor IRRM, Stateflow decision supervisor, and all 6 Indian scenarios. |
| [`dashboard/js/dashboard_app.js`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/dashboard/js/dashboard_app.js) | UI controller managing 60 FPS Canvas rendering, SVG needle animation, telemetry updates, modal events, and graph tab switching. |
| `dashboard/figures/`: High-resolution benchmark figures accessible directly by the web browser for the Evaluation Metrics viewer. |

---

## 6. Generated Reports & Publication Figures (`reports/`)

| File Path | Description |
|---|---|
| [`reports/TECHNICAL_REPORT.md`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/reports/TECHNICAL_REPORT.md) | Full technical paper detailing mathematical models, Indian road challenges, and benchmark results. |
| [`reports/evaluation_metrics_report.json`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/reports/evaluation_metrics_report.json) | Raw numerical telemetry and aggregate benchmark metrics stored in structured JSON. |
| [`reports/evaluation_metrics_report.md`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/reports/evaluation_metrics_report.md) | Markdown summary of benchmark findings and statistical comparisons against baselines. |
| `reports/figures/*.png` & `*.svg` | Publication-grade charts: clearance, collision rate, speed, completion, replanning latency, jerk, and CPU usage. |

---

## 7. Automated Test Suite (`python/tests/`)

14 comprehensive test modules enforcing 100% test pass rate across 83 test cases:
- [`test_config.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/tests/test_config.py), [`test_state_and_transforms.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/tests/test_state_and_transforms.py), [`test_bicycle_model.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/tests/test_bicycle_model.py), [`test_frenet.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/tests/test_frenet.py)
- [`test_yolo_detection.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/tests/test_yolo_detection.py), [`test_sensor_fusion.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/tests/test_sensor_fusion.py), [`test_trajectory_prediction.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/tests/test_trajectory_prediction.py)
- [`test_risk_assessment.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/tests/test_risk_assessment.py), [`test_adaptive_path_planning.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/tests/test_adaptive_path_planning.py), [`test_stateflow_decision.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/tests/test_stateflow_decision.py)
- [`test_vehicle_controller.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/tests/test_vehicle_controller.py), [`test_scenarios.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/tests/test_scenarios.py), [`test_end_to_end_integration.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/tests/test_end_to_end_integration.py), [`test_evaluation_metrics.py`](file:///c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI/python/tests/test_evaluation_metrics.py)
