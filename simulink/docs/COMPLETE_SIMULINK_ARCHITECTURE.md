# Complete End-to-End Autonomous Driving Architecture
**BharatDrive-AI — MathWorks Autonomous Driving Hackathon**

---

## 1. Master System Block Diagram

```mermaid
graph TD
    subgraph S1 ["1. Sensor Suite (20 Hz)"]
        CAM["Wide RGB Camera (120° FOV)"]
        LID["32-Channel Rotating LiDAR"]
        RAD["77 GHz Doppler Radar"]
        ENV_G["RoadRunner 3D Environment"] --> CAM
        ENV_G --> LID
        ENV_G --> RAD
    end

    subgraph S2 ["2. Detection & Fusion (20 Hz)"]
        CAM --> YOLO["YOLOv11 11-Class Detection"]
        LID --> CLUST["LiDAR 3D Clustering"]
        RAD --> EKF["Radar Doppler EKF"]
        YOLO --> FUSION["GNN Multi-Sensor Fusion Engine"]
        CLUST --> FUSION
        EKF --> FUSION
        FUSION --> FUSED_TRACKS["Fused Perception Tracks (5x7)
        [x, y, vx, vy, class_id, conf, dist]"]
    end

    subgraph S3 ["3. Trajectory Prediction (10 Hz)"]
        FUSED_TRACKS --> XFORM["Spatio-Temporal Motion Transformer"]
        ROAD_CTX["Road Boundaries & Camber"] --> XFORM
        XFORM --> PRED_TRAJ["5-Second Trajectory Forecasts (5x10)
        [X(1s..5s), Y(1s..5s)]"]
    end

    subgraph S4 ["4. Dynamic Risk Assessment (10 Hz)"]
        FUSED_TRACKS --> IRRM["7-Factor Indian Road Risk Metric (IRRM)"]
        PRED_TRAJ --> IRRM
        EGO_FB["Ego State Feedback [x, y, v, yaw, r]"] --> IRRM
        WEATHER_CTX["Weather (mu=0.5-0.85) & Width"] --> IRRM
        IRRM --> THREAT_OBS["Threat Obstacle Vector (1x7)
        [d, vrel, x, y, class, RiskScore, TTC]"]
        IRRM --> COSTMAP["Dynamic Spatio-Temporal Costmap"]
    end

    subgraph S5 ["5. Adaptive Path Planner (10 Hz Continuous Replan)"]
        THREAT_OBS --> PLANNER["Tri-Hybrid Planner Coordinator"]
        COSTMAP --> PLANNER
        EGO_FB --> PLANNER
        GOAL["Target Goal [x_g, y_g, yaw_g]"] --> PLANNER
        PLANNER --> PPO["PPO Tactical Negotiation Policy"]
        PPO --> HASTAR["Hybrid A* Kinematic Corridor Guide"]
        HASTAR --> DWA["Dynamic Window Approach (DWA)"]
        DWA --> LOCAL_TRAJ["Local Trajectory Waypoints (20x3)"]
        DWA --> PLAN_CMD["Planner Directives (V_target, Nudge)"]
    end

    subgraph S6 ["6. Decision Engine (10 Hz Stateflow)"]
        THREAT_OBS --> SF["Stateflow 8-State Tactical Supervisor"]
        EGO_FB --> SF
        PLAN_CMD --> SF
        ENV_SIG["Traffic Signals & Clearance"] --> SF
        SF --> MANEUVER["Tactical Decision State:
        1:Accelerate | 2:Brake | 3:Overtake
        4:Wait | 5:Stop | 6:EmergencyStop
        7:MoveLeft | 8:MoveRight"]
        SF --> SUP_CMD["Supervised Targets:
        [V_cmd, a_cmd, LatOffset, Horn, Hazards]"]
    end

    subgraph S7 ["7. Vehicle Controller (100 Hz Loop)"]
        LOCAL_TRAJ --> CTRL["Vehicle Motion Controller Engine"]
        SUP_CMD --> CTRL
        EGO_FB --> CTRL
        CTRL --> STANLEY["Adaptive Stanley Steering Controller
        delta = e_theta + arctan(-k*ey / (v + k_soft)) + delta_ff - kd*r"]
        CTRL --> LON_PID["Longitudinal Speed PID with Anti-Windup"]
        STANLEY --> STEER_CMD["SteerCommand delta [-31.5°, +31.5°]"]
        LON_PID --> ACCEL_CMD["AccelCommand a [-6.5, +3.5 m/s²]"]
        LON_PID --> BRAKE_CMD["BrakePressure P [0, 100 bar]"]
    end

    subgraph S8 ["8. Vehicle Dynamics Plant (100 Hz Discrete Plant)"]
        STEER_CMD --> PLANT["Tata Nexon EV 2-DOF Plant Model"]
        ACCEL_CMD --> PLANT
        BRAKE_CMD --> PLANT
        ROAD_MU["Road Friction mu"] --> PLANT
        PLANT --> TIRES["Non-Linear Pacejka Tire Slip Forces
        (alpha_f, alpha_r, Fy_f, Fy_r)"]
        TIRES --> PLANT
        PLANT --> UPDATED_EGO["Updated Vehicle State (BusVehicleState)
        [x, y, v, yaw, yaw_rate, accel, steer, slip]"]
    end

    UPDATED_EGO -.->|Closed-Loop Ego State Feedback| EGO_FB
    UPDATED_EGO -.->|Pose for Sensor Raycasting| S1
```

---

## 2. Comprehensive Connection & Signal Line Specification

The table below explains **every single signal connection** in the master Simulink model:

| # | Source Subsystem & Port | Destination Subsystem & Port | Signal / Bus Name | Data Type & Dimensions | Units | Physical & Mathematical Explanation |
|---|---|---|---|---|---|---|
| **C1** | `1_Sensor_Suite/1` | `2_Detection_and_Fusion/1` | `CameraDetections` | `double [5 x 5]` | pixels / m | Normalized bounding boxes $[u, v, w, h]$ and YOLO class predictions for up to 5 detected objects. |
| **C2** | `1_Sensor_Suite/2` | `2_Detection_and_Fusion/2` | `LiDARClusters` | `double [5 x 4]` | m, counts | Clustered 3D point cloud centroids $[x_{rel}, y_{rel}, z_{rel}]$ and return intensity for spatial ground-truth localization. |
| **C3** | `1_Sensor_Suite/3` | `2_Detection_and_Fusion/3` | `RadarTargets` | `double [5 x 4]` | m, m/s, deg, dB | Direct Doppler radar measurements $[r, \dot{r}, \phi, \text{RCS}]$ providing instantaneous range rate and velocity. |
| **C4** | `2_Detection_and_Fusion/1` | `3_Trajectory_Prediction/1` | `FusedTracks` | `double [5 x 7]` | m, m/s, ID, % | Fused obstacle states: $[x, y, v_x, v_y, \text{class\_id}, \text{confidence}, \text{distance}]$ from Camera-LiDAR-Radar EKF fusion. |
| **C5** | `2_Detection_and_Fusion/1` | `4_Risk_Assessment/1` | `FusedTracks` | `double [5 x 7]` | m, m/s | Feeds current obstacle positions and velocities into the 7-factor IRRM risk evaluation engine. |
| **C6** | `3_Trajectory_Prediction/1` | `4_Risk_Assessment/2` | `PredictedTrajectories` | `double [5 x 10]` | m | 5-second multi-agent trajectory forecasts $[X(1s..5s), Y(1s..5s)]$ enabling predictive time-to-collision and spatial corridor risk mapping. |
| **C7** | `4_Risk_Assessment/1` | `5_Path_Planner/3` | `ThreatObstacle` | `double [1 x 7]` | m, m/s, ID, score, s | Primary threat vector $[d_{threat}, v_{rel}, x, y, \text{class}, \mathcal{R}_{score}, TTC]$. Signals PPO for tactical lateral nudging. |
| **C8** | `4_Risk_Assessment/1` | `6_Decision_Engine/3` | `ThreatObstacle` | `double [1 x 7]` | m, m/s, score, s | Evaluated by Stateflow for priority transitions: triggers `Emergency Stop` if $TTC \le 1.1\text{ s}$ or $\mathcal{R} \ge 0.88$; triggers `Wait` for cattle. |
| **C9** | `5_Path_Planner/1` | `7_Vehicle_Controller/3` | `LocalTrajectoryWaypoints` | `double [20 x 3]` | m, rad | Reference path waypoints $[x_{ref}, y_{ref}, \theta_{ref}]$ computed by Hybrid A* and DWA with PPO tactical nudge for Stanley tracking. |
| **C10** | `5_Path_Planner/2` | `6_Decision_Engine/2` | `PlannerVelocity` | `double [1 x 1]` | m/s | Nominal velocity proposal from DWA continuous replanning, subject to Stateflow tactical supervisor confirmation. |
| **C11** | `6_Decision_Engine/1` | `7_Vehicle_Controller/2` | `DecisionCommandBus` | `Bus: BusDecisionOutput` | m/s, m/s², m, bool | Stateflow tactical directives: `[State, TargetSpeed, TargetAccel, TargetLateralOffset, Horn, HazardLights]`. |
| **C12** | `7_Vehicle_Controller/1` | `8_Vehicle_Dynamics/1` | `ControlCommandBus` | `Bus: BusControlCommand` | rad, m/s², bar | Low-level actuator signals: `SteerCommand` ($\delta$), `AccelCommand` ($a$), and `BrakePressure` ($P$) feeding Tata Nexon EV powertrain and brakes. |
| **C13** | `8_Vehicle_Dynamics/1` | `1_Sensor_Suite/1` | `EgoVehicleState` (Feedback) | `Bus: BusVehicleState` | m, m/s, rad, rad/s | Closed-loop vehicle pose and velocity feedback for sensor raycasting and coordinate frame transforms. |
| **C14** | `8_Vehicle_Dynamics/1` | `4_Risk_Assessment/3` | `EgoVehicleState` (Feedback) | `Bus: BusVehicleState` | m/s, rad/s | Real-time vehicle speed and heading for exact time-to-collision and dynamic approach angle calculation. |
| **C15** | `8_Vehicle_Dynamics/1` | `5_Path_Planner/1` | `EgoVehicleState` (Feedback) | `Bus: BusVehicleState` | m, m/s, rad | Starting origin state for continuous Hybrid A* search and DWA velocity window $(V_d)$ formulation. |
| **C16** | `8_Vehicle_Dynamics/1` | `6_Decision_Engine/1` | `EgoVehicleState` (Feedback) | `Bus: BusVehicleState` | m/s | Velocity feedback for zero-speed confirmation in `Wait` and `Stop` states. |
| **C17** | `8_Vehicle_Dynamics/1` | `7_Vehicle_Controller/1` | `EgoVehicleState` (Feedback) | `Bus: BusVehicleState` | m, m/s, rad, rad/s | Front axle state $[x_f, y_f]$ and yaw rate $\dot{\psi}$ feedback for Stanley error computation and longitudinal PID error tracking. |

---

## 3. Multi-Rate Execution & Sample Time Architecture

The system operates across three optimized time domains:

| Time Domain | Rate | Sample Time $T_s$ | Subsystems | Engineering Rationale |
|---|---|---|---|---|
| **Perception Domain** | $20\text{ Hz}$ | $0.05\text{ s}$ | `Sensor Suite`, `Detection & Fusion` | Matches camera sensor frame rate (20-30 FPS) and LiDAR scan period (50ms). |
| **Tactical Planning Domain** | $10\text{ Hz}$ | $0.10\text{ s}$ | `Trajectory Prediction`, `Risk Assessment`, `Path Planner`, `Decision Engine` | Allows 100ms budget for Transformer inference, costmap update, PPO evaluation, and DWA rollout. |
| **Actuation & Dynamics Domain** | $100\text{ Hz}$ | $0.01\text{ s}$ | `Vehicle Controller`, `Vehicle Dynamics Plant` | Ensures fast numerical integration of tire slip dynamics ($\alpha_f, \alpha_r$) and zero-delay steering rate limiting. |

**Rate Transition Handling**:
- Simulink `Rate Transition` blocks (configured with Zero-Order Hold) buffer signals between the 10 Hz planning domain and the 100 Hz vehicle controller, eliminating jitter and numerical algebraic loops.

---

## 4. Fail-Safe Closed-Loop Guarantees

1. **Preemptive Brake Channel**:
   If a high-velocity hazard (e.g. calf sprints or auto-rickshaw cuts in) occurs between 10 Hz replan cycles, the 100 Hz Vehicle Controller directly monitors proximity and triggers **Autonomous Emergency Braking (AEB)** ($a = -6.5\text{ m/s}^2$) with zero pipeline latency.
2. **Dynamic Road Friction Adaptation**:
   When transitioning from dry asphalt ($\mu = 0.85$) to monsoon wet road ($\mu = 0.50$), `Risk Assessment` immediately inflates stopping distance bounds, `Decision Engine` inhibits high-speed overtaking, and `Vehicle Dynamics` scales tire cornering saturation thresholds.
