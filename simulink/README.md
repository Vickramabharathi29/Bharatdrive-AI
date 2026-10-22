# Simulink Models: Adaptive Path Planning and Collision Avoidance
**BharatDrive-AI — MathWorks Autonomous Driving Hackathon**

This directory contains the Simulink models, bus definitions, and MATLAB Function blocks for real-time continuous replanning on Indian roads.

---

## 1. Model Overview (`AdaptivePathPlanningSystem.slx`)

The `AdaptivePathPlanningSystem` implements the multi-tier tri-hybrid architecture:
1. **Inputs**:
   - `EgoVehicleState` (Bus: `BusVehicleState`): $[x, y, v, \text{yaw}, \dot{\psi}, a, \delta]$
   - `EnvironmentalRiskContext` (Bus: `BusRiskContext`): $[W_{road}, \mu_{friction}, \nu_{visibility}, \rho_{traffic}]$
   - `DetectedObstacles` ($5 \times 6$ matrix): $[x, y, v_x, v_y, \text{class\_id}, \mathcal{R}_{score}]$
   - `TargetGoal` ($3 \times 1$ vector): $[x_{goal}, y_{goal}, \theta_{goal}]$
2. **Subsystems**:
   - `PPO_Tactical_Negotiation_Policy`: Evaluates deep reinforcement learning policy at 10 Hz; produces tactical lateral nudge $\Delta d \in [-1.5\text{m}, +1.5\text{m}]$, speed factor $\alpha_v \in [0.25, 1.15]$, and dynamic risk weight $w_{risk} \in [0.5, 2.5]$.
   - `Hybrid_AStar_Corridor_Planner`: Calculates non-holonomic, curvature-bounded local target waypoint taking into account static blockages, cattle, and potholes.
   - `DWA_Continuous_Replanner`: Evaluates forward kinematic rollout arcs over dynamic acceleration bounds ($a_{accel} = 3.5\text{ m/s}^2, a_{decel} = 6.5\text{ m/s}^2$) with stopping distance admissibility checks.
   - `Emergency_Braking_Safety_Supervisor`: Monitors time-to-collision (TTC) and emergency perimeter ($< 4.0\text{ m}$ forward intrusion); triggers autonomous emergency braking (AEB) when critical threshold is exceeded.
3. **Outputs**:
   - `PlannerCommandBus` (Bus: `BusPlannerOutput`):
     - `TargetVelocity` ($v_{cmd}$ [m/s])
     - `TargetYawRate` ($\omega_{cmd}$ [rad/s])
     - `CommandedSteer` ($\delta_{cmd}$ [rad])
     - `CommandedAccel` ($a_{cmd}$ [m/s$^2$])
     - `TacticalNudge` ($\Delta d$ [m])
     - `RiskIndex` ($\mathcal{R}_{max}$)

---

## 2. Bus Objects & Signal Routing

Defined in `simulink/buses/setup_planner_buses.m`:

| Bus Object | Elements | Data Types | Units | Description |
|---|---|---|---|---|
| `BusVehicleState` | `x, y, v, yaw, yaw_rate, accel, steer` | double | m, m, m/s, rad, rad/s, m/s², rad | Tata Nexon EV vehicle odometry & state |
| `BusRiskContext` | `RoadWidth, FrictionMu, Visibility, TrafficDensity` | double | m, [0-1], [0-1], count | Dynamic environmental road & friction conditions |
| `BusPlannerOutput` | `TargetVelocity, TargetYawRate, CommandedSteer, CommandedAccel, TacticalNudge, RiskIndex` | double | m/s, rad/s, rad, m/s², m, [0-1] | High-frequency continuous replanning control directives |

---

## 3. How to Build & Simulate

In MATLAB Command Window:
```matlab
% 1. Setup paths
run('matlab/startup.m');

% 2. Initialize Bus objects
run('simulink/buses/setup_planner_buses.m');

% 3. Generate or open Simulink model
build_adaptive_planner_simulink();
open_system('AdaptivePathPlanningSystem');

% 4. Run closed-loop continuous replanning verification
run('matlab/scripts/run_adaptive_planning_simulation.m');
```
