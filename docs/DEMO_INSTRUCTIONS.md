# BharatDrive-AI: Interactive Demonstration Instructions

This guide provides step-by-step instructions for demonstrating **BharatDrive-AI** via the **Interactive Web Dashboard** and the **MATLAB App Designer Interface**.

---

## 1. Quick Start: Interactive Web Dashboard

### Step 1: Start the Local Telemetry Web Server
From the root of the repository:
```bash
python -m http.server 8085
```

### Step 2: Open the Dashboard in Any Modern Browser
Navigate to:
```
http://localhost:8085/dashboard/index.html
```

---

## 2. Web Dashboard Interface Overview

```text
┌────────────────────────────────────────────────────────────────────────────────────────┐
│ BHARATDRIVE-AI TELEMETRY DASHBOARD LAYOUT                                              │
├─────────────────────┬────────────────────────────────────────────┬─────────────────────┤
│ LEFT COLUMN         │ CENTER COLUMN (BIRD'S-EYE VIEW)            │ RIGHT COLUMN        │
├─────────────────────┼────────────────────────────────────────────┼─────────────────────┤
│ 1. Vehicle Speed    │ 1. 2D Bird's-Eye View (BEV) Canvas         │ 1. Current Risk     │
│    - SVG Speedometer│    - Asphalt carriageway & shoulders       │    - 7-Factor IRRM  │
│    - Digital readout│    - Tata Nexon EV with sensor FOV cones   │    - Criticality bar│
│    - Target cruise  │    - Hybrid A* global corridor guide       │    - Action callout │
│ 2. Collision Warning│    - DWA predictive trajectory rollout     │ 2. Fused Objects    │
│    - 4-Tier banner  │    - Tactical lateral nudge vectors        │    - Tabular tracks │
│    - TTC Countdown  │    - Heterogeneous obstacle boxes (Cattle, │    - Semantic class │
│ 3. Sensor Diagnostics│      Auto-rickshaws, Trucks, Potholes)     │    - Range & VRel   │
│    - Cam, LiDAR,    │ 2. Simulation Playback Control Bar         │    - Active sensors │
│      Radar, GNSS/IMU│    - Play / Pause / Step / Reset / AEB     │    - Risk impact    │
└─────────────────────┴────────────────────────────────────────────┴─────────────────────┘
```

---

## 3. Scenario Demonstration Walkthrough

Use the **Scenario Dropdown** in the top navigation bar to select and run each test scenario:

### Scenario 5: Cattle Crossing (Bovine Obstruction) — Tactical Nudge Demo
1. Select **"Scenario 5: Cattle Crossing (Bovine Obstruction)"**.
2. Click **"▶ Play"**.
3. **What to Observe**:
   - The ego vehicle cruises along the rural two-lane road at 30 km/h.
   - At 45 meters ahead, the sensor fusion engine registers a **stationary cow** (Object ID 1, Class: Cattle) directly blocking the lane, and an **oncoming bus** in the opposite lane.
   - The IRRM score rises to 0.42 (Medium risk).
   - The Stateflow engine initiates a **"Tactical Nudge Left"** ($d_{\text{target}} = +1.25\text{ m}$).
   - Notice the vehicle glides smoothly into the available road space without abrupt braking, maintaining a **1.45 m margin** from the cow and a **1.37 m clearance** from the bus.

### Scenario 6: Mixed Hazard Master — Compound Multi-Threat Negotiation
1. Select **"Scenario 6: Mixed Hazard Master"**.
2. Click **"▶ Play"**.
3. **What to Observe**:
   - Simultaneous compound hazards: a stationary cow in the lane, an aggressive auto-rickshaw executing a sudden cut-in from the left, and a 10 cm deep pothole.
   - Watch the 7-Factor Risk bar chart: Multi-Agent Density and Kinetic threat spike.
   - The vehicle executes controlled deceleration to 18 km/h to yield to the auto-rickshaw, micro-steers by 0.35 m to straddle the pothole between its wheel tracks, and completes the bypass around the cow.

### Scenario 1: Village Road (Gramin Sadak) — Soft Shoulder Passing
1. Select **"Scenario 1: Village Road (Gramin Sadak - Tractor & Pedestrian)"**.
2. Click **"▶ Play"**.
3. **What to Observe**:
   - Narrow single-lane paved road (3.7 m) with an oncoming tractor.
   - The vehicle smoothly utilizes the gravel shoulder envelope while maintaining safe distance from a walking pedestrian.

---

## 4. Demonstrating the Evaluation Metrics & Graphs Modal

1. In the top navigation bar, click the glowing cyan button: **"📊 Evaluation Metrics"**.
2. A sleek glassmorphic modal opens over the simulation.
3. **Inspect the 7 KPI Summary Cards**:
   - **Collision Rate**: `0.0%` (vs 16.7% Baseline 2, 66.7% Baseline 1)
   - **Average Speed**: `33.8 km/h` (67.6% Cruise Efficiency)
   - **Scenario Completion**: `100%` (6 / 6 finished)
   - **Avg Replanning Time**: `12.2 ms` (P99: 18.9 ms vs 100 ms deadline)
   - **Path Smoothness**: `0.89 m/s³` (ISO 2631 Passenger Comfort: 92/100)
   - **Minimum Clearance**: `1.37 m` (1.0 m buffer strictly preserved)
   - **CPU Usage**: `40.0%` (20.0 ms per 50 ms loop budget at 20 Hz)
4. **Toggle Interactive Graph Tabs**:
   - Click **"Master Summary Dashboard"** to see the 6-panel executive chart.
   - Click **"Clearance & Collision Rate"** to inspect edge-to-edge spatial margins.
   - Click **"Replanning Latency Distribution"** to view P50/P95/P99 latency vs 100 ms limit.
   - Click **"CPU Breakdown by Subsystem"** to examine the compute footprint pie chart.
5. **Review Empirical Benchmark Table**: Scroll down to examine the detailed comparative metrics across all 3 stacks.
6. Click **"×"** or press **ESC** to close the modal and resume the simulation.

---

## 5. Native MATLAB App Designer Demonstration

For native MathWorks environment demonstrations:
1. Open MATLAB.
2. Run:
   ```matlab
   startup
   run_dashboard
   ```
3. An App Designer window launches featuring:
   - High-fidelity circular speed gauge.
   - Dynamic 7-factor IRRM risk bar chart.
   - Bird's-Eye View road axes with live Hybrid A* trajectory rollouts.
   - Active sensor diagnostics and detected objects table.
