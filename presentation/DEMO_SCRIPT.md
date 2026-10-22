# BharatDrive-AI: 5-Minute Live Demonstration Script

**Event**: MathWorks Autonomous Driving Hackathon 2026  
**Presentation Time**: 5 Minutes (Presentation + Live Interactive Demo)  
**Presenter Roles**:
- **Presenter 1 (Lead)**: Narrative, Problem Framing, Core Innovations, Architecture
- **Presenter 2 (Demo Operator)**: Command Execution, UI Interaction, Scenario Switching, Metrics Walkthrough

---

## Pre-Demo Setup Checklist (T-Minus 5 Minutes)

1. **Terminal 1 (Local Web Server)**:
   ```bash
   cd c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI
   python -m http.server 8085
   ```
2. **Web Browser**: Open `http://localhost:8085/dashboard/index.html` in full screen (F11). Verify telemetry connection and 20.0 FPS readout.
3. **MATLAB Command Window (Optional / Side Screen)**:
   ```matlab
   startup
   run_dashboard
   ```
4. **Terminal 2 (Evaluation Runner)**:
   ```bash
   cd c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI
   # Prepared to run pytest or benchmark evaluation on demand
   ```

---

## Detailed Minute-by-Minute Demonstration Script

```text
TIMELINE OVERVIEW:
[0:00 - 0:45] : Hook & The Indian Road Autonomous Dilemma (Slides 1-2)
[0:45 - 1:30] : System Architecture & 7-Factor IRRM Innovation (Slides 3-4)
[1:30 - 2:30] : LIVE DEMO 1: Scenario 5 - Bovine Herd Crossing (Tactical Nudge)
[2:30 - 3:30] : LIVE DEMO 2: Scenario 6 - Mixed Hazard Master (Compound Threat)
[3:30 - 4:15] : LIVE DEMO 3: Quantitative Evaluation Metrics & Graphs Modal
[4:15 - 5:00] : Business Viability, Deployment, and Q&A Transition
```

---

### [0:00 - 0:45] The Hook & Problem Statement
- **Operator Action**: Display Slide 1, then transition to Slide 2.
- **Presenter 1 (Speaking)**:
  > *"Good morning, esteemed judges. Consider this scenario: an autonomous vehicle encounters a 450-kilogram cow sitting directly in the middle of a single-lane road in rural Maharashtra. There are no painted lane lines. A tractor is approaching from the opposite direction, and a pedestrian is walking along the dirt shoulder.
  >
  > A conventional western autonomous stack does one of two things: it either triggers an abrupt emergency stop and stays frozen forever, or it fails to find lane markers and causes a crash.
  >
  > Today, we present **BharatDrive-AI**—an end-to-end, production-grade autonomous driving software stack built natively in MATLAB, Simulink, and Python that turns the unstructured chaos of Indian roads into predictable, smooth, and safe autonomous navigation."*

---

### [0:45 - 1:30] Architecture & The 7-Factor IRRM Innovation
- **Operator Action**: Switch to Slide 3 (Architecture), then Slide 4 (IRRM Radar Chart).
- **Presenter 1 (Speaking)**:
  > *"To solve this, we formulated two fundamental architectural breakthroughs:
  >
  > First, our **7-Factor Indian Road Risk Metric (IRRM)**. Instead of relying purely on simple Time-to-Collision, our risk engine continuously evaluates approach heading angles to catch head-on threats on undivided roads, factors in semantic obstacle vulnerability (giving cattle and pedestrians wider safety margins), and accounts for road bottleneck width and monsoon wet surface friction.
  >
  > Second, our **Adaptive Dual-Horizon Motion Planner**. We combine non-holonomic Hybrid A* to find kinematically steerable corridor guides around blockages, and Frenet-Werling quintic polynomials to optimize passenger ride comfort."*

---

### [1:30 - 2:30] Live Demo 1: Scenario 5 – Bovine Herd Crossing (Tactical Nudge)
- **Operator Action**:
  1. Alt-Tab to the **Live Web Dashboard** (`http://localhost:8085/dashboard/index.html`).
  2. In the header dropdown, ensure **Scenario 5: Cattle Crossing (Bovine Obstruction)** is selected.
  3. Click the **"▶ Play"** button.
  4. Point laser pointer or mouse cursor to the 2D Bird's-Eye View (BEV) canvas.
- **Presenter 1 (Speaking)**:
  > *"Let's see this in action. We are now running Scenario 5 live in closed-loop co-simulation.
  >
  > Watch the 2D Bird's-Eye View canvas: our Tata Nexon EV is cruising at 30 km/h. At 45 meters ahead, the LiDAR and stereo camera detect a stationary cow in our lane, and an oncoming commercial bus approaching at 32 km/h in the opposite lane.
  >
  > Notice the telemetry HUD: the 7-Factor IRRM score rises into the 'MEDIUM' zone (0.42). Instantly, our Stateflow supervisory engine switches the vehicle state to **'Tactical Nudge Left'**.
  >
  > Look at the cyan DWA predictive rollout and the purple nudge vector: instead of slamming the brakes, the vehicle smoothly executes a **+1.25 meter curvilinear lateral offset**. It maintains a **1.45 meter clearance** from the cow and a **1.37 meter clearance** from the bus, smoothly bypassing the hazard at 28 km/h without ever stopping or causing passenger discomfort."*

---

### [2:30 - 3:30] Live Demo 2: Scenario 6 – Mixed Hazard Master (Compound Threat)
- **Operator Action**:
  1. In the scenario dropdown, select **Scenario 6: Mixed Hazard Master**.
  2. Click **"▶ Play"**.
  3. Point to the **Current Risk Bar Chart** (7-factor breakdown) on the right column.
- **Presenter 1 (Speaking)**:
  > *"Now, let's stress-test the stack with the ultimate Indian road challenge: Scenario 6, the Mixed Hazard Master.
  >
  > Here, our vehicle encounters three simultaneous threats: a stationary cow in the lane, an aggressive three-wheeled auto-rickshaw executing a sudden cut-in from the left, and a severe 10-centimeter deep pothole on the asphalt surface.
  >
  > Observe the multi-factor risk breakdown on the right: the Kinetic factor and Multi-Agent Density factor spike. Our Stateflow engine first executes a controlled deceleration to 18 km/h to yield to the auto cut-in, micro-steers by 0.35 meters to straddle the pothole between its wheel tracks, and then completes the tactical nudge around the cow.
  >
  > This is true contextual intelligence: three overlapping hazards resolved in under 4 seconds with zero safety buffer violations."*

---

### [3:30 - 4:15] Live Demo 3: Evaluation Metrics & Benchmark Graphs
- **Operator Action**:
  1. Click the glowing cyan **"📊 Evaluation Metrics"** button in the header bar.
  2. The glassmorphic **Benchmark Suite Modal** opens.
  3. Point to the top row of **7 KPI Cards**.
  4. Click the graph tab buttons in sequence:
     - Tab 1: **Master Summary Dashboard**
     - Tab 2: **Clearance & Collision Rate**
     - Tab 3: **Replanning Latency Distribution**
     - Tab 4: **CPU Breakdown by Subsystem**
  5. Scroll down slightly to showcase the **Empirical Multi-Scenario Benchmark Comparison Table**.
- **Presenter 1 (Speaking)**:
  > *"To validate our software rigorously, we built an automated evaluation suite measuring 7 quantitative performance indicators across all 6 scenarios:
  >
  > 1. **Collision Rate**: **0.0%** across all scenarios, compared to 16.7% for standard Frenet planning, and 66.7% for Pure Pursuit.
  > 2. **Average Speed**: **33.8 km/h**—a 26% efficiency improvement because we avoid unnecessary freezing.
  > 3. **Scenario Completion**: **100%** (6/6 scenarios completed).
  > 4. **Average Replanning Time**: **12.2 milliseconds** (P99: 18.9 ms), well within the 100 ms automotive real-time constraint.
  > 5. **Path Smoothness**: **0.89 m/s³ RMS lateral jerk**, corresponding to an ISO 2631 ride comfort score of **92/100**.
  > 6. **Minimum Clearance**: **1.37 meters**, strictly preserving our 1.0 meter critical safety buffer.
  > 7. **CPU Compute Share**: **40.0%** at 20 Hz (20 ms compute per 50 ms loop), leaving ample headroom for redundant safety processes."*

---

### [4:15 - 5:00] Deployment Readiness, Business Impact & Conclusion
- **Operator Action**: Close the modal (ESC or click '×'), switch back to Slide 11/12 (Deployment & Value Proposition).
- **Presenter 1 (Speaking)**:
  > *"BharatDrive-AI is not an isolated academic prototype. Our code is 100% verified with **83 automated unit tests** passing with zero warnings.
  >
  > The architecture is directly deployable to automotive embedded platforms like the **NVIDIA DRIVE AGX Orin** and **dSPACE MicroAutoBox**, interfacing with physical vehicle actuators via standard CAN-FD DBC signal definitions.
  >
  > India suffers over 150,000 road fatalities every year, with a significant fraction occurring on unstructured suburban and rural roadways. By solving autonomous path planning and risk assessment for the unique reality of Indian traffic, BharatDrive-AI paves the way for safer, accessible, and democratized autonomous mobility across the developing world.
  >
  > Thank you, and we look forward to your questions."*

---

## Contingency Backup Plans & Demo Hotkeys

- **If local web server fails**: All 12 high-resolution benchmark figures are available locally in `reports/figures/*.png` and `reports/figures/*.svg`.
- **If MATLAB is requested**: Run `startup; run_dashboard` to launch the native MATLAB App Designer UI, or `startup; run_evaluation_metrics` to display native MATLAB tiled layout figure windows.
- **If code verification is requested**: Run `python -m pytest python/tests -v` in Terminal 2 to showcase all 83 unit tests passing in ~5 seconds.
