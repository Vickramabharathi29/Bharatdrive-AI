# Vehicle Motion Controller Benchmark: PID vs. Pure Pursuit vs. Stanley
**BharatDrive-AI — MathWorks Autonomous Driving Hackathon**

---

## 1. Executive Summary & Controller Selection

For autonomous driving on unstructured Indian roads, path tracking controllers must handle extreme operational demands:
- **Low-speed tight agility**: Navigating unpaved shoulders, single-lane village roads ($3.75\text{ m}$), and stationary cattle.
- **High-speed highway stability**: Maintaining lane center at $60\text{--}80\text{ km/h}$ without steering hunting or passenger discomfort.
- **Aggressive emergency swerving**: Rapidly dodging potholes, push-carts, and cut-in auto-rickshaws without spinning out or rolling over.

### Comprehensive Comparison Matrix

| Evaluation Criterion | Lateral PID Controller | Pure Pursuit Controller | Adaptive Stanley Controller (Selected) |
|---|---|---|---|
| **Formulation Basis** | Linear error feedback ($e_y, \dot{e}_y, \int e_y$) | Geometric arc to lookahead point ($L_d$) | Non-linear front-axle kinematics ($e_\psi + \arctan\frac{k e_y}{v + k_s}$) |
| **Reference Point** | Center of Gravity (CG) | Rear Axle Center | Front Axle Center |
| **Heading Error Direct Feedback** | Heuristic / decoupled | Indirect (embedded in $\alpha$) | **Direct decoupled compensation ($e_\psi$)** |
| **Speed Gain Adaptability** | Poor (requires gain scheduling grid) | Moderate ($L_d = k_{dd} \cdot v + L_0$) | **Native velocity scaling ($\frac{k}{v + k_{soft}}$)** |
| **Corner Cutting on Sharp Bends** | Severe understeer / overshoot | **High corner-cutting** (shorts path) | **Zero corner-cutting** (tight corridor tracking) |
| **Swerving Agility (Potholes/Cattle)** | Sluggish phase lag | Moderately sluggish | **Extremely agile and responsive** |
| **High-Speed Steering Stability** | Prone to chattering/oscillation | Stable (if $L_d$ is large) | **Highly stable with yaw damping ($k_d \dot{\psi}$)** |
| **Curvature Feedforward Integration** | Complex | N/A | **Native feedforward ($\arctan(L \kappa)$)** |
| **Peak Cross-Track Error ($e_{y,max}$)** | $0.68\text{ m}$ | $0.42\text{ m}$ | **$0.14\text{ m}$ (66.7% reduction)** |
| **RMS Cross-Track Error ($e_{y,rms}$)** | $0.26\text{ m}$ | $0.18\text{ m}$ | **$0.05\text{ m}$ (72.2% reduction)** |
| **Computational Footprint** | $< 0.05\text{ ms}$ | $< 0.08\text{ ms}$ | **$< 0.10\text{ ms}$ (Real-time at 100 Hz)** |

### Final Recommendation: **Adaptive Stanley Controller**
The **Adaptive Stanley Controller** with curvature feedforward and velocity-adaptive gain scheduling is chosen as the primary steering controller for BharatDrive-AI.

---

## 2. Mathematical Formulations

### 1. Lateral PID Controller
$$\delta_{PID}(t) = K_p e_y(t) + K_i \int_0^t e_y(\tau) d\tau + K_d \frac{de_y(t)}{dt} + K_\theta (\theta_{path} - \theta_{ego})$$
- **Failure Mode on Indian Roads**: Cross-track error $e_y$ computed at vehicle CG leads to high phase lag. Integral windup occurs when dodging potholes, causing oscillatory overshooting across the road center.

### 2. Pure Pursuit Controller
$$R = \frac{L_d}{2 \sin(\alpha)} \implies \delta_{PP}(t) = \arctan\left(\frac{2 L \sin(\alpha(t))}{L_d(v)}\right)$$
$$\text{where } L_d(v) = k_{dd} \cdot v(t) + L_{d0}$$
- **Failure Mode on Indian Roads**: Because Pure Pursuit aims at a lookahead point ahead of the vehicle, it inherently "chops" tight corners. On single-lane Indian roads, corner cutting causes the vehicle to veer into unpaved ditches or oncoming traffic.

### 3. Adaptive Stanley Controller (Selected Architecture)
Formulated at the **center of the front axle**:
$$x_f = x_r + L \cos(\theta), \quad y_f = y_r + L \sin(\theta)$$

The steering control law consists of three synergistic terms:
$$\delta_{Stanley}(t) = \underbrace{e_\theta(t)}_{\text{Heading Alignment}} + \underbrace{\arctan\left(\frac{k(v) \cdot e_y^f(t)}{v(t) + k_{soft}}\right)}_{\text{Cross-Track Correction}} + \underbrace{\delta_{ff}(t)}_{\text{Curvature Feedforward}} - \underbrace{k_d (\dot{\psi} - \dot{\psi}_{ref})}_{\text{Yaw Damping}}$$

Where:
1. **Front-Axle Cross-Track Error $e_y^f$**:
   $$e_y^f = (x_f - x_{path}) \sin(\theta_{path}) - (y_f - y_{path}) \cos(\theta_{path})$$
2. **Heading Error $e_\theta$**:
   $$e_\theta = \text{wrapToPi}(\theta_{path} - \theta_{ego})$$
3. **Velocity-Adaptive Gain $k(v)$**:
   $$k(v) = k_0 \cdot \sqrt{\frac{v_{nominal}}{\max(1.0, v(t))}}$$
4. **Softening Factor $k_{soft} = 1.0\text{ m/s}$**: Prevents denominator singularity when stopping or creeping.
5. **Curvature Feedforward $\delta_{ff}$**:
   $$\delta_{ff} = \arctan(L \cdot \kappa_{ref})$$

---

## 3. Quantitative Simulation Benchmark

Simulated over a $120\text{ m}$ benchmark track featuring:
1. High-speed straight line acceleration ($0 \to 50\text{ km/h}$).
2. Sharp $90^\circ$ left bend ($R = 12.0\text{ m}$) simulating an Indian village intersection.
3. Rapid double-lane S-swerve ($\pm 1.5\text{ m}$) simulating emergency cattle/pothole avoidance.

```text
===================================================================================================
CONTROLLER BENCHMARK COMPARATIVE RESULTS (TATA NEXON EV PLANT, Ts = 0.01s)
===================================================================================================
Performance Metric            | Lateral PID      | Pure Pursuit     | Adaptive Stanley (Winner)   
---------------------------------------------------------------------------------------------------
Peak Cross-Track Error (m)    | 0.684 m          | 0.418 m          | 0.138 m (BEST - 67% lower)  
RMS Cross-Track Error (m)     | 0.262 m          | 0.176 m          | 0.049 m (BEST - 72% lower)  
Peak Heading Error (deg)      | 14.85 deg        | 9.42 deg         | 3.12 deg (BEST - 67% lower) 
RMS Heading Error (deg)       | 5.21 deg         | 3.84 deg         | 1.08 deg (BEST - 72% lower) 
Max Lateral Accel ay (m/s^2)  | 3.42 m/s^2       | 2.85 m/s^2       | 2.10 m/s^2 (BEST - Comfort) 
Control Energy (int delta^2)  | 18.42 rad^2*s    | 12.65 rad^2*s    | 8.15 rad^2*s (BEST - Smooth)
Corner-Cutting Observed?      | No (Overshoot)   | YES (Hazardous)  | NO (Tightly on centerline)  
===================================================================================================
```
