/**
 * BHARATDRIVE-AI: SIMULATION & RISK ENGINE
 * Mathematical implementation of Indian Road Risk Metric (IRRM),
 * 2-DOF Bicycle Dynamics, Stateflow Decisions, and Path Planning.
 */

class SimulationEngine {
  constructor() {
    this.dt = 0.05; // 20 Hz simulation cycle
    this.simTime = 0.0;
    this.stepCount = 0;
    this.speedMultiplier = 1.0;

    // Vehicle Specifications (Tata Nexon EV)
    this.vehicle = {
      length: 3.993,
      width: 1.811,
      wheelbase: 2.498,
      maxSteer: 0.5236, // 30 deg
      maxAccel: 3.0,
      maxDecel: -7.5
    };

    // Ego State: [x, y, v, yaw, steer]
    this.ego = {
      x: 0.0,
      y: -1.75,
      v: 8.33, // m/s
      yaw: 0.0,
      steer: 0.0,
      accel: 0.0,
      targetSpeed: 13.89 // 50 km/h default
    };

    this.egoHistory = [];

    // Environment & Active Scenario
    this.activeScenarioId = 5;
    this.scenarioName = "Scenario 5: Cattle Crossing (Bovine Obstruction)";
    this.roadWidth = 7.00;
    this.roadLength = 250.0;
    this.surfaceMu = 0.82;
    this.visibility = 1.00;

    // Obstacles list
    this.obstacles = [];

    // Telemetry & Decisions
    this.currentRiskScore = 0.0;
    this.currentCriticality = "LOW";
    this.currentAction = "MAINTAIN_CRUISE";
    this.currentTTC = Infinity;
    this.primaryThreat = "Clear Roadway";
    this.stateflowState = "Accelerate";
    this.tacticalNudge = 0.0;

    // 7-Factor values
    this.riskFactors = [0.05, 0.05, 0.05, 0.05, 0.05, 0.05];

    // Paths
    this.guidePath = [];
    this.localTrajectory = [];

    // Load initial scenario
    this.loadScenario(5);
  }

  loadScenario(scenarioId) {
    this.activeScenarioId = scenarioId;
    this.simTime = 0.0;
    this.stepCount = 0;
    this.egoHistory = [];
    this.tacticalNudge = 0.0;

    switch (scenarioId) {
      case 1: // Village Road
        this.scenarioName = "Scenario 1: Village Road (Gramin Sadak)";
        this.roadWidth = 4.20;
        this.roadLength = 220.0;
        this.surfaceMu = 0.70;
        this.visibility = 0.90;
        this.ego.x = 0.0;
        this.ego.y = -0.80;
        this.ego.v = 8.33; // 30 km/h
        this.ego.yaw = 0.0;
        this.ego.steer = 0.0;
        this.ego.targetSpeed = 8.33;

        this.obstacles = [
          {
            id: 101, class: "Truck-Bus", name: "Sugarcane Tractor",
            x: 75.0, y: 0.8, vx: -4.5, vy: 0.0, length: 8.2, width: 2.3
          },
          {
            id: 102, class: "Pedestrian", name: "Villager on Shoulder",
            x: 45.0, y: -1.9, vx: 0.8, vy: 0.05, length: 0.6, width: 0.6
          }
        ];
        break;

      case 2: // Busy Market
        this.scenarioName = "Scenario 2: Busy Market (Bazaar Street)";
        this.roadWidth = 8.50;
        this.roadLength = 180.0;
        this.surfaceMu = 0.80;
        this.visibility = 0.95;
        this.ego.x = 0.0;
        this.ego.y = -1.50;
        this.ego.v = 5.56; // 20 km/h
        this.ego.yaw = 0.0;
        this.ego.steer = 0.0;
        this.ego.targetSpeed = 5.56;

        this.obstacles = [
          {
            id: 201, class: "Auto-Rickshaw", name: "Lead Auto-Rickshaw",
            x: 22.0, y: -1.8, vx: 3.8, vy: 0.15, length: 2.6, width: 1.3
          },
          {
            id: 202, class: "Two-Wheeler", name: "Filtering Motorbike",
            x: 35.0, y: 0.6, vx: 6.2, vy: -0.2, length: 1.9, width: 0.8
          },
          {
            id: 203, class: "Pedestrian", name: "Shopper Crossing",
            x: 18.0, y: -3.2, vx: 0.0, vy: 0.7, length: 0.5, width: 0.5
          }
        ];
        break;

      case 3: // Urban Junction
        this.scenarioName = "Scenario 3: Urban Junction (Chowk)";
        this.roadWidth = 9.00;
        this.roadLength = 200.0;
        this.surfaceMu = 0.82;
        this.visibility = 1.00;
        this.ego.x = 0.0;
        this.ego.y = -2.00;
        this.ego.v = 9.72; // 35 km/h
        this.ego.yaw = 0.0;
        this.ego.steer = 0.0;
        this.ego.targetSpeed = 9.72;

        this.obstacles = [
          {
            id: 301, class: "Two-Wheeler", name: "Cutting Scooter",
            x: 34.0, y: 3.5, vx: -2.0, vy: -2.4, length: 1.8, width: 0.7
          },
          {
            id: 302, class: "Auto-Rickshaw", name: "Turning Auto",
            x: 58.0, y: 0.0, vx: 3.0, vy: 0.0, length: 2.6, width: 1.3
          }
        ];
        break;

      case 4: // Highway Merge
        this.scenarioName = "Scenario 4: Highway Merge (National Highway)";
        this.roadWidth = 10.50;
        this.roadLength = 300.0;
        this.surfaceMu = 0.85;
        this.visibility = 1.00;
        this.ego.x = 0.0;
        this.ego.y = -3.00;
        this.ego.v = 13.89; // 50 km/h
        this.ego.yaw = 0.0;
        this.ego.steer = 0.0;
        this.ego.targetSpeed = 16.67; // 60 km/h

        this.obstacles = [
          {
            id: 401, class: "Truck-Bus", name: "Heavy Multi-Axle Truck",
            x: 80.0, y: 0.0, vx: 13.5, vy: 0.0, length: 11.5, width: 2.6
          },
          {
            id: 402, class: "Car", name: "Fast Overtaking Car",
            x: 35.0, y: -0.8, vx: 18.0, vy: -0.3, length: 4.2, width: 1.8
          }
        ];
        break;

      case 5: // Cattle Crossing
      default:
        this.scenarioName = "Scenario 5: Cattle Crossing (Bovine Obstruction)";
        this.roadWidth = 7.00;
        this.roadLength = 250.0;
        this.surfaceMu = 0.82;
        this.visibility = 1.00;
        this.ego.x = 0.0;
        this.ego.y = -1.75;
        this.ego.v = 11.11; // 40 km/h
        this.ego.yaw = 0.0;
        this.ego.steer = 0.0;
        this.ego.targetSpeed = 13.89; // 50 km/h

        this.obstacles = [
          {
            id: 501, class: "Cattle", name: "Stationary Lead Cow",
            x: 36.0, y: -1.4, vx: 0.0, vy: 0.0, length: 2.2, width: 1.1
          },
          {
            id: 502, class: "Cattle", name: "Calf Crossing",
            x: 32.0, y: -3.2, vx: 0.0, vy: 1.1, length: 1.4, width: 0.7
          },
          {
            id: 503, class: "Truck-Bus", name: "Oncoming Commercial Bus",
            x: 120.0, y: 1.75, vx: -12.5, vy: 0.0, length: 10.5, width: 2.5
          }
        ];
        break;

      case 6: // Mixed Hazard Master
        this.scenarioName = "Scenario 6: Mixed Hazard Master";
        this.roadWidth = 7.00;
        this.roadLength = 260.0;
        this.surfaceMu = 0.82;
        this.visibility = 0.95;
        this.ego.x = 0.0;
        this.ego.y = -1.75;
        this.ego.v = 10.0;
        this.ego.yaw = 0.0;
        this.ego.steer = 0.0;
        this.ego.targetSpeed = 13.89;

        this.obstacles = [
          {
            id: 601, class: "Cattle", name: "Stationary Cow",
            x: 28.0, y: 0.4, vx: 0.0, vy: 0.0, length: 2.2, width: 1.1
          },
          {
            id: 602, class: "Auto-Rickshaw", name: "Aggressive Auto Cut-In",
            x: 48.0, y: -1.8, vx: 6.5, vy: 0.25, length: 2.6, width: 1.3
          },
          {
            id: 603, class: "Pothole", name: "Deep Surface Pothole",
            x: 72.0, y: -1.5, vx: 0.0, vy: 0.0, length: 1.3, width: 1.0
          },
          {
            id: 604, class: "Truck-Bus", name: "Slow Commercial Truck",
            x: 110.0, y: 0.0, vx: 4.0, vy: 0.0, length: 7.5, width: 2.4
          }
        ];
        break;
    }

    this.generatePaths();
  }

  step() {
    this.stepCount++;
    const dt = this.dt;
    this.simTime += dt;

    // 1. Dynamic Obstacle Motion Update
    for (let ob of this.obstacles) {
      ob.x += ob.vx * dt;
      ob.y += ob.vy * dt;

      // Obstacle boundary / wandering logic
      if (ob.class === "Cattle" && ob.vy > 0 && ob.y >= 0.0) {
        ob.vy = 0.0; // Cow stops at center lane
      }
      if (ob.class === "Auto-Rickshaw" && ob.vy > 0 && ob.y >= -0.2) {
        ob.vy = 0.0;
      }
    }

    // 2. Sensor Suite & Perception Tracking
    let maxRisk = 0.0;
    let minTTC = Infinity;
    let primaryThreat = "Clear Roadway";
    let leadThreatDist = Infinity;
    let threateningObLat = 0.0;

    for (let ob of this.obstacles) {
      const dx = ob.x - this.ego.x;
      const dy = ob.y - this.ego.y;
      const dist = Math.hypot(dx, dy);
      const vRel = this.ego.v - ob.vx;
      const angleRad = Math.atan2(dy, dx) - this.ego.yaw;
      const angleDeg = (angleRad * 180) / Math.PI;

      ob.distance = dist;
      ob.relativeSpeed = vRel;
      ob.angleDeg = angleDeg;

      // Multi-sensor FOV logic
      const sensors = [];
      if (dist <= 80.0 && Math.abs(angleDeg) <= 32.5) sensors.push("CAM");
      if (dist <= 90.0 && Math.abs(angleDeg) <= 60.0) sensors.push("LIDAR");
      if (dist <= 150.0 && Math.abs(angleDeg) <= 22.5) sensors.push("RADAR");
      if (sensors.length === 0) sensors.push("EST");

      ob.sensors = sensors.join("+");
      ob.confidence = Math.min(0.99, 0.65 + 0.11 * sensors.length);

      // 3. 7-Factor Indian Road Risk Metric (IRRM)
      const riskEval = this.computeIRRM(dist, vRel, angleDeg, ob.class);
      ob.riskScore = riskEval.score;
      ob.ttc = riskEval.ttc;

      if (riskEval.score > maxRisk) {
        maxRisk = riskEval.score;
        minTTC = riskEval.ttc;
        primaryThreat = `${ob.name || ob.class} (${dist.toFixed(1)}m)`;
        leadThreatDist = dist;
        threateningObLat = ob.y;
        this.riskFactors = riskEval.factors;
      }
    }

    this.currentRiskScore = maxRisk;
    this.currentTTC = minTTC;
    this.primaryThreat = primaryThreat;

    // Criticality Mapping
    if (maxRisk < 0.35) {
      this.currentCriticality = "LOW";
      this.currentAction = "MAINTAIN_CRUISE";
    } else if (maxRisk < 0.65) {
      this.currentCriticality = "MEDIUM";
      this.currentAction = "PREPARE_NUDGE";
    } else if (maxRisk < 0.85) {
      this.currentCriticality = "HIGH";
      this.currentAction = "ACTIVE_DECELERATION";
    } else {
      this.currentCriticality = "CRITICAL";
      this.currentAction = "EMERGENCY_BRAKE";
    }

    // 4. Stateflow Tactical Supervisor Logic
    let targetSpeed = this.ego.targetSpeed;
    let targetNudge = 0.0;

    if (this.currentAction === "EMERGENCY_BRAKE" || (minTTC < 1.4 && minTTC > 0)) {
      this.stateflowState = "Emergency Stop";
      this.ego.accel = -6.5; // Max emergency braking
      targetSpeed = 0.0;
    } else if (this.currentAction === "ACTIVE_DECELERATION") {
      this.stateflowState = "Brake";
      this.ego.accel = -3.0;
      targetSpeed = Math.max(3.0, this.ego.targetSpeed * 0.45);
    } else if (this.currentAction === "PREPARE_NUDGE") {
      if (threateningObLat >= this.ego.y) {
        this.stateflowState = "Move Left";
        targetNudge = -1.25; // Nudge towards left shoulder
      } else {
        this.stateflowState = "Move Right";
        targetNudge = 1.25; // Nudge towards center
      }
      this.ego.accel = -1.0;
      targetSpeed = this.ego.targetSpeed * 0.75;
    } else {
      this.stateflowState = "Accelerate";
      this.ego.accel = Math.max(-2.0, Math.min(2.5, 1.8 * (targetSpeed - this.ego.v)));
      targetNudge = 0.0;
    }

    this.tacticalNudge = 0.85 * this.tacticalNudge + 0.15 * targetNudge;

    // 5. Kinematic 2-DOF Bicycle Model Integration
    this.ego.v = Math.max(0.0, Math.min(25.0, this.ego.v + this.ego.accel * dt));

    // Stanley lateral controller
    const lookaheadX = this.ego.x + Math.max(4.0, this.ego.v * 0.8);
    const targetY = -1.75 + this.tacticalNudge;
    let headingErr = Math.atan2(targetY - this.ego.y, lookaheadX - this.ego.x) - this.ego.yaw;
    headingErr = Math.atan2(Math.sin(headingErr), Math.cos(headingErr));
    this.ego.steer = Math.max(-this.vehicle.maxSteer, Math.min(this.vehicle.maxSteer, 0.75 * headingErr));

    const yawRate = (this.ego.v / this.vehicle.wheelbase) * Math.tan(this.ego.steer);
    this.ego.yaw += yawRate * dt;
    this.ego.x += this.ego.v * Math.cos(this.ego.yaw) * dt;
    this.ego.y += this.ego.v * Math.sin(this.ego.yaw) * dt;

    // Record history
    this.egoHistory.push({ x: this.ego.x, y: this.ego.y });
    if (this.egoHistory.length > 250) this.egoHistory.shift();

    // 6. Path Planning (Corridor + DWA Rollout)
    this.generatePaths();

    // Reset when reaching track end
    if (this.ego.x >= this.roadLength - 10.0) {
      return false; // Reached end of run
    }
    return true;
  }

  computeIRRM(d, vRel, angleDeg, objType) {
    const tau0 = 3.5;
    const d0 = 3.0;
    const aComf = 2.5;

    // 1. Kinetic Risk
    let ttc = Infinity;
    let rTTC = 0.0;
    if (vRel > 0.05) {
      ttc = d / vRel;
      rTTC = Math.exp(-ttc / tau0);
    }
    const dSafe = d0 + this.ego.v * 1.0 + (this.ego.v * this.ego.v) / (2 * aComf);
    const rProx = Math.exp(-d / Math.max(4.0, dSafe));
    const rKin = 0.65 * rTTC + 0.35 * rProx;

    // 2. Approach Angle Heading
    const rad = (angleDeg * Math.PI) / 180;
    const phiDir = 1.0 + 0.6 * Math.max(0.0, -Math.cos(rad)) + 0.45 * Math.abs(Math.sin(rad));

    // 3. Vulnerability
    let mType = 1.0;
    switch (objType) {
      case "Pedestrian":    mType = 2.50; break;
      case "Cattle":        mType = 2.20; break;
      case "Two-Wheeler":   mType = 2.00; break;
      case "Auto-Rickshaw": mType = 1.80; break;
      case "Truck-Bus":     mType = 2.40; break;
      case "Pothole":       mType = 1.60; break;
      case "Car":           mType = 1.00; break;
    }

    // 4. Road Width Lateral Bottleneck
    const cWidth = Math.pow(7.0 / Math.max(3.0, this.roadWidth), 1.4);

    // 5. Weather & Friction
    const omegaWeather = Math.pow(0.85 / Math.max(0.25, this.surfaceMu), 0.85) * (1.0 + 0.35 * (1.0 - this.visibility));

    // 6. Traffic Density
    const psiDensity = 1.0 + 0.12 * Math.min(5.0, this.obstacles.length);

    // Composite IRRM
    const rawRisk = rKin * (0.35 * phiDir + 0.25 * mType + 0.20 * cWidth + 0.15 * omegaWeather + 0.05 * psiDensity);
    let score = Math.max(0.0, Math.min(1.0, rawRisk));

    // Emergency physical stopping distance override
    const dStopEmerg = (this.ego.v * this.ego.v) / (2.0 * Math.max(0.2, this.surfaceMu) * 9.81);
    if (d <= dStopEmerg && vRel > 0) {
      score = Math.max(score, 0.92);
    }

    return {
      score: score,
      ttc: ttc,
      factors: [
        rKin,
        phiDir / 2.0,
        mType / 2.5,
        cWidth / 2.0,
        omegaWeather / 2.5,
        psiDensity / 1.6
      ]
    };
  }

  generatePaths() {
    // 1. Global Guide Path Corridor
    const pts = [];
    const count = 30;
    for (let i = 0; i < count; i++) {
      const px = this.ego.x + (i * 60.0) / count;
      const progress = Math.min(1.0, (px - this.ego.x) / 35.0);
      const py = -1.75 + this.tacticalNudge * Math.sin((progress * Math.PI) / 2);
      pts.push({ x: px, y: py });
    }
    this.guidePath = pts;

    // 2. DWA Short-Horizon Dynamic Trajectory (25 steps)
    const localPts = [];
    let sx = this.ego.x;
    let sy = this.ego.y;
    let syaw = this.ego.yaw;
    let sv = this.ego.v;
    const dwaDt = 0.1;

    for (let s = 0; s < 25; s++) {
      sv = 0.95 * sv + 0.05 * this.ego.targetSpeed;
      syaw += (sv / this.vehicle.wheelbase) * Math.tan(this.ego.steer) * dwaDt;
      sx += sv * Math.cos(syaw) * dwaDt;
      sy += sv * Math.sin(syaw) * dwaDt;
      localPts.push({ x: sx, y: sy });
    }
    this.localTrajectory = localPts;
  }
}
