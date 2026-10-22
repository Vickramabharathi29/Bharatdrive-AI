/**
 * BHARATDRIVE-AI: DASHBOARD CONTROLLER & UI RENDERER
 * Real-time Canvas Bird's-Eye View (BEV), SVG Speedometer Gauge,
 * Sensor Status, Collision Alert HUD, and 7-Factor IRRM Telemetry.
 */

document.addEventListener("DOMContentLoaded", () => {
  const sim = new SimulationEngine();

  // DOM Elements
  const canvas = document.getElementById("bevCanvas");
  const ctx = canvas.getContext("2d");

  // Speedometer Elements
  const speedValEl = document.getElementById("speedValue");
  const speedMsEl = document.getElementById("speedMsValue");
  const targetSpeedEl = document.getElementById("targetSpeedValue");
  const accelStatusEl = document.getElementById("accelStatusValue");
  const gaugeFillEl = document.getElementById("speedGaugeFill");

  // Collision Warning Elements
  const warningBannerEl = document.getElementById("warningBanner");
  const warningHeadlineEl = document.getElementById("warningHeadline");
  const warningSubtextEl = document.getElementById("warningSubtext");
  const warningIconEl = document.getElementById("warningIcon");
  const ttcValueEl = document.getElementById("ttcValue");
  const threatDetailEl = document.getElementById("threatDetail");

  // Sensor Elements
  const sensorCamStatus = document.getElementById("sensorCamStatus");
  const sensorLidarStatus = document.getElementById("sensorLidarStatus");
  const sensorRadarStatus = document.getElementById("sensorRadarStatus");
  const sensorImuStatus = document.getElementById("sensorImuStatus");
  const weatherFrictionEl = document.getElementById("weatherFriction");

  // Risk Elements
  const riskScoreNumEl = document.getElementById("riskScoreNum");
  const riskCriticalityBadgeEl = document.getElementById("riskCriticalityBadge");
  const riskBarFillEl = document.getElementById("riskBarFill");
  const riskActionDirectiveEl = document.getElementById("riskActionDirective");

  // 7-Factor Bars
  const factorBars = [
    document.getElementById("factorBarKinetic"),
    document.getElementById("factorBarHeading"),
    document.getElementById("factorBarVulnerability"),
    document.getElementById("factorBarBottleneck"),
    document.getElementById("factorBarWeather"),
    document.getElementById("factorBarDensity")
  ];
  const factorVals = [
    document.getElementById("factorValKinetic"),
    document.getElementById("factorValHeading"),
    document.getElementById("factorValVulnerability"),
    document.getElementById("factorValBottleneck"),
    document.getElementById("factorValWeather"),
    document.getElementById("factorValDensity")
  ];

  // Objects Table
  const objectsTableBody = document.getElementById("objectsTableBody");

  // Header & Controls Elements
  const scenarioSelect = document.getElementById("scenarioSelect");
  const fpsValueEl = document.getElementById("fpsValue");
  const simTimeValueEl = document.getElementById("simTimeValue");
  const stepCountValueEl = document.getElementById("stepCountValue");
  const maneuverBadgeEl = document.getElementById("maneuverBadge");

  const btnPlay = document.getElementById("btnPlay");
  const btnPause = document.getElementById("btnPause");
  const btnStep = document.getElementById("btnStep");
  const btnReset = document.getElementById("btnReset");
  const btnEmergency = document.getElementById("btnEmergency");
  const simSpeedSelect = document.getElementById("simSpeedSelect");

  // Animation & FPS Tracker
  let isRunning = true;
  let lastFrameTime = performance.now();
  let frameCount = 0;
  let currentFps = 20.0;
  let accumulator = 0;

  // Initialize Canvas DPI
  function resizeCanvas() {
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;
    ctx.scale(dpr, dpr);
  }
  window.addEventListener("resize", resizeCanvas);
  resizeCanvas();

  // SVG Gauge Circumference Calculation (radius = 70, stroke arc ~ 275)
  const GAUGE_CIRCUMFERENCE = 275;
  if (gaugeFillEl) {
    gaugeFillEl.style.strokeDasharray = `${GAUGE_CIRCUMFERENCE}`;
    gaugeFillEl.style.strokeDashoffset = `${GAUGE_CIRCUMFERENCE}`;
  }

  // --- UI Update Function ---
  function updateUI() {
    // 1. Vehicle Speed Readout & Gauge
    const speedKmh = sim.ego.v * 3.6;
    speedValEl.textContent = speedKmh.toFixed(1);
    speedMsEl.textContent = `${sim.ego.v.toFixed(2)} m/s`;
    targetSpeedEl.textContent = `${(sim.ego.targetSpeed * 3.6).toFixed(1)} km/h`;

    if (sim.ego.accel > 0.3) {
      accelStatusEl.textContent = `+${sim.ego.accel.toFixed(2)} m/s² (Accel)`;
      accelStatusEl.style.color = "var(--accent-green)";
    } else if (sim.ego.accel < -0.3) {
      accelStatusEl.textContent = `${sim.ego.accel.toFixed(2)} m/s² (Braking)`;
      accelStatusEl.style.color = "var(--accent-red)";
    } else {
      accelStatusEl.textContent = "Cruising Steady";
      accelStatusEl.style.color = "var(--accent-cyan)";
    }

    // Update gauge arc (0 - 100 km/h)
    const gaugeFrac = Math.min(1.0, Math.max(0.0, speedKmh / 90.0));
    const offset = GAUGE_CIRCUMFERENCE * (1.0 - gaugeFrac);
    if (gaugeFillEl) gaugeFillEl.style.strokeDashoffset = `${offset}`;

    // 2. Collision Warning & TTC Banner
    warningBannerEl.className = "warning-banner";
    if (sim.currentRiskScore >= 0.85 || (sim.currentTTC < 1.5 && sim.currentTTC > 0)) {
      warningBannerEl.classList.add("status-critical");
      warningHeadlineEl.textContent = "COLLISION IMMINENT - AEB ENGAGED";
      warningSubtextEl.textContent = "Maximum emergency braking activated";
    } else if (sim.currentRiskScore >= 0.65 || (sim.currentTTC < 3.0 && sim.currentTTC > 0)) {
      warningBannerEl.classList.add("status-warning");
      warningHeadlineEl.textContent = "FCW: FORWARD COLLISION WARNING";
      warningSubtextEl.textContent = "Obstacle closing fast within safety corridor";
    } else if (sim.currentRiskScore >= 0.35) {
      warningBannerEl.classList.add("status-caution");
      warningHeadlineEl.textContent = "CAUTION: THREAT AHEAD";
      warningSubtextEl.textContent = "Adaptive deceleration & nudge in progress";
    } else {
      warningBannerEl.classList.add("status-safe");
      warningHeadlineEl.textContent = "SAFE: ROAD CLEAR AHEAD";
      warningSubtextEl.textContent = "Cruising inside safe corridor envelope";
    }

    if (isFinite(sim.currentTTC)) {
      ttcValueEl.textContent = `${sim.currentTTC.toFixed(1)}s`;
      ttcValueEl.style.color = sim.currentTTC < 2.0 ? "var(--accent-red)" : "var(--text-main)";
    } else {
      ttcValueEl.textContent = "Safe (∞)";
      ttcValueEl.style.color = "var(--accent-green)";
    }
    threatDetailEl.innerHTML = `Threat: <strong>${sim.primaryThreat}</strong>`;

    // 3. Sensor Suite Status
    weatherFrictionEl.textContent = `Friction: μ=${sim.surfaceMu.toFixed(2)} | Vis: ${(sim.visibility * 100).toFixed(0)}%`;

    // 4. Current Risk & 7-Factor IRRM Breakdown
    const riskPct = Math.round(sim.currentRiskScore * 100);
    riskScoreNumEl.textContent = riskPct;
    riskBarFillEl.style.width = `${riskPct}%`;

    riskCriticalityBadgeEl.className = "risk-criticality-badge";
    riskCriticalityBadgeEl.classList.add(sim.currentCriticality.toLowerCase());
    riskCriticalityBadgeEl.textContent = `Criticality: ${sim.currentCriticality}`;

    riskActionDirectiveEl.innerHTML = `Recommended Action: <strong>${sim.currentAction.replace(/_/g, " ")}</strong>`;

    // Update 6 factor bars
    for (let f = 0; f < 6; f++) {
      const val = sim.riskFactors[f] || 0.05;
      const pct = Math.min(100, Math.max(5, Math.round(val * 100)));
      factorBars[f].style.width = `${pct}%`;
      factorVals[f].textContent = (val * 10).toFixed(1);
    }

    // 5. Maneuver Badge & Headers
    maneuverBadgeEl.textContent = `State: ${sim.stateflowState}`;
    simTimeValueEl.textContent = `${sim.simTime.toFixed(2)}s`;
    stepCountValueEl.textContent = sim.stepCount;

    // 6. Detected Objects Table
    renderObjectsTable();

    // 7. Render BEV Canvas
    renderBEV();
  }

  function renderObjectsTable() {
    let rowsHtml = "";
    for (let ob of sim.obstacles) {
      let badgeClass = "class-car";
      switch (ob.class) {
        case "Cattle": badgeClass = "class-cattle"; break;
        case "Auto-Rickshaw": badgeClass = "class-auto"; break;
        case "Two-Wheeler": badgeClass = "class-twowheeler"; break;
        case "Truck-Bus": badgeClass = "class-truckbus"; break;
        case "Pedestrian": badgeClass = "class-pedestrian"; break;
        case "Pothole": badgeClass = "class-pothole"; break;
      }

      const sign = ob.relativeSpeed >= 0 ? "+" : "";
      const riskP = Math.round(ob.riskScore * 100);
      let riskColor = "var(--accent-green)";
      if (riskP >= 85) riskColor = "var(--accent-red)";
      else if (riskP >= 65) riskColor = "var(--accent-orange)";
      else if (riskP >= 35) riskColor = "var(--accent-amber)";

      rowsHtml += `
        <tr>
          <td>#${ob.id}</td>
          <td><span class="class-badge ${badgeClass}">${ob.class}</span></td>
          <td>${ob.distance.toFixed(1)}m</td>
          <td>${sign}${ob.relativeSpeed.toFixed(1)}m/s</td>
          <td>${Math.round(ob.confidence * 100)}%</td>
          <td><span class="sensor-tags">${ob.sensors}</span></td>
          <td style="color:${riskColor}; font-weight:700;">${riskP}%</td>
        </tr>
      `;
    }
    objectsTableBody.innerHTML = rowsHtml;
  }

  // --- BEV Bird's-Eye-View Canvas Rendering ---
  function renderBEV() {
    const rect = canvas.getBoundingClientRect();
    const w = rect.width;
    const h = rect.height;

    ctx.clearRect(0, 0, w, h);

    // World to Screen Transformation:
    // Follow ego vehicle: Ego is placed at 20% from left
    const egoX = sim.ego.x;
    const egoY = sim.ego.y;
    const egoYaw = sim.ego.yaw;

    const scaleX = w / 75.0; // Show ~75 meters longitudinally
    const scaleY = h / 14.0; // Show ~14 meters laterally (road width ~7m + borders)
    const originScreenX = w * 0.18;
    const originScreenY = h * 0.50;

    function worldToScreen(wx, wy) {
      const sx = originScreenX + (wx - egoX) * scaleX;
      const sy = originScreenY - wy * scaleY;
      return { x: sx, y: sy };
    }

    const rw = sim.roadWidth;

    // 1. Draw Road Surface Asphalt
    const roadTopY = rw / 2;
    const roadBotY = -rw / 2;
    const p1 = worldToScreen(egoX - 25, roadTopY);
    const p2 = worldToScreen(egoX + 90, roadBotY);

    ctx.fillStyle = "#181d26";
    ctx.fillRect(0, p1.y, w, p2.y - p1.y);

    // Subtle asphalt noise / road shoulder
    ctx.fillStyle = "#12151c";
    ctx.fillRect(0, 0, w, p1.y);
    ctx.fillRect(0, p2.y, w, h - p2.y);

    // 2. Road Boundary Solid Lines
    ctx.strokeStyle = "#e0e6ed";
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(0, p1.y);
    ctx.lineTo(w, p1.y);
    ctx.moveTo(0, p2.y);
    ctx.lineTo(w, p2.y);
    ctx.stroke();

    // 3. Dashed Center Lane Marking (Indian Road Yellow or White)
    const pCenter = worldToScreen(egoX, 0.0);
    ctx.strokeStyle = "#ffca28";
    ctx.lineWidth = 1.8;
    ctx.setLineDash([16, 12]);
    ctx.beginPath();
    ctx.moveTo(0, pCenter.y);
    ctx.lineTo(w, pCenter.y);
    ctx.stroke();
    ctx.setLineDash([]); // Reset dash

    // 4. Traveled Historical Path
    if (sim.egoHistory.length > 1) {
      ctx.strokeStyle = "rgba(0, 229, 255, 0.35)";
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      for (let i = 0; i < sim.egoHistory.length; i++) {
        const pt = worldToScreen(sim.egoHistory[i].x, sim.egoHistory[i].y);
        if (i === 0) ctx.moveTo(pt.x, pt.y);
        else ctx.lineTo(pt.x, pt.y);
      }
      ctx.stroke();
    }

    // 5. Global Guide Path (Hybrid A* Corridor with Tactical Nudge)
    if (sim.guidePath.length > 1) {
      ctx.strokeStyle = "rgba(0, 229, 255, 0.55)";
      ctx.lineWidth = 2.0;
      ctx.setLineDash([6, 6]);
      ctx.beginPath();
      for (let i = 0; i < sim.guidePath.length; i++) {
        const pt = worldToScreen(sim.guidePath[i].x, sim.guidePath[i].y);
        if (i === 0) ctx.moveTo(pt.x, pt.y);
        else ctx.lineTo(pt.x, pt.y);
      }
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // 6. Dynamic Window Approach (DWA) Execution Trajectory
    if (sim.localTrajectory.length > 1) {
      ctx.strokeStyle = "#00e676";
      ctx.lineWidth = 3.2;
      ctx.shadowColor = "rgba(0, 230, 118, 0.6)";
      ctx.shadowBlur = 8;
      ctx.beginPath();
      for (let i = 0; i < sim.localTrajectory.length; i++) {
        const pt = worldToScreen(sim.localTrajectory[i].x, sim.localTrajectory[i].y);
        if (i === 0) ctx.moveTo(pt.x, pt.y);
        else ctx.lineTo(pt.x, pt.y);
      }
      ctx.stroke();
      ctx.shadowBlur = 0; // Reset glow
    }

    // 7. Tactical Nudge Lateral Vector
    if (Math.abs(sim.tacticalNudge) > 0.1) {
      const nudgeStart = worldToScreen(egoX + 4.0, -1.75);
      const nudgeEnd = worldToScreen(egoX + 4.0, -1.75 + sim.tacticalNudge);
      ctx.strokeStyle = "#b388ff";
      ctx.lineWidth = 2.2;
      ctx.beginPath();
      ctx.moveTo(nudgeStart.x, nudgeStart.y);
      ctx.lineTo(nudgeEnd.x, nudgeEnd.y);
      ctx.stroke();

      ctx.fillStyle = "#b388ff";
      ctx.font = "bold 10px JetBrains Mono";
      ctx.fillText(`NUDGE ${sim.tacticalNudge > 0 ? "+" : ""}${sim.tacticalNudge.toFixed(1)}m`, nudgeEnd.x + 8, nudgeEnd.y + 4);
    }

    // 8. Draw Detected Obstacles with Oriented Bounding Boxes (OBBs)
    for (let ob of sim.obstacles) {
      const pOb = worldToScreen(ob.x, ob.y);

      // Color mapping
      let fillColor = "rgba(255, 109, 0, 0.75)";
      let strokeColor = "#ff9100";
      switch (ob.class) {
        case "Cattle":
          fillColor = "rgba(255, 109, 0, 0.75)";
          strokeColor = "#ff9100";
          break;
        case "Auto-Rickshaw":
          fillColor = "rgba(255, 179, 0, 0.75)";
          strokeColor = "#ffd54f";
          break;
        case "Two-Wheeler":
          fillColor = "rgba(0, 229, 255, 0.75)";
          strokeColor = "#40c4ff";
          break;
        case "Truck-Bus":
          fillColor = "rgba(255, 51, 75, 0.75)";
          strokeColor = "#ff5252";
          break;
        case "Pedestrian":
          fillColor = "rgba(179, 136, 255, 0.75)";
          strokeColor = "#b388ff";
          break;
        case "Pothole":
          fillColor = "rgba(120, 120, 120, 0.65)";
          strokeColor = "#e0e0e0";
          break;
      }

      const boxW = ob.length * scaleX;
      const boxH = ob.width * scaleY;

      ctx.save();
      ctx.translate(pOb.x, pOb.y);
      // Rotation if obstacle has heading
      const obYaw = Math.atan2(ob.vy, ob.vx);
      if (Math.hypot(ob.vx, ob.vy) > 0.3) {
        ctx.rotate(-obYaw);
      }

      ctx.fillStyle = fillColor;
      ctx.strokeStyle = strokeColor;
      ctx.lineWidth = 1.6;
      ctx.beginPath();
      ctx.roundRect(-boxW / 2, -boxH / 2, boxW, boxH, 3);
      ctx.fill();
      ctx.stroke();

      // Velocity arrow
      if (Math.hypot(ob.vx, ob.vy) > 0.5) {
        ctx.strokeStyle = "#fff";
        ctx.lineWidth = 1.4;
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(boxW * 0.8, 0);
        ctx.lineTo(boxW * 0.8 - 4, -3);
        ctx.moveTo(boxW * 0.8, 0);
        ctx.lineTo(boxW * 0.8 - 4, 3);
        ctx.stroke();
      }

      ctx.restore();

      // Obstacle Label
      ctx.fillStyle = "#ffffff";
      ctx.font = "bold 9px Inter, sans-serif";
      ctx.textAlign = "center";
      ctx.fillText(`${ob.class} (${ob.distance.toFixed(0)}m)`, pOb.x, pOb.y - boxH / 2 - 6);
    }

    // 9. Draw Ego Vehicle (Tata Nexon EV)
    const pEgo = worldToScreen(egoX, egoY);
    const egoScreenL = sim.vehicle.length * scaleX;
    const egoScreenW = sim.vehicle.width * scaleY;

    ctx.save();
    ctx.translate(pEgo.x, pEgo.y);
    ctx.rotate(-egoYaw);

    // Sensor FOV Cones: Camera (65 deg) & LiDAR (120 deg)
    ctx.fillStyle = "rgba(0, 229, 255, 0.08)";
    ctx.beginPath();
    ctx.moveTo(egoScreenL / 2, 0);
    ctx.arc(egoScreenL / 2, 0, 75 * (scaleX / 3.0), -0.56, 0.56); // 65 deg FOV
    ctx.closePath();
    ctx.fill();

    // Ego Body Polygon
    ctx.fillStyle = "#00e5ff";
    ctx.shadowColor = "rgba(0, 229, 255, 0.55)";
    ctx.shadowBlur = 12;
    ctx.beginPath();
    ctx.roundRect(-egoScreenL / 2, -egoScreenW / 2, egoScreenL, egoScreenW, 4);
    ctx.fill();
    ctx.shadowBlur = 0;

    // Roof & Windshield
    ctx.fillStyle = "#0a1324";
    ctx.fillRect(-egoScreenL * 0.25, -egoScreenW * 0.35, egoScreenL * 0.5, egoScreenW * 0.7);

    // Headlight Beams
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 2.0;
    ctx.beginPath();
    ctx.moveTo(egoScreenL / 2, -egoScreenW * 0.35);
    ctx.lineTo(egoScreenL / 2 + 8, -egoScreenW * 0.35);
    ctx.moveTo(egoScreenL / 2, egoScreenW * 0.35);
    ctx.lineTo(egoScreenL / 2 + 8, egoScreenW * 0.35);
    ctx.stroke();

    ctx.restore();
  }

  // --- Master Animation Frame Loop ---
  function loop(currentTime) {
    const elapsed = currentTime - lastFrameTime;
    lastFrameTime = currentTime;

    // Calculate real-time FPS
    frameCount++;
    if (frameCount % 10 === 0 && elapsed > 0) {
      currentFps = 0.85 * currentFps + 0.15 * (1000 / Math.max(elapsed, 1));
      fpsValueEl.textContent = currentFps.toFixed(1);
    }

    if (isRunning) {
      accumulator += elapsed;
      const stepInterval = (sim.dt * 1000) / sim.speedMultiplier;

      while (accumulator >= stepInterval) {
        const canContinue = sim.step();
        accumulator -= stepInterval;
        if (!canContinue) {
          isRunning = false;
          btnPlay.textContent = "▶ Play";
          break;
        }
      }
    }

    updateUI();
    requestAnimationFrame(loop);
  }

  // --- User Event Listeners ---
  btnPlay.addEventListener("click", () => {
    isRunning = !isRunning;
    btnPlay.textContent = isRunning ? "⏸ Pause" : "▶ Play";
    btnPlay.className = isRunning ? "btn btn-secondary" : "btn btn-primary";
  });

  btnPause.addEventListener("click", () => {
    isRunning = false;
    btnPlay.textContent = "▶ Play";
    btnPlay.className = "btn btn-primary";
  });

  btnStep.addEventListener("click", () => {
    isRunning = false;
    btnPlay.textContent = "▶ Play";
    sim.step();
    updateUI();
  });

  btnReset.addEventListener("click", () => {
    sim.loadScenario(sim.activeScenarioId);
    updateUI();
  });

  btnEmergency.addEventListener("click", () => {
    sim.stateflowState = "Emergency Stop";
    sim.currentAction = "EMERGENCY_BRAKE";
    sim.currentRiskScore = 0.98;
    sim.currentCriticality = "CRITICAL";
    sim.ego.accel = -7.0;
    updateUI();
  });

  scenarioSelect.addEventListener("change", (e) => {
    const scId = parseInt(e.target.value, 10);
    sim.loadScenario(scId);
    updateUI();
  });

  simSpeedSelect.addEventListener("change", (e) => {
    sim.speedMultiplier = parseFloat(e.target.value);
  });

  // --- Evaluation Metrics Modal & Graph Tabs ---
  const metricsModal = document.getElementById("metricsModal");
  const openMetricsBtn = document.getElementById("openMetricsBtn");
  const closeMetricsBtn = document.getElementById("closeMetricsBtn");
  const closeMetricsBackdrop = document.getElementById("closeMetricsBackdrop");
  const activeGraphImg = document.getElementById("activeGraphImg");
  const graphTabBtns = document.querySelectorAll(".graph-tab-btn");

  if (openMetricsBtn && metricsModal) {
    openMetricsBtn.addEventListener("click", () => {
      metricsModal.style.display = "flex";
      document.body.style.overflow = "hidden";
    });

    const closeModal = () => {
      metricsModal.style.display = "none";
      document.body.style.overflow = "";
    };

    if (closeMetricsBtn) closeMetricsBtn.addEventListener("click", closeModal);
    if (closeMetricsBackdrop) closeMetricsBackdrop.addEventListener("click", closeModal);

    // Keyboard ESC to close modal
    window.addEventListener("keydown", (e) => {
      if (e.key === "Escape" && metricsModal.style.display === "flex") {
        closeModal();
      }
    });

    // Tab switching
    graphTabBtns.forEach(btn => {
      btn.addEventListener("click", () => {
        graphTabBtns.forEach(b => b.classList.remove("active"));
        btn.classList.add("active");
        const graphSrc = btn.getAttribute("data-graph");
        if (graphSrc && activeGraphImg) {
          activeGraphImg.style.opacity = "0.3";
          activeGraphImg.src = graphSrc;
          setTimeout(() => {
            activeGraphImg.style.opacity = "1";
          }, 80);
        }
      });
    });
  }

  // Start the simulation animation loop
  requestAnimationFrame(loop);
});
