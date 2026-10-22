# BharatDrive-AI: Complete System Setup & Installation Guide

This guide walks you through setting up the complete **BharatDrive-AI** development, simulation, and evaluation environment on **Windows 10/11** or **Ubuntu 20.04/22.04 LTS**.

---

## 1. System Requirements & Prerequisites

### Minimum Hardware:
- **Processor**: Intel Core i5 / AMD Ryzen 5 (4+ Cores, 2.5 GHz+)
- **System Memory**: 8 GB RAM minimum (16 GB recommended for RoadRunner 3D)
- **Disk Storage**: 5 GB free disk space
- **Display**: 1920x1080 resolution recommended for dual-screen dashboard telemetry

### Recommended Hardware (with GPU Acceleration):
- **Processor**: Intel Core i7/i9 or AMD Ryzen 7/9 (8+ Cores)
- **Memory**: 32 GB DDR4/DDR5 RAM
- **GPU**: NVIDIA RTX 3060 / 4060 or higher with CUDA 12.0+ (for real-time YOLO perception & 3D point cloud rendering)

### Supported Software Versions:
- **Python**: 3.10, 3.11, or 3.12 (Tested on 3.12.10)
- **MATLAB**: R2024b or R2025b
- **Simulink**: R2024b or R2025b
- **Toolboxes (Optional for Native Co-Sim)**:
  - Automated Driving Toolbox™
  - Navigation Toolbox™
  - Sensor Fusion and Tracking Toolbox™
  - Stateflow®
  - RoadRunner & RoadRunner Scenario

---

## 2. Python Environment Installation

### Step 2.1: Clone or Navigate to Repository
```bash
git clone https://github.com/your-org/Bharatdrive-AI.git
cd Bharatdrive-AI
```

### Step 2.2: Create and Activate Virtual Environment
```bash
# On Windows (PowerShell):
python -m venv venv
.\venv\Scripts\Activate.ps1

# On Linux/macOS:
python3 -m venv venv
source venv/bin/activate
```

### Step 2.3: Install Dependencies
```bash
python -m pip install --upgrade pip
pip install -r requirements.txt
pip install -e .
```

Verify installation:
```bash
python -c "import bharatdrive; print('BharatDrive-AI version:', getattr(bharatdrive, '__version__', '2.5.0'))"
```

---

## 3. MATLAB & Simulink Environment Setup

### Step 3.1: Launch MATLAB and Set Working Directory
1. Open MATLAB (R2024b or R2025b).
2. Set the current working folder to the root of the repository:
   ```matlab
   cd('c:/Users/HP/OneDrive/Desktop/Bharatdrive-AI')
   ```

### Step 3.2: Initialize Paths and Packages
Run the automated environment setup script:
```matlab
startup
```
This automatically configures:
- Path addition for `matlab/`, `matlab/+bharatdrive/`, `matlab/scripts/`, `config/`, and `simulink/`.
- Vehicle configuration dictionary initialization (`load_vehicle_params`).
- Indian Roads Congress standard catalog loading (`load_indian_road_specs`).

### Step 3.3: Verify MATLAB Installation
Run the automated self-diagnostic script:
```matlab
% Validate bicycle models
kin = bharatdrive.models.KinematicBicycle();
disp('Kinematic Bicycle Model: OK');

% Validate Frenet transforms
spline = bharatdrive.frenet.Spline2D([0 20 50 100], [0 1.5 0 -1.5]);
disp('Frenet Spline Engine: OK');
```

---

## 4. RoadRunner 3D Scenario Setup

1. Open **RoadRunner**.
2. Select **File -> Open Project** and navigate to `roadrunner_scenes/`.
3. Open any of the 6 scenario maps:
   - `roadrunner_scenes/scenarios/scenario_1_gramin_sadak.rrscene`
   - `roadrunner_scenes/scenarios/scenario_5_bovine_herd.rrscene`
   - `roadrunner_scenes/scenarios/scenario_6_mixed_hazard.rrscene`
4. The ASAM OpenDRIVE geometry (`.xodr`) and OpenSCENARIO dynamic actor definitions (`.xosc`) will load automatically with custom Indian 3D assets (Tata 407 trucks, auto-rickshaws, zebu cattle, and potholes).

---

## 5. Web Telemetry Dashboard Setup (Zero-Install)

The interactive telemetry dashboard requires **no complex Node.js or web frameworks**:
```bash
# Start lightweight Python HTTP server from project root
python -m http.server 8085
```
Open your web browser and navigate to:
```
http://localhost:8085/dashboard/index.html
```
- Real-time 20 Hz simulation engine with 60 FPS Canvas Bird's-Eye View.
- Live speed gauge, 7-factor IRRM risk bar chart, collision alert banner, and detected objects table.
- Direct **"Evaluation Metrics"** button in the header bar to inspect all 7 KPIs, interactive vector graph tabs, and comparison table.

---

## 6. Docker Containerization Setup (Optional / Production)

For deployment on Linux servers or containerized in-vehicle computers:
```bash
# Build production Docker container
docker build -t bharatdrive-ai:v2.5 .

# Run with GPU acceleration and web dashboard port
docker run -d -p 8085:8085 --gpus all --name bharatdrive bharatdrive-ai:v2.5
```
Navigate to `http://localhost:8085/dashboard/index.html`.
