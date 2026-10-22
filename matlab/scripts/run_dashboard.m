% RUN_DASHBOARD Launches the BharatDrive-AI MATLAB App Designer Dashboard.
%
% Displays:
%   1. Vehicle Speed (Analog & Digital Gauge, Target Speed, Acceleration)
%   2. Current Risk (7-Factor Indian Road Risk Metric IRRM, Criticality, Action, Factor Bar Chart)
%   3. Detected Objects (Detailed Tracked Objects Table & Sensor Contributors)
%   4. Collision Warning (FCW / AEB Alert Lamp, TTC Countdown, Threat Description)
%   5. Current Path (2D Bird's-Eye View Trajectory, Hybrid A* Corridor, DWA Path, Tactical Nudge)
%   6. FPS (Real-time Simulation/Rendering Framerate & Step Execution Latency)
%   7. Sensor Status (Camera, 3D LiDAR, Radar, IMU/GNSS, Health & Weather Degradation)
%   8. Scenario Name (Scenario Selector: Village Road, Busy Market, Urban Junction, Highway, Cattle Crossing)
%
% Usage:
%   run_dashboard          % Launches interactive App Designer GUI
%   app = run_dashboard;   % Returns app handle for programmatic inspection
%
% BharatDrive-AI - MathWorks Autonomous Driving Hackathon.

function app_out = run_dashboard()
    clc;
    fprintf('=========================================================================================\n');
    fprintf('   BHARATDRIVE-AI: MATLAB APP DESIGNER TELEMETRY & RISK DASHBOARD                        \n');
    fprintf('   Adaptive Path Planning, Collision Avoidance & Multi-Agent Negotiation                 \n');
    fprintf('=========================================================================================\n\n');

    % 1. Setup paths
    script_dir = fileparts(mfilename('fullpath'));
    project_root = fullfile(script_dir, '..');
    addpath(project_root);
    addpath(fullfile(project_root, '+bharatdrive'));
    addpath(fullfile(project_root, 'scripts'));
    addpath(fullfile(project_root, 'simulink'));

    fprintf('[1/3] Workspace paths loaded successfully.\n');
    fprintf('[2/3] Initializing BharatDriveDashboard App Designer Engine...\n');

    try
        app = BharatDriveDashboard();
        fprintf('[3/3] Dashboard UIFigure opened successfully!\n\n');
        fprintf('Active Features:\n');
        fprintf('  • Vehicle Speed:        Real-Time Gauge + Digital Readout (km/h & m/s)\n');
        fprintf('  • Current Risk:         7-Factor IRRM Risk Gauge + Criticality + Action\n');
        fprintf('  • Detected Objects:     Multi-Sensor Fused Object Tracking Table\n');
        fprintf('  • Collision Warning:    FCW / AEB Alert Lamp with Dynamic TTC Countdown\n');
        fprintf('  • Current Path:         2D Bird''s-Eye View (BEV) with Hybrid A* & DWA Trajectories\n');
        fprintf('  • FPS Counter:          Live 20 Hz Simulation & Rendering Framerate\n');
        fprintf('  • Sensor Status:        Camera, 3D LiDAR, Radar & IMU/GNSS Health Diagnostics\n');
        fprintf('  • Scenario Selector:    5 Indian Road Scenarios + 1 Mixed Hazard Master Scenario\n\n');
        fprintf('Click "▶ Play" in the dashboard toolbar to begin closed-loop simulation.\n');
        fprintf('=========================================================================================\n');

        if nargout > 0
            app_out = app;
        end
    catch ME
        fprintf('Error launching MATLAB App Designer interface: %s\n', ME.message);
        rethrow(ME);
    end
end
