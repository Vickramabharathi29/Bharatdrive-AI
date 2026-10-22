% RUN_RISK_ASSESSMENT_SIMULATION Evaluates the 7-factor Indian Road Risk Metric (IRRM)
% across diverse road widths, weather conditions, and heterogeneous obstacles.

clear; clc; close all;

fprintf('=========================================================================================\n');
fprintf('   BHARATDRIVE-AI: DYNAMIC RISK ASSESSMENT ENGINE (MATLAB)                               \n');
fprintf('   Indian Road Risk Metric (IRRM): 7-Factor Contextual Risk Evaluation                    \n');
fprintf('=========================================================================================\n\n');

% 1. Setup paths
root_dir = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(root_dir);
addpath(fullfile(root_dir, '+bharatdrive'));

% 2. Instantiate Risk Engine and Costmap Grid
risk_engine = bharatdrive.risk.RiskAssessmentEngine();
costmap = bharatdrive.risk.SpatioTemporalCostmap([-10, 80], [-10, 10], 0.25);

% 3. Define Contextual Driving Scenarios
ego_speed = 13.89; % 50 km/h

% Compare 3 Environmental Road Contexts
test_contexts(1) = struct('Name', 'Dry Two-Lane Highway', 'RoadWidth', 7.00, ...
    'Weather', struct('friction_mu', 0.85, 'visibility', 1.0));
test_contexts(2) = struct('Name', 'Monsoon Rain Two-Lane Road', 'RoadWidth', 7.00, ...
    'Weather', struct('friction_mu', 0.50, 'visibility', 0.60));
test_contexts(3) = struct('Name', 'Village Road / Narrow Street', 'RoadWidth', 3.75, ...
    'Weather', struct('friction_mu', 0.45, 'visibility', 0.85));

% Test Obstacles
obstacles(1) = struct('ID', 1, 'Type', 'Cattle', 'Distance', 28.0, 'VRel', 13.89, ...
    'AngleDeg', 0.0, 'Desc', 'Stationary Cow in Ego Path');
obstacles(2) = struct('ID', 2, 'Type', 'Cattle', 'Distance', 22.0, 'VRel', 2.50, ...
    'AngleDeg', 75.0, 'Desc', 'Calf Sprinting from Left Shoulder');
obstacles(3) = struct('ID', 3, 'Type', 'Truck-Bus', 'Distance', 65.0, 'VRel', 26.39, ...
    'AngleDeg', 180.0, 'Desc', 'Oncoming Commercial Bus');
obstacles(4) = struct('ID', 4, 'Type', 'Two-Wheeler', 'Distance', 14.0, 'VRel', 3.50, ...
    'AngleDeg', 45.0, 'Desc', 'Motorcycle Filtering / Weaving');
obstacles(5) = struct('ID', 5, 'Type', 'Pothole', 'Distance', 9.5, 'VRel', 13.89, ...
    'AngleDeg', 0.0, 'Desc', 'Severe Road Surface Pothole');

for c = 1:length(test_contexts)
    ctx = test_contexts(c);
    fprintf('-----------------------------------------------------------------------------------------\n');
    fprintf('SCENARIO CONTEXT: %s | Road Width = %0.2f m | Friction mu = %0.2f | Visibility = %0.0f%%\n', ...
        ctx.Name, ctx.RoadWidth, ctx.Weather.friction_mu, ctx.Weather.visibility * 100);
    fprintf('-----------------------------------------------------------------------------------------\n');
    fprintf('%-6s | %-14s | %-9s | %-8s | %-6s | %-10s | %-8s | %-18s\n', ...
        'Obs ID', 'Object Class', 'Distance', 'RelSpeed', 'TTC', 'Risk Score', 'Level', 'Recommended Action');
    fprintf('-----------------------------------------------------------------------------------------\n');

    for obs_idx = 1:length(obstacles)
        ob = obstacles(obs_idx);
        res = risk_engine.evaluate_risk( ...
            ob.Distance, ob.VRel, ob.AngleDeg, ob.Type, ...
            ctx.RoadWidth, ctx.Weather, 2, ego_speed);

        if res.Time_to_Collision == inf
            ttc_str = 'inf';
        else
            ttc_str = sprintf('%4.1fs', res.Time_to_Collision);
        end

        fprintf('#%-5d | %-14s | %6.1fm  | %5.1fm/s | %-6s | %8.1f%%  | %-8s | %-18s\n', ...
            ob.ID, ob.Type, ob.Distance, ob.VRel, ttc_str, ...
            res.RiskScore * 100, res.Criticality, res.ActionRecommendation);
    end
    fprintf('\n');
end

fprintf('=========================================================================================\n');
fprintf('[SUCCESS] Dynamic Risk Assessment completed across all dimensions and contexts.\n');
