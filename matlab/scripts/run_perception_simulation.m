% RUN_PERCEPTION_SIMULATION Runs end-to-end multi-sensor perception and fusion.
% Simulates Camera, LiDAR, and Radar on an Indian Road scenario with
% heterogeneous obstacles (Cattle, Commercial Bus, Two-Wheeler, Auto-Rickshaw).
%
% Output displays:
% - Detected Objects
% - Object Position [X, Y, Z]
% - Velocity [Vx, Vy]
% - Distance (m)
% - Confidence Score [0.0 - 1.0]

clear; clc; close all;

fprintf('=================================================================\n');
fprintf('   BHARATDRIVE-AI: SENSOR FUSION PERCEPTION ENGINE (MATLAB)      \n');
fprintf('   Multi-Modal Fusion: Camera + 3D LiDAR + Doppler Radar         \n');
fprintf('=================================================================\n\n');

% 1. Add workspace paths
root_dir = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(root_dir));
addpath(fullfile(root_dir, '+bharatdrive'));

% 2. Simulation parameters
dt = 0.05;          % 20 Hz perception update rate
total_time = 4.0;   % 4 seconds simulation
time_steps = round(total_time / dt);

% 3. Initialize Sensor Suite and Fusion Engine
sensor_suite = bharatdrive.perception.SensorSuite(0.15); % 15% weather/dust factor
fusion_engine = bharatdrive.perception.SensorFusionEngine(dt);

% 4. Create Indian Road Scenario Ground Truth Actors (Scenario 5: Cattle Crossing)
% Ego vehicle traveling at 13.89 m/s (50 km/h)
ego_v = 13.89;
ego_x = 10.0;
ego_y = -1.75; % Left lane
ego_yaw = 0.0;

% Target obstacles on road
actors(1).ActorID = 101;
actors(1).ObjectClass = 'Cattle';
actors(1).Position = [75.0, -1.50, 0.0]; % Stationary lead cow in ego path
actors(1).Velocity = [0.0, 0.0, 0.0];
actors(1).Dimensions = [2.20, 0.90, 1.50];

actors(2).ActorID = 102;
actors(2).ObjectClass = 'Cattle';
actors(2).Position = [70.0, -3.80, 0.0]; % Calf sprinting from shoulder
actors(2).Velocity = [0.0, 1.80, 0.0];  % Lateral velocity towards roadway
actors(2).Dimensions = [1.30, 0.60, 1.00];

actors(3).ActorID = 103;
actors(3).ObjectClass = 'Truck-Bus';
actors(3).Position = [140.0, 1.75, 0.0]; % Oncoming State Transport Bus
actors(3).Velocity = [-12.50, 0.0, 0.0]; % Oncoming at 45 km/h
actors(3).Dimensions = [11.20, 2.60, 3.20];

actors(4).ActorID = 104;
actors(4).ObjectClass = 'Two-Wheeler';
actors(4).Position = [2.0, -1.75, 0.0];  % Motorcycle following close behind
actors(4).Velocity = [13.89, 0.0, 0.0];
actors(4).Dimensions = [2.00, 0.80, 1.30];

fprintf('Simulating %d steps (%0.1fs) of multi-sensor tracking...\n\n', time_steps, total_time);

% 5. Main Perception Loop
for step = 1:time_steps
    t = step * dt;

    % Propagate ground truth actors
    ego_x = ego_x + ego_v * dt;
    for i = 1:length(actors)
        actors(i).Position = actors(i).Position + actors(i).Velocity * dt;
    end

    ego_state = [ego_x, ego_y, ego_v, ego_yaw];

    % A. Generate raw synthetic detections from Camera, LiDAR, Radar
    raw_measurements = sensor_suite.generate_synthetic_measurements(ego_state, actors, t);

    % B. Execute multi-sensor fusion (Predict -> Associate -> EKF Update -> Manage)
    fused_tracks = fusion_engine.step(raw_measurements, t);

    % C. Display telemetry at key intervals (e.g. at t = 1.0s and t = 2.5s)
    if step == 20 || step == 50
        fprintf('---------------------------------------------------------------------------\n');
        fprintf('TELEMETRY SNAPSHOT AT t = %0.2f s | Ego Speed = %0.1f km/h\n', t, ego_v * 3.6);
        fprintf('%-8s | %-13s | %-14s | %-12s | %-8s | %-10s | %-15s\n', ...
            'TrackID', 'Object Class', 'Position [X, Y]', 'Velocity', 'Distance', 'Confidence', 'Sensors Fused');
        fprintf('---------------------------------------------------------------------------\n');

        for k = 1:length(fused_tracks)
            tr = fused_tracks{k};
            pos_str = sprintf('[%5.1f, %4.1f]', tr.Position(1), tr.Position(2));
            vel_str = sprintf('[%4.1f, %4.1f]', tr.Velocity(1), tr.Velocity(2));
            sensors_str = strjoin(tr.SensorContributors, '+');
            if isempty(sensors_str)
                sensors_str = 'Coasted';
            end

            fprintf('ID #%-5d | %-13s | %-14s | %-12s | %6.1fm | %9.1f%% | %-15s\n', ...
                tr.ObjectID, tr.ObjectClass, pos_str, vel_str, tr.Distance, tr.ConfidenceScore * 100, sensors_str);
        end
        fprintf('---------------------------------------------------------------------------\n\n');
    end
end

fprintf('Perception Simulation Completed Successfully!\n');
fprintf('All required outputs (Object, Position, Velocity, Distance, Confidence) generated.\n');
