function [camera_detections, lidar_clusters, radar_targets] = block_sensor_suite(ego_bus, sim_time)
%#codegen
% BLOCK_SENSOR_SUITE Camera + LiDAR + Doppler Radar Sensor Suite Subsystem.
% BharatDrive-AI - MathWorks Autonomous Driving Hackathon.
%
% Inputs:
%   ego_bus: BusVehicleState [x, y, v, yaw, yaw_rate, accel, steer, lateral_slip]
%   sim_time: current simulation time (s)
%
% Outputs:
%   camera_detections: 5x5 array [u, v, bbox_w, bbox_h, class_id]
%   lidar_clusters:    5x4 array [x_rel, y_rel, z_rel, cluster_pts]
%   radar_targets:     5x4 array [range, range_rate, azimuth_deg, rcs_db]

% Deterministic scenario obstacles in world coordinates
% Obs 1: Cow (x=24, y=0.5)
% Obs 2: Auto-Rickshaw (x=38, y=-1.5, moving vx=6.5)
% Obs 3: Pothole (x=55, y=-1.2)
% Obs 4: Oncoming Bus (x=90, y=2.0, moving vx=-10.0)
% Obs 5: Pedestrian on shoulder (x=70, y=-2.5)

persistent obs_world
if isempty(obs_world)
    obs_world = [ ...
        24.0,  0.5,  0.0,  0.0, 7.0; ...  % Cow (class 7)
        38.0, -1.5,  6.5,  0.0, 5.0; ...  % Auto (class 5)
        55.0, -1.2,  0.0,  0.0, 11.0; ... % Pothole (class 11)
        90.0,  2.0, -10.0, 0.0, 2.0; ...  % Bus (class 2)
        70.0, -2.5,  0.0,  0.0, 9.0];     % Pedestrian (class 9)
end

camera_detections = zeros(5, 5);
lidar_clusters = zeros(5, 4);
radar_targets = zeros(5, 4);

ego_x = ego_bus.x;
ego_y = ego_bus.y;
ego_v = ego_bus.v;
ego_yaw = ego_bus.yaw;

for i = 1:5
    % Propagate dynamic obstacles
    ox = obs_world(i, 1) + obs_world(i, 3) * sim_time;
    oy = obs_world(i, 2) + obs_world(i, 4) * sim_time;
    ovx = obs_world(i, 3);
    class_id = obs_world(i, 5);

    % Transform to vehicle relative frame
    dx = ox - ego_x;
    dy = oy - ego_y;

    x_rel = dx * cos(ego_yaw) + dy * sin(ego_yaw);
    y_rel = -dx * sin(ego_yaw) + dy * cos(ego_yaw);
    range = hypot(x_rel, y_rel);
    azimuth = atan2(y_rel, x_rel);

    % Camera Sensor (120 deg FOV, max 80m)
    if x_rel > 1.0 && x_rel < 80.0 && abs(azimuth) < deg2rad(60.0)
        camera_detections(i, :) = [x_rel, y_rel, 2.0, 1.5, class_id];
    end

    % 32-Channel LiDAR Sensor (360 deg, max 100m)
    if range > 0.5 && range < 100.0
        lidar_clusters(i, :) = [x_rel, y_rel, 0.5, 64.0];
    end

    % 77 GHz Doppler Radar (Direct relative velocity)
    if range > 1.0 && range < 150.0 && abs(azimuth) < deg2rad(45.0)
        range_rate = (ovx - ego_v) * cos(azimuth);
        radar_targets(i, :) = [range, range_rate, rad2deg(azimuth), 15.0];
    end
end

end
