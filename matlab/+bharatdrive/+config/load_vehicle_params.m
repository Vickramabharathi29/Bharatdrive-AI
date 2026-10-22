function params = load_vehicle_params(yaml_path)
% LOAD_VEHICLE_PARAMS Loads vehicle parameters for BharatDrive-AI.
%
% Returns a struct containing dimensions, mass properties, kinematic limits,
% and tire cornering stiffness matching Tata Nexon EV / Indian compact EV class.

    if nargin < 1 || isempty(yaml_path)
        yaml_path = fullfile(fileparts(mfilename('fullpath')), '..', '..', '..', 'config', 'vehicle_params.yaml');
    end

    % Default calibrated parameters
    params.name = 'BharatDrive_Ego_EV';
    
    % Geometric dimensions (m)
    params.dimensions.length = 3.993;
    params.dimensions.width = 1.811;
    params.dimensions.height = 1.606;
    params.dimensions.wheelbase = 2.498;
    params.dimensions.front_overhang = 0.780;
    params.dimensions.rear_overhang = 0.715;
    params.dimensions.track_width = 1.520;
    
    % Mass and Inertia
    params.mass.m = 1400.0;             % kg
    params.mass.Iz = 2250.0;            % kg*m^2
    params.mass.lf = 1.150;             % Distance CG to front axle (m)
    params.mass.lr = 1.348;             % Distance CG to rear axle (m)
    
    % Kinematic and Comfort Limits
    params.limits.max_steer_rad = 0.55;        % ~31.5 deg
    params.limits.max_steer_rate = 0.60;       % rad/s
    params.limits.max_velocity = 22.22;        % m/s (80 km/h)
    params.limits.min_velocity = 0.0;          % m/s
    params.limits.max_accel = 3.5;             % m/s^2
    params.limits.max_decel = -6.5;            % m/s^2
    params.limits.comfort_accel = 1.8;         % m/s^2
    params.limits.comfort_decel = -2.5;        % m/s^2
    params.limits.max_lat_accel = 3.5;         % m/s^2
    params.limits.max_lat_jerk = 2.0;          % m/s^3
    
    % Tires
    params.tires.Cf = 80000.0;                 % N/rad (Front cornering stiffness)
    params.tires.Cr = 85000.0;                 % N/rad (Rear cornering stiffness)
    params.tires.mu = 0.85;                    % Friction coefficient
end
