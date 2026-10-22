classdef VehicleControllerEngine < handle
    % VEHICLECONTROLLERENGINE Unified Motion Control Architecture for Autonomous Driving.
    % Integrates Adaptive Stanley Lateral Control with Anti-Windup Longitudinal PID.
    % BharatDrive-AI - MathWorks Autonomous Driving Hackathon.

    properties
        LateralController       % Primary lateral controller (StanleyController by default)
        LongitudinalController  % Longitudinal PID controller
        ControllerType          % 'Stanley', 'PurePursuit', or 'PID'
        StanleyCtrl             % Instance of StanleyController
        PurePursuitCtrl         % Instance of PurePursuitController
        LateralPIDCtrl          % Instance of PIDController
        Wheelbase               % Vehicle wheelbase L (m) [2.498m]
        MaxSteer                % Max steering angle (rad) [0.55 rad]
        MaxAccel                % Max drive acceleration (m/s^2) [3.5]
        MaxDecel                % Max emergency braking deceleration (m/s^2) [-6.5]
    end

    methods
        function obj = VehicleControllerEngine(controller_type)
            if nargin < 1, controller_type = 'Stanley'; end

            obj.ControllerType = controller_type;
            obj.Wheelbase = 2.498;
            obj.MaxSteer = 0.55;
            obj.MaxAccel = 3.50;
            obj.MaxDecel = -6.50;

            % Instantiate all 3 lateral controllers for dynamic switching & benchmarking
            obj.StanleyCtrl = bharatdrive.control.StanleyController(obj.Wheelbase, obj.MaxSteer, 2.20, 1.0, 0.15);
            obj.PurePursuitCtrl = bharatdrive.control.PurePursuitController(obj.Wheelbase, obj.MaxSteer, 0.20, 3.5);
            obj.LateralPIDCtrl = bharatdrive.control.PIDController(0.60, 0.05, 0.20, -obj.MaxSteer, obj.MaxSteer, 0.50);

            % Longitudinal Controller (Kp=1.2, Ki=0.15, Kd=0.08)
            obj.LongitudinalController = bharatdrive.control.PIDController(1.20, 0.15, 0.08, obj.MaxDecel, obj.MaxAccel);

            obj.set_controller_type(controller_type);
        end

        function set_controller_type(obj, controller_type)
            obj.ControllerType = controller_type;
            switch lower(controller_type)
                case 'stanley'
                    obj.LateralController = obj.StanleyCtrl;
                case 'purepursuit'
                    obj.LateralController = obj.PurePursuitCtrl;
                case 'pid'
                    obj.LateralController = obj.LateralPIDCtrl;
                otherwise
                    warning('Unknown controller type %s. Defaulting to Stanley.', controller_type);
                    obj.LateralController = obj.StanleyCtrl;
            end
        end

        function [steer_cmd, accel_cmd, ey, etheta] = compute_controls(obj, ego_state, waypoints, target_v, dt, kappa_ref)
            % COMPUTE_CONTROLS Evaluates coupled lateral steering and longitudinal acceleration.
            %
            % ego_state: [x, y, v, yaw, yaw_rate]
            % waypoints: Nx2 or Nx3 [x, y, (optional yaw)]
            % target_v: reference cruise/target speed (m/s)
            % dt: sampling step (s) [default 0.01s for 100 Hz control loop]
            % kappa_ref: path curvature at target point (1/m) [optional, default 0]

            if nargin < 6, kappa_ref = 0.0; end
            if nargin < 5, dt = 0.01; end

            curr_v = ego_state(3);
            yaw_rate = 0.0;
            if length(ego_state) >= 5, yaw_rate = ego_state(5); end

            % 1. Longitudinal Acceleration Command
            accel_cmd = obj.LongitudinalController.compute_longitudinal(target_v, curr_v, dt);

            % 2. Lateral Steering Command based on selected controller
            switch lower(obj.ControllerType)
                case 'stanley'
                    [steer_cmd, ey, etheta] = obj.StanleyCtrl.compute_steering(ego_state(1:4), waypoints, yaw_rate, kappa_ref);

                case 'purepursuit'
                    [steer_cmd, ~, ~, ey] = obj.PurePursuitCtrl.compute_steering(ego_state(1:4), waypoints);
                    % Estimate heading error to nearest waypoint
                    dists = hypot(waypoints(:, 1) - ego_state(1), waypoints(:, 2) - ego_state(2));
                    [~, idx] = min(dists);
                    if size(waypoints, 2) >= 3
                        etheta = atan2(sin(waypoints(idx, 3) - ego_state(4)), cos(waypoints(idx, 3) - ego_state(4)));
                    else
                        etheta = 0.0;
                    end

                case 'pid'
                    % Find cross-track and heading error to nearest waypoint
                    dists = hypot(waypoints(:, 1) - ego_state(1), waypoints(:, 2) - ego_state(2));
                    [~, idx] = min(dists);
                    if size(waypoints, 2) >= 3
                        path_yaw = waypoints(idx, 3);
                    else
                        path_yaw = 0.0;
                    end
                    dx = ego_state(1) - waypoints(idx, 1);
                    dy = ego_state(2) - waypoints(idx, 2);
                    ey = -dx * sin(path_yaw) + dy * cos(path_yaw);
                    etheta = atan2(sin(path_yaw - ego_state(4)), cos(path_yaw - ego_state(4)));
                    steer_cmd = obj.LateralPIDCtrl.compute_lateral(ey, etheta, dt);
            end
        end
    end
end
