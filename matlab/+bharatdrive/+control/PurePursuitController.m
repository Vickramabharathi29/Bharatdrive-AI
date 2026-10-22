classdef PurePursuitController < handle
    % PUREPURSUITCONTROLLER Geometric Lookahead Path Tracking Controller.
    % Parameterized at the center of the rear axle.
    % BharatDrive-AI - MathWorks Autonomous Driving Hackathon.

    properties
        Wheelbase       % Distance between front and rear axle L (m) [2.498m]
        MaxSteer        % Maximum allowable front wheel steering angle (rad) [0.55 rad]
        Kdd             % Lookahead velocity gain factor (s) [default 0.20s]
        L0              % Minimum base lookahead distance (m) [default 3.5m]
        MinLookahead    % Lookahead saturation floor (m) [3.0m]
        MaxLookahead    % Lookahead saturation ceiling (m) [25.0m]
    end

    methods
        function obj = PurePursuitController(wheelbase, max_steer, kdd, l0)
            if nargin < 4, l0 = 3.50; end
            if nargin < 3, kdd = 0.20; end
            if nargin < 2, max_steer = 0.55; end
            if nargin < 1, wheelbase = 2.498; end

            obj.Wheelbase = wheelbase;
            obj.MaxSteer = max_steer;
            obj.Kdd = kdd;
            obj.L0 = l0;
            obj.MinLookahead = 3.0;
            obj.MaxLookahead = 25.0;
        end

        function [steer_cmd, lookahead_pt, ld, ey] = compute_steering(obj, ego_state, waypoints)
            % COMPUTE_STEERING Evaluates Pure Pursuit geometric arc steering.
            %
            % ego_state: [x, y, v, yaw]
            % waypoints: Nx2 or Nx3 [x, y, (optional yaw)]
            %
            % Returns:
            %   steer_cmd: Commanded front steering angle (rad)
            %   lookahead_pt: [x_la, y_la] target point
            %   ld: Effective lookahead distance (m)
            %   ey: Cross-track error to nearest waypoint (m)

            xr = ego_state(1);
            yr = ego_state(2);
            v = max(0.1, ego_state(3));
            yaw = ego_state(4);

            % 1. Compute dynamic lookahead distance
            ld = max(obj.MinLookahead, min(obj.MaxLookahead, obj.Kdd * v + obj.L0));

            % 2. Find nearest waypoint and cross-track error
            dists = hypot(waypoints(:, 1) - xr, waypoints(:, 2) - yr);
            [min_d, nearest_idx] = min(dists);
            ey = min_d;

            % 3. Search forward along waypoints for lookahead point at distance ld
            num_pts = size(waypoints, 1);
            target_idx = nearest_idx;
            for i = nearest_idx:num_pts
                d = hypot(waypoints(i, 1) - xr, waypoints(i, 2) - yr);
                if d >= ld
                    target_idx = i;
                    break;
                end
            end
            if target_idx == nearest_idx && num_pts > nearest_idx
                target_idx = num_pts;
            end

            lookahead_pt = waypoints(target_idx, 1:2);

            % 4. Angle alpha between vehicle heading and lookahead vector
            dx = lookahead_pt(1) - xr;
            dy = lookahead_pt(2) - yr;
            target_angle = atan2(dy, dx);
            alpha = atan2(sin(target_angle - yaw), cos(target_angle - yaw));

            % 5. Pure pursuit curvature: kappa = 2*sin(alpha) / ld
            % delta = atan(kappa * L) = atan(2 * L * sin(alpha) / ld)
            raw_steer = atan2(2.0 * obj.Wheelbase * sin(alpha), ld);
            steer_cmd = max(-obj.MaxSteer, min(obj.MaxSteer, raw_steer));
        end
    end
end
