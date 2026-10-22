classdef StanleyController < handle
    % STANLEYCONTROLLER Front-Axle Non-Linear Tracking Controller.
    % Winner of the DARPA Grand Challenge (Stanford Racing Team).
    % Enhanced with Curvature Feedforward and High-Speed Yaw Damping for Indian Roads.
    % BharatDrive-AI - MathWorks Autonomous Driving Hackathon.

    properties
        Wheelbase           % Distance between front and rear axle L (m) [2.498m]
        MaxSteer            % Maximum front steering angle (rad) [0.55 rad]
        K                   % Cross-track gain factor [default 0.90]
        Ksoft               % Softening velocity parameter (m/s) [default 1.0 m/s]
        Kd                  % Yaw-rate damping gain [default 0.15]
        EnableFeedforward   % Toggle curvature feedforward (Boolean)
    end

    methods
        function obj = StanleyController(wheelbase, max_steer, k, ksoft, kd)
            if nargin < 5, kd = 0.15; end
            if nargin < 4, ksoft = 1.0; end
            if nargin < 3, k = 2.20; end
            if nargin < 2, max_steer = 0.55; end
            if nargin < 1, wheelbase = 2.498; end

            obj.Wheelbase = wheelbase;
            obj.MaxSteer = max_steer;
            obj.K = k;
            obj.Ksoft = ksoft;
            obj.Kd = kd;
            obj.EnableFeedforward = true;
        end

        function [steer_cmd, ey_f, e_theta] = compute_steering(obj, ego_state, waypoints, yaw_rate, kappa_ref)
            % COMPUTE_STEERING Evaluates front-axle Stanley control law.
            %
            % ego_state: [x_r, y_r, v, yaw]
            % waypoints: Nx2 [x, y] or Nx3 [x, y, yaw]
            % yaw_rate: current vehicle yaw rate (rad/s) [optional, default 0]
            % kappa_ref: road curvature at target point (1/m) [optional, default 0]
            %
            % Returns:
            %   steer_cmd: Saturated commanded front steering angle (rad)
            %   ey_f: Signed lateral cross-track error at front axle (m)
            %   e_theta: Heading alignment error (rad)

            if nargin < 5, kappa_ref = 0.0; end
            if nargin < 4, yaw_rate = 0.0; end

            xr = ego_state(1);
            yr = ego_state(2);
            v = max(0.1, ego_state(3));
            yaw = ego_state(4);

            % 1. Front Axle Coordinates
            xf = xr + obj.Wheelbase * cos(yaw);
            yf = yr + obj.Wheelbase * sin(yaw);

            % 2. Find Nearest Waypoint to Front Axle
            dists = hypot(waypoints(:, 1) - xf, waypoints(:, 2) - yf);
            [~, nearest_idx] = min(dists);
            target_pt = waypoints(nearest_idx, :);

            % 3. Reference Path Heading at Nearest Waypoint
            num_pts = size(waypoints, 1);
            if size(waypoints, 2) >= 3
                path_yaw = waypoints(nearest_idx, 3);
            else
                if nearest_idx < num_pts
                    dx = waypoints(nearest_idx + 1, 1) - waypoints(nearest_idx, 1);
                    dy = waypoints(nearest_idx + 1, 2) - waypoints(nearest_idx, 2);
                elseif nearest_idx > 1
                    dx = waypoints(nearest_idx, 1) - waypoints(nearest_idx - 1, 1);
                    dy = waypoints(nearest_idx, 2) - waypoints(nearest_idx - 1, 2);
                else
                    dx = 1.0; dy = 0.0;
                end
                path_yaw = atan2(dy, dx);
            end

            % 4. Heading Alignment Error: e_theta = wrapToPi(path_yaw - ego_yaw)
            e_theta = atan2(sin(path_yaw - yaw), cos(path_yaw - yaw));

            % 5. Signed Front-Axle Cross-Track Error
            % Vector from front axle to nearest path point
            dx_f = xf - target_pt(1);
            dy_f = yf - target_pt(2);
            % Dot with normal vector [-sin(path_yaw), cos(path_yaw)]
            ey_f = -dx_f * sin(path_yaw) + dy_f * cos(path_yaw);

            % 6. Non-linear Cross-Track Steering Correction (steers toward path)
            delta_e = atan2(-obj.K * ey_f, v + obj.Ksoft);

            % 7. Curvature Feedforward
            delta_ff = 0.0;
            if obj.EnableFeedforward && abs(kappa_ref) > 1e-4
                delta_ff = atan(obj.Wheelbase * kappa_ref);
            end

            % 8. High-Speed Yaw-Rate Damping
            desired_yaw_rate = v * kappa_ref;
            delta_damp = -obj.Kd * (yaw_rate - desired_yaw_rate);

            % 9. Total Steering Angle Assembly
            raw_steer = e_theta + delta_e + delta_ff + delta_damp;
            steer_cmd = max(-obj.MaxSteer, min(obj.MaxSteer, raw_steer));
        end
    end
end
