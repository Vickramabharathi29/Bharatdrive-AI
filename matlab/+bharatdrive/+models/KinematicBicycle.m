classdef KinematicBicycle < handle
    % KINEMATICBICYCLE Non-holonomic Kinematic Bicycle Model for Autonomous Navigation.
    % Parameterized at the center of the rear axle.
    %
    % State vector: [x, y, v, yaw, steer]'
    % Inputs: accel_cmd (m/s^2), steer_cmd (rad)

    properties
        L               % Wheelbase (m)
        max_steer       % Max front wheel steer angle (rad)
        max_steer_rate  % Max steer rate (rad/s)
        max_velocity    % Max forward speed (m/s)
        min_velocity    % Min forward speed (m/s)
        max_accel       % Max longitudinal acceleration (m/s^2)
        max_decel       % Max braking deceleration (m/s^2)
    end

    methods
        function obj = KinematicBicycle(params)
            if nargin < 1 || isempty(params)
                params = bharatdrive.config.load_vehicle_params();
            end
            obj.L = params.dimensions.wheelbase;
            obj.max_steer = params.limits.max_steer_rad;
            obj.max_steer_rate = params.limits.max_steer_rate;
            obj.max_velocity = params.limits.max_velocity;
            obj.min_velocity = params.limits.min_velocity;
            obj.max_accel = params.limits.max_accel;
            obj.max_decel = params.limits.max_decel;
        end

        function dxdt = derivatives(obj, state, a, steer)
            % state: [x, y, v, yaw]
            v = state(3);
            yaw = state(4);

            dx = v * cos(yaw);
            dy = v * sin(yaw);
            dv = a;
            dyaw = (v / obj.L) * tan(steer);

            dxdt = [dx; dy; dv; dyaw];
        end

        function next_state = step(obj, current_state, a_cmd, steer_cmd, dt, method)
            % current_state: [x, y, v, yaw, steer]
            if nargin < 6
                method = 'rk4';
            end

            % Saturate commanded inputs
            a = max(obj.max_decel, min(obj.max_accel, a_cmd));
            target_steer = max(-obj.max_steer, min(obj.max_steer, steer_cmd));

            % Rate-limit steering
            current_steer = current_state(5);
            steer_diff = target_steer - current_steer;
            max_delta = obj.max_steer_rate * dt;
            actual_steer = current_steer + max(-max_delta, min(max_delta, steer_diff));

            s4 = current_state(1:4)';

            if strcmpi(method, 'rk4')
                k1 = obj.derivatives(s4, a, actual_steer);
                k2 = obj.derivatives(s4 + 0.5 * dt * k1, a, actual_steer);
                k3 = obj.derivatives(s4 + 0.5 * dt * k2, a, actual_steer);
                k4 = obj.derivatives(s4 + dt * k3, a, actual_steer);
                next_s4 = s4 + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4);
            else
                next_s4 = s4 + dt * obj.derivatives(s4, a, actual_steer);
            end

            % Wrap yaw to [-pi, pi] and clamp velocity
            next_x = next_s4(1);
            next_y = next_s4(2);
            next_v = max(obj.min_velocity, min(obj.max_velocity, next_s4(3)));
            next_yaw = atan2(sin(next_s4(4)), cos(next_s4(4)));

            next_state = [next_x, next_y, next_v, next_yaw, actual_steer];
        end
    end
end
