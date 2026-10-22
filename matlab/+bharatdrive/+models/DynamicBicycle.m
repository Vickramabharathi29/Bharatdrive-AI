classdef DynamicBicycle < handle
    % DYNAMICBICYCLE 2-DOF Vehicle Dynamic Model with Lateral Tire Slip.
    % Parameterized at Center of Gravity (CG).
    %
    % State vector: [x, y, vx, vy, yaw, omega, steer]'
    % Inputs: accel_cmd (m/s^2), steer_cmd (rad)

    properties
        m       % Total mass (kg)
        Iz      % Yaw moment of inertia (kg*m^2)
        lf      % Distance from CG to front axle (m)
        lr      % Distance from CG to rear axle (m)
        L       % Total wheelbase (m)
        Cf      % Front cornering stiffness (N/rad)
        Cr      % Rear cornering stiffness (N/rad)
        mu      % Tire-road friction coefficient
        g       % Gravitational acceleration (m/s^2)
        Kus     % Understeer gradient (rad/(m/s^2))
    end

    methods
        function obj = DynamicBicycle(params)
            if nargin < 1 || isempty(params)
                params = bharatdrive.config.load_vehicle_params();
            end
            obj.m = params.mass.m;
            obj.Iz = params.mass.Iz;
            obj.lf = params.mass.lf;
            obj.lr = params.mass.lr;
            obj.L = obj.lf + obj.lr;
            obj.Cf = params.tires.Cf;
            obj.Cr = params.tires.Cr;
            obj.mu = params.tires.mu;
            obj.g = 9.81;

            % Understeer gradient: Kus = (m / L) * (lr / (2*Cf) - lf / (2*Cr))
            obj.Kus = (obj.m / obj.L) * ((obj.lr / (2.0 * obj.Cf)) - (obj.lf / (2.0 * obj.Cr)));
        end

        function [alpha_f, alpha_r] = compute_slip_angles(obj, vx, vy, omega, delta)
            if abs(vx) < 0.2
                alpha_f = 0.0;
                alpha_r = 0.0;
                return;
            end
            alpha_f = delta - atan2(vy + obj.lf * omega, vx);
            alpha_r = -atan2(vy - obj.lr * omega, vx);
        end

        function [Fy_f, Fy_r] = compute_tire_forces(obj, alpha_f, alpha_r)
            Fz_f = (obj.m * obj.g * obj.lr) / obj.L;
            Fz_r = (obj.m * obj.g * obj.lf) / obj.L;

            Fy_max_f = obj.mu * Fz_f;
            Fy_max_r = obj.mu * Fz_r;

            % Saturated linear tire model
            Fy_f = max(-Fy_max_f, min(Fy_max_f, 2.0 * obj.Cf * alpha_f));
            Fy_r = max(-Fy_max_r, min(Fy_max_r, 2.0 * obj.Cr * alpha_r));
        end

        function next_state = step(obj, current_state, a_cmd, steer_cmd, dt)
            % current_state: [x, y, vx, vy, yaw, omega, steer]
            x = current_state(1);
            y = current_state(2);
            vx = max(0.1, current_state(3));
            vy = current_state(4);
            yaw = current_state(5);
            omega = current_state(6);

            delta = steer_cmd;
            ax = a_cmd;

            [alpha_f, alpha_r] = obj.compute_slip_angles(vx, vy, omega, delta);
            [Fy_f, Fy_r] = obj.compute_tire_forces(alpha_f, alpha_r);

            % Dynamic derivatives in vehicle body frame
            d_vx = ax + vy * omega;
            d_vy = (Fy_f * cos(delta) + Fy_r) / obj.m - vx * omega;
            d_omega = (obj.lf * Fy_f * cos(delta) - obj.lr * Fy_r) / obj.Iz;

            % Global frame velocities
            dx = vx * cos(yaw) - vy * sin(yaw);
            dy = vx * sin(yaw) + vy * cos(yaw);

            % Update state
            next_x = x + dx * dt;
            next_y = y + dy * dt;
            next_vx = max(0.0, vx + d_vx * dt);
            next_vy = vy + d_vy * dt;
            next_yaw = atan2(sin(yaw + omega * dt), cos(yaw + omega * dt));
            next_omega = omega + d_omega * dt;

            next_state = [next_x, next_y, next_vx, next_vy, next_yaw, next_omega, delta];
        end
    end
end
