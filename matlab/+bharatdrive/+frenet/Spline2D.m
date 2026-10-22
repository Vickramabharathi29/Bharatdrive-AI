classdef Spline2D < handle
    % SPLINE2D 2D Parametric Cubic Spline reference path parameterized by arc length s.

    properties
        s               % Cumulative arc-length array
        total_length    % Total path length (m)
        pp_x            % Piecewise polynomial structure for X(s)
        pp_y            % Piecewise polynomial structure for Y(s)
    end

    methods
        function obj = Spline2D(x_pts, y_pts)
            dx = diff(x_pts);
            dy = diff(y_pts);
            ds = hypot(dx, dy);
            obj.s = [0, cumsum(ds)];
            obj.total_length = obj.s(end);

            % Natural cubic spline via spline fit
            obj.pp_x = spline(obj.s, x_pts);
            obj.pp_y = spline(obj.s, y_pts);
        end

        function [x, y] = calc_position(obj, s_query)
            s_clamped = max(0, min(obj.total_length, s_query));
            x = ppval(obj.pp_x, s_clamped);
            y = ppval(obj.pp_y, s_clamped);
        end

        function theta_r = calc_tangent_angle(obj, s_query)
            s_clamped = max(0, min(obj.total_length, s_query));
            ds = 0.01;
            x1 = ppval(obj.pp_x, max(0, s_clamped - ds));
            x2 = ppval(obj.pp_x, min(obj.total_length, s_clamped + ds));
            y1 = ppval(obj.pp_y, max(0, s_clamped - ds));
            y2 = ppval(obj.pp_y, min(obj.total_length, s_clamped + ds));
            theta_r = atan2(y2 - y1, x2 - x1);
        end

        function kappa = calc_curvature(obj, s_query)
            s_clamped = max(0, min(obj.total_length, s_query));
            ds = 0.02;
            s_m = max(0, s_clamped - ds);
            s_p = min(obj.total_length, s_clamped + ds);

            th_m = obj.calc_tangent_angle(s_m);
            th_p = obj.calc_tangent_angle(s_p);
            d_th = atan2(sin(th_p - th_m), cos(th_p - th_m));
            kappa = d_th / max(1e-4, s_p - s_m);
        end

        function s_proj = find_projection(obj, x, y)
            s_grid = linspace(0, obj.total_length, 100);
            xs = ppval(obj.pp_x, s_grid);
            ys = ppval(obj.pp_y, s_grid);
            dist_sq = (xs - x).^2 + (ys - y).^2;
            [~, idx] = min(dist_sq);
            s_proj = s_grid(idx);
        end
    end
end
