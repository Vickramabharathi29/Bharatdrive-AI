function cart_state = frenet_to_cartesian(frenet_state, spline_ref)
% FRENET_TO_CARTESIAN Transforms Frenet state [s, s_d, s_dd, d, d_d, d_dd] to Cartesian [x, y, v, yaw, a].

    s = max(0, min(spline_ref.total_length, frenet_state.s));
    [rx, ry] = spline_ref.calc_position(s);
    theta_r = spline_ref.calc_tangent_angle(s);
    kappa_r = spline_ref.calc_curvature(s);

    d = frenet_state.d;
    x = rx - d * sin(theta_r);
    y = ry + d * cos(theta_r);

    denom = frenet_state.s_d * (1.0 - kappa_r * d);
    if abs(denom) < 1e-4
        delta_theta = 0.0;
    else
        delta_theta = atan2(frenet_state.d_d, denom);
    end

    yaw = atan2(sin(theta_r + delta_theta), cos(theta_r + delta_theta));
    v = hypot(denom, frenet_state.d_d);
    a = frenet_state.s_dd * cos(delta_theta) + frenet_state.d_dd * sin(delta_theta);

    cart_state.x = x;
    cart_state.y = y;
    cart_state.v = v;
    cart_state.yaw = yaw;
    cart_state.a = a;
end
