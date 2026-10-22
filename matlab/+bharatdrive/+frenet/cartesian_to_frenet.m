function frenet_state = cartesian_to_frenet(cart_state, spline_ref)
% CARTESIAN_TO_FRENET Transforms Cartesian state [x, y, v, yaw, a] to Frenet [s, s_d, s_dd, d, d_d, d_dd].

    x = cart_state.x;
    y = cart_state.y;
    v = cart_state.v;
    yaw = cart_state.yaw;
    a = cart_state.a;

    s = spline_ref.find_projection(x, y);
    [rx, ry] = spline_ref.calc_position(s);
    theta_r = spline_ref.calc_tangent_angle(s);
    kappa_r = spline_ref.calc_curvature(s);

    dx = x - rx;
    dy = y - ry;

    % Lateral distance d (normal vector = [-sin(theta_r), cos(theta_r)])
    d = -dx * sin(theta_r) + dy * cos(theta_r);

    delta_theta = atan2(sin(yaw - theta_r), cos(yaw - theta_r));
    denom = 1.0 - kappa_r * d;
    if abs(denom) < 1e-4
        denom = 1e-4;
    end

    s_d = (v * cos(delta_theta)) / denom;
    d_d = v * sin(delta_theta);
    d_dd = a * sin(delta_theta);
    s_dd = (a * cos(delta_theta)) / denom;

    frenet_state.s = s;
    frenet_state.s_d = s_d;
    frenet_state.s_dd = s_dd;
    frenet_state.d = d;
    frenet_state.d_d = d_d;
    frenet_state.d_dd = d_dd;
end
