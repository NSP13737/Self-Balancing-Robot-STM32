function y = chassisSim(t_sim, params, m_c, l_geom)

    J_c = params(1);
    b_theta = params(2);
    theta0 = params(3);
    dtheta0 = params(4);

    % Pendulum ODE: solve for ddtheta
    % (J_c + m_c*l_geom^2)*ddtheta + b_theta*dtheta ...
    % + m_c*g*l_geom*sin(theta) = 0

    g = 9.81;

    ddtheta = @(x) -(b_theta*x(1) ...
        + m_c*g*l_geom*sin(x(2))) ...
        / (J_c + m_c*l_geom^2);

    dx = @(t,x) [ddtheta(x); x(1)];

    [~, x_sim] = ode45(dx, t_sim, [dtheta0; theta0]);

    y = x_sim(:, 2);

    % Return simulated angle vector

end

% Initial guesses: [J_c, b_theta, theta0, dtheta0]
params0 = [1e-3, 1e-3, theta_measured(1), 0];

% Define optimization cost
cost = @(params) norm(theta_measured - ...
    chassisSim(time, params, m_c, l_geom));

% Optimize parameters
[params_opt, costVal] = fminsearch(cost, params0);

J_c_opt = params_opt(1);