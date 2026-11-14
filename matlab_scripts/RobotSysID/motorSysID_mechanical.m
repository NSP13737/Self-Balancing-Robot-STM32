clear
clc
close all

load('Left_FreewheelSpin_5s_20261113_172903.mat')
time = motor.time;
voltage = motor.voltage;
velocity = motor.velocity;
current = motor.current;

N = 30;
Ki = 0.0070; %derived from motorSysID_torque.m

params0 = [0.03, 0.06, 0.06]; % these are our initial guesses for Jw, bf, br

tau_m_in = @(t)  interp1(time, Ki*N*current, t);
function simulated_w_a = simulated_velocity(t_sim, params, tau_m_in, N)
    Jm = params(1);
    bf = params(2);
    br = params(3);

    function output = resistanceTorque(w_m)
        if (w_m >= 0)
            output = bf*w_m;
        else
            output = br*w_m;
        end
    end


    dw_m = @(t,w_m) (1/Jm)*(tau_m_in(t) - resistanceTorque(w_m));

    w_m0 = 0; % initial angular velo
    [~,simulated_w_m] = ode45(dw_m,t_sim,w_m0);

    simulated_w_a = N * simulated_w_m;
end

cost = @(params) norm(simulated_velocity(time, params, tau_m_in, N) - velocity);

[params_opt, costVal] = fminsearch(cost,params0)

%% Plot Results in Subplots
% Load validation dataset
load('Left_FreewheelSpin_3s_20261113_173009.mat')
time2     = motor.time;
voltage2  = motor.voltage;
velocity2 = motor.velocity;
current2  = motor.current;

% Input torque interpolation for validation dataset
tau_m_in2 = @(t) interp1(time2, Ki * N * current2, t);

% Simulate velocities
predictedVelo  = simulated_velocity(time, params_opt, tau_m_in, N);
predictedVelo2 = simulated_velocity(time2, params_opt, tau_m_in2, N);

figure(1)

% Subplot 1: Training Dataset
subplot(2, 1, 1)
plot(time, velocity)
hold on
plot(time, predictedVelo, '+')
grid on
title('Training Velocity Dataset (based on optimized J, bf, br)')
xlabel('Time (s)')
ylabel('\omega_a (rad/sec)')
legend('Measured velocity', 'Predicted velocity')
hold off

% Subplot 2: Testing / Validation Dataset
subplot(2, 1, 2)
plot(time2, velocity2)
hold on
plot(time2, predictedVelo2, '+')
grid on
title('Validation Velocity Dataset (based on optimized J, bf, br)')
xlabel('Time (s)')
ylabel('\omega_a (rad/sec)')
legend('Measured velocity', 'Predicted velocity')
hold off