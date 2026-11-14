%% swingRig_sim.m
% Chassis system identification from the swing rig (Lectorial 10):
%   angle from the rig encoder, angular velocity and acceleration by numerical
%   differentiation, and linear least squares for J and b_theta with the pendulum model
%       J*theta_ddot + b_theta*theta_dot = -m*g*l*sin(theta),   J = Jc + m*l^2 (parallel axis)
% Validation as in Lab 6 Problem 3: simulate with ode45 using the fitted J and b_theta, with
% fminsearch finding only the initial conditions of each recording.
%
% Recording (collectData_swingRig.m / logswing, 200 Hz, 10 s; t, encoderCount, theta, x_gyro):
%   - robot hanging from the rig (clamp the rig to the bench)
%   - hold it still at the release angle for ~0.5 s, release without pushing, and let it
%     settle before the log ends: the settled position defines theta = 0
%   swing_log.mat   fit
%   swing_log2.mat  validation (use a different release angle)
clear
clc
close all

%% Settings
fitFile = 'cleaned_swing_test_log_20261110_172933.mat';
valFile = 'cleaned_swing_test_log_20261110_172838.mat';
m = 0.833;          % chassis mass (kg)
l = 0.1 - (0.065/2);           % pivot to centre of mass (m) I'm subtracting the radius of the wheel since I measured from ground
g = 9.81;
countsPerRev = [1024*4];   % rig encoder counts per revolution. Leave [] to get the scale from the
                           % IMU: held-still start angle vs settled hanging angle
smoothWin = 5;       % moving-average window before differentiating

%% Load the fit data
[time, theta, radPerCount] = load_swing(fitFile, countsPerRev, []);
fprintf('Encoder scale: %.6g rad/count (%.1f counts per revolution)\n', radPerCount, 2*pi/abs(radPerCount));

%% Numerical differentiation
Ts      = mean(diff(time));
th      = movmean(theta, smoothWin);
dtheta  = gradient(th, Ts);
ddtheta = gradient(dtheta, Ts);

%% Linear least squares
% theta_ddot = -(m*g*l/J)*sin(theta) - (b_theta/J)*theta_dot
Y   = ddtheta;
Phi = [-sin(th), -dtheta];
p   = (Phi.'*Phi)\Phi.'*Y;
J       = m*g*l/p(1);
b_theta = p(2)*J;
Jc      = J - m*l^2;      % parallel axis theorem

fprintf('\nJ  (about the pivot)          = %.5g kg m^2\n', J);
fprintf('Jc (about the centre of mass) = %.5g kg m^2\n', Jc);
fprintf('b_theta                       = %.5g N m s/rad\n', b_theta);

if J <= 0 || b_theta < 0 || Jc <= 0
    warning('Non-physical result (J, b_theta or Jc <= 0). Check m, l and the recording.');
end

%% Fit data: simulate with fitted J, b_theta (initial conditions optimised)
ic1      = fit_ics(time, theta, J, b_theta, m, g, l);
thetaSim = pendSim(time, [J; b_theta; ic1], m, g, l);
rmse1    = sqrt(mean((theta - thetaSim).^2));

figure(1)
hold on
grid on
plot(time, theta, 'LineWidth', 1.2)
plot(time, thetaSim, '--', 'LineWidth', 1.2)
hold off
xlabel('Time (s)'), ylabel('Angle (rad)')
title(sprintf('Swing rig fit data'))
legend('Measured (encoder)', 'Simulated', 'Location', 'best')

%% Validation data: same J, b_theta, only the initial conditions optimised
[timeV, thetaV] = load_swing(valFile, countsPerRev, radPerCount);
ic2       = fit_ics(timeV, thetaV, J, b_theta, m, g, l);
thetaSimV = pendSim(timeV, [J; b_theta; ic2], m, g, l);
rmse2     = sqrt(mean((thetaV - thetaSimV).^2));

figure(2)
hold on
grid on
plot(timeV, thetaV, 'LineWidth', 1.2)
plot(timeV, thetaSimV, '--', 'LineWidth', 1.2)
hold off
xlabel('Time (s)'), ylabel('Angle (rad)')
title(sprintf('Swing rig validation data'))
legend('Measured (encoder)', 'Simulated', 'Location', 'best')

fprintf('\nFit RMSE        = %.4f rad (%.2f deg)\n', rmse1, rad2deg(rmse1));
fprintf('Validation RMSE = %.4f rad (%.2f deg)\n', rmse2, rad2deg(rmse2));

save('swingParams.mat', 'J', 'Jc', 'b_theta', 'm', 'l', 'g', 'radPerCount');
fprintf('Saved swingParams.mat\n');

%% Functions
function [t, theta, k] = load_swing(file, countsPerRev, kGiven)
% Load a swing recording and convert the encoder to an angle, 0 = settled hanging position
S = load(file, 'swing');
swing = S.swing;
t      = swing.time(:) - swing.time(1);
counts = double(swing.encoderCount(:));
imu    = swing.imu_angle(:);
N      = numel(t);
nEnd   = max(10, round(0.1*N));

% Held-still start and settled end of the log
cStart = mean(counts(1));
cEnd   = mean(counts(end-nEnd+1:end));
aStart = atan2(mean(sin(imu(1))), mean(cos(imu(1))));          % circular means,
aEnd   = atan2(mean(sin(imu(end-nEnd+1:end))), mean(cos(imu(end-nEnd+1:end))));  % avoids the +-pi wrap
dA     = atan2(sin(aStart - aEnd), cos(aStart - aEnd));

if std(counts(end-nEnd+1:end)) > 2
    warning('%s: still moving at the end of the log, so the hanging position (theta = 0) is uncertain.', file);
end
if abs(cStart - cEnd) < 20
    warning('%s: the start and end positions are almost the same; release from a larger angle.', file);
end

% Counts -> radians
if ~isempty(kGiven)
    k = kGiven;                                         % use the scale from the fit file
elseif ~isempty(countsPerRev)
    k = sign(dA/(cStart - cEnd)) * 2*pi/countsPerRev;   % known scale, sign from the IMU
else
    k = dA/(cStart - cEnd);                             % scale from the IMU start/end angles
end

theta = k*(counts - cEnd);

% Start from the first recorded data point (t = 0 at index 1)
t = t - t(1);
end

function ic = fit_ics(t, theta, J, b_theta, m, g, l)
% fminsearch for the initial conditions [theta0; dtheta0] with J and b_theta fixed (Lab 6 P3)
n  = 5;
c  = polyfit(t(1:n), theta(1:n), 1);
q0 = [theta(1); c(1)];
cost = @(q) norm(theta - pendSim(t, [J; b_theta; q], m, g, l));
ic = fminsearch(cost, q0);
end

function y = pendSim(t_sim, params, m, g, l)
% Pendulum simulation (Lab 6 Problem 3 style), x = (dtheta, theta)
J       = params(1);
b_theta = params(2);
theta0  = params(3);
dtheta0 = params(4);

ddtheta = @(x) -(m*g*l*sin(x(2)) + b_theta*x(1))/J;
dtheta  = @(x) x(1);
dx = @(t,x) [ddtheta(x); dtheta(x)];

x0 = [dtheta0; theta0];
[~, xSim] = ode45(dx, t_sim, x0);
y = xSim(:,2);
end