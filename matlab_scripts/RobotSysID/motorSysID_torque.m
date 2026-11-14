clear
clc
close all

% Load motor data
load('usefulData/RIGHTdata_sine_5s_20261110_151859.mat')
N = 30;
% Unpack data
time = motor.time;
current = motor.current;
torque = motor.torque - 0.025;
% time = motorData.time;
% current = motorData.current;
% torque = motorData.torque;

% set up linear least squares for closed form solution
% measurements are already in columns which is convinient
Y = torque;
Phi = [current];

% closed form solution
Theta = (Phi.' * Phi) \ (Phi.' * Y);
gamma = Theta(1); % gamma has form (1/Ki)*N (where N is gear ratio)
Ki = gamma / N

% %%%% AI CODE
% Phi   = [current, ones(size(current))];
% Theta = Phi \ torque; %this is same as above just syntax different?
% gamma = Theta(1)          % Nm/A at the output shaft, expect ~0.25
% T0    = Theta(2)          % offset, expect ~ +0.12 Nm (no-load current term)
% Ki    = gamma / N         % effective motor-side Kt, expect ~0.008 Nm/A
% %reconstruction
% That = Phi * Theta;

%% Validating measurements on second set of data
load('usefulData/RIGHTdata_sine_1s_20261110_152438.mat');
time2 = motor.time;
current2 = motor.current;
torque2 = motor.torque;
% time2 = motorData.time;
% current2 = motorData.current;
% torque2 = motorData.torque;

figure(1)

% Subplot 1: Training Dataset
subplot(2,1,1)
plot(time, torque)
hold on
plot(time, Ki*N*current, '+')
% plot(time, That, '+')
grid on
title('Training Dataset')
legend("Measured torque", "Reconstructed torque")
hold off

% Subplot 2: Testing Dataset
subplot(2,1,2)
plot(time2, torque2)
hold on
plot(time2, Ki*N*current2, '+')
grid on
title('Testing Dataset')
legend("Measured torque", "Reconstructed torque")
hold off

