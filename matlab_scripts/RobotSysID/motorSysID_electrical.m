clear
clc
close all

load('usefulData/RIGHTdata_sine_5s_20261110_151859.mat');


% Unpack data
time = motor.time;
voltage = motor.voltage;
current = motor.current;
velocity = motor.velocity;

% set up linear least squares for closed form solution
% measurements are already in columns which is convinient
Y = voltage;
Phi = [velocity, current];

% closed form solution
Theta = (Phi.' * Phi) \ (Phi.' * Y);
Kw = Theta(1)
Ra = Theta(2)

%% Validating measurements on second set of data
load('usefulData/RIGHTdata_sine_1s_20261110_152438.mat');
time2 = motor.time;
voltage2 = motor.voltage;
current2 = motor.current;
velocity2 = motor.velocity;

figure(1)

% Subplot 1: Training Dataset
subplot(2,1,1)
plot(time, voltage)
hold on
plot(time, Ra*current + Kw*velocity, '+')
grid on
title('Training Dataset')
legend("Measured voltage", "Reconstructed Voltage")
hold off

% Subplot 2: Testing Dataset
subplot(2,1,2)
plot(time2, voltage2)
hold on
plot(time2, Ra*current2 + Kw*velocity2, '+')
grid on
title('Testing Dataset')
legend("Measured voltage", "Reconstructed Voltage")
hold off