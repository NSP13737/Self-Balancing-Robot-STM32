clear
clc
close all

load('Left_FreewheelSpin_3s_20261113_173009.mat');


% Unpack data
time = motor.time;
voltage = motor.voltage;
current = motor.current;
velocity = motor.velocity;
torque = motor.torque;
% time = motorData.time;
% voltage = motorData.voltage;
% current = motorData.current;
% velocity = motorData.velocity;
% torque = motorData.torque;

figure;
t = tiledlayout(4, 1, 'TileSpacing', 'compact');

nexttile;
plot(time, voltage, 'LineWidth', 1.2);
ylabel('Voltage [V]');
grid on;

nexttile;
plot(time, current, 'LineWidth', 1.2);
ylabel('Current [A]');
grid on;

nexttile;
plot(time, velocity, 'LineWidth', 1.2);
ylabel('Velocity [rad/s]');
grid on;

nexttile;
plot(time, torque, 'LineWidth', 1.2);
ylabel('Torque [Nm]');
xlabel('Time [s]');
grid on;

linkaxes(findall(gcf, 'type', 'axes'), 'x');