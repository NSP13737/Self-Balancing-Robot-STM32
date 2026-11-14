clear
clc
close all

% Load file (adjust file name/path as needed)
load('swing_rig_osc_slow.mat');

% Unpack data
time     = imuData.time;
voltage  = imuData.voltage;
angle    = imuData.angle;
ang_velo = imuData.ang_velo;

% Plot data
figure;
t = tiledlayout(3, 1, 'TileSpacing', 'compact');

nexttile;
plot(time, voltage, 'LineWidth', 1.2);
ylabel('Voltage [V]');
grid on;

nexttile;
plot(time, angle, 'LineWidth', 1.2);
ylabel('Angle [deg]');
grid on;

nexttile;
plot(time, ang_velo, 'LineWidth', 1.2);
ylabel('Angular Velocity [deg/s]');
xlabel('Time [s]');
grid on;

linkaxes(findall(gcf, 'type', 'axes'), 'x');