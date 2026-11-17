clear
clc
close all

% Load file (adjust file name/path as needed)
load('onboard_data/onboard_imu_kf_20261115_144148.mat');

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

figure(2)
cpr = 1320.0;
wheelAngle = (voltage - voltage(1)) * (2 * pi / cpr);
plot(time, wheelAngle)