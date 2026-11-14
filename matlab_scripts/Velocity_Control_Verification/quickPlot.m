clear
clc
close all

load('left_SetVel_5.00_rad_s_20261114_150323.mat');


% Unpack data
time = motor.time;
velocity = motor.velocity;

figure;
t = tiledlayout(1, 1, 'TileSpacing', 'compact');


plot(time, velocity, 'LineWidth', 1.2);
ylabel('Velocity [rad/s]');
grid on;