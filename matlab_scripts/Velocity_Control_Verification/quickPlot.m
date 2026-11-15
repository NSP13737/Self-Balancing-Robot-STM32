clear
clc
close all

mat_filename = 'left_SetVel_7.00_rad_s_20261116_000143.mat';
load(mat_filename);

figure;
tiledlayout(3, 1);

nexttile;
plot(motor.time, motor.velocity, 'LineWidth', 1.2);
ylabel('Velocity [rad/s]');
grid on;

nexttile;
plot(motor.time, motor.current, 'LineWidth', 1.2);
ylabel('Current [A]');
grid on;

nexttile;
plot(motor.time, motor.duty_cycle, 'LineWidth', 1.2);
ylabel('Duty Cycle [%]');
xlabel('Time [s]');
grid on;

sgtitle(mat_filename, 'Interpreter', 'none');
