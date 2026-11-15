clear
clc
%close all

% Specify the saved .mat filename (located in the same folder as this script, or provide a full path)
mat_filename = 'control_loop_20261115_140646.mat';

% Resolve path relative to this script if a full path wasn't provided
if isempty(fileparts(mat_filename))
    script_dir = fileparts(mfilename('fullpath'));
    filepath = fullfile(script_dir, mat_filename);
else
    filepath = mat_filename;
end

if ~isfile(filepath)
    error('File not found: %s', filepath);
end

loaded_data = load(filepath);
if ~isfield(loaded_data, 'control_loop')
    error('The selected .mat file does not contain the "control_loop" struct.');
end
control_loop = loaded_data.control_loop;

[~, name, ext] = fileparts(filepath);
display_filename = [name, ext];

tuning_text = sprintf(['Kp=%g | Integrator=%g | mK=[%g, %g, %g, %g] | ', ...
    'Az=[%g, %g, %g, %g] | Q=diag([%g, %g, %g, %g]) | R=%g'], ...
    control_loop.tuning.Kp, control_loop.tuning.integrator_scalar, ...
    control_loop.tuning.mK, control_loop.tuning.Az, ...
    control_loop.tuning.Q_diag, control_loop.tuning.R);

title_text = {display_filename, tuning_text};

% Figure 1: Estimated states and control action
figure;
tiledlayout(2, 1);

nexttile;
plot(control_loop.time, control_loop.wheel_state, 'LineWidth', 1.2);
hold on;
plot(control_loop.time, control_loop.angle_state, 'LineWidth', 1.2);
plot(control_loop.time, control_loop.momentum_state, 'LineWidth', 1.2);
ylabel('Estimated states');
legend('Wheel', 'Angle', 'Momentum', 'Location', 'best');
grid on;

nexttile;
plot(control_loop.time, control_loop.lqr_action, 'LineWidth', 1.2);
hold on;
plot(control_loop.time, control_loop.allocated_voltage, 'LineWidth', 1.2);
ylabel('Control');
xlabel('Time [s]');
legend('LQR action', 'Allocated voltage', 'Location', 'best');
grid on;

sgtitle(title_text, 'Interpreter', 'none');

% Figure 2: Raw IMU, current, voltage, velocity, and encoder position
figure;
tiledlayout(5, 1);

nexttile;
plot(control_loop.time, control_loop.raw_imu_angle, 'LineWidth', 1.2);
hold on;
plot(control_loop.time, control_loop.raw_imu_angular_velocity, 'LineWidth', 1.2);
ylabel('Raw IMU');
legend('Angle [rad]', 'Angular velocity [rad/s]', 'Location', 'best');
grid on;

nexttile;
plot(control_loop.time, control_loop.left_current, 'LineWidth', 1.2);
hold on;
plot(control_loop.time, control_loop.right_current, 'LineWidth', 1.2);
ylabel('Current [A]');
legend('Left', 'Right', 'Location', 'best');
grid on;

nexttile;
plot(control_loop.time, control_loop.left_voltage, 'LineWidth', 1.2);
hold on;
plot(control_loop.time, control_loop.right_voltage, 'LineWidth', 1.2);
ylabel('Voltage [V]');
legend('Left', 'Right', 'Location', 'best');
grid on;

nexttile;
plot(control_loop.time, control_loop.left_velocity, 'LineWidth', 1.2);
hold on;
plot(control_loop.time, control_loop.right_velocity, 'LineWidth', 1.2);
plot(control_loop.time, control_loop.average_velocity, '--', 'LineWidth', 1.2);
ylabel('Velocity [rad/s]');
legend('Left', 'Right', 'Average', 'Location', 'best');
grid on;

nexttile;
plot(control_loop.time, control_loop.left_encoder_position, 'LineWidth', 1.2);
hold on;
plot(control_loop.time, control_loop.right_encoder_position, 'LineWidth', 1.2);
ylabel('Encoder [rad]');
xlabel('Time [s]');
legend('Left', 'Right', 'Location', 'best');
grid on;

sgtitle(title_text, 'Interpreter', 'none');