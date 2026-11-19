clear
clc
close all

% controlLoopLog emits up to 32 CSV fields per sample, then an END,count marker.
sample_period_s = 0.005;
expected_samples = 5 / sample_period_s;
field_count = 32;
csv_format = [repmat('%f,', 1, field_count - 1), '%f'];

% The firmware buffers the capture, then streams the rows and an end marker.
stm_device = serialport('COM3', 115200, 'Timeout', 10.0);
configureTerminator(stm_device, "LF");
cleanupObj = onCleanup(@() clear('stm_device')); %#ok<NASGU>

% Allow the board to finish prior output, then discard stale serial data.
pause(1.0);
flush(stm_device);
writeline(stm_device, "controlLoopLog");

% Columns: time, three estimated states, LQR action, allocated voltage,
% raw IMU angle/rate, left/right current, left/right voltage,
% left/right/average wheel speed, and left/right encoder position.
data = zeros(expected_samples, field_count);
k = 0;
t0 = tic;
collection_timeout_s = 60.0;
capture_complete = false;

while ~capture_complete && toc(t0) < collection_timeout_s
    rxStr = readline(stm_device);

    end_marker = regexp(char(rxStr), '^END,(\d+)$', 'tokens', 'once');
    if ~isempty(end_marker)
        reported_samples = str2double(end_marker{1});
        if reported_samples ~= k
            error('Firmware reported %d samples, but MATLAB received %d.', ...
                reported_samples, k);
        end
        capture_complete = true;
        continue;
    end

    values = sscanf(char(rxStr), csv_format);
    if numel(values) ~= field_count || any(~isfinite(values))
        continue;
    end
    if k >= expected_samples
        error('Received more than %d control-loop samples.', expected_samples);
    end
    k = k + 1;
    data(k, :) = values(:)';
end

if ~capture_complete
    error('Timed out before the firmware ended the capture. Received %d samples.', k);
end
if k == 0
    error('No control-loop samples received. The angle cutoff may have triggered immediately.');
end

data = data(1:k, :);
control_loop.sample_count = k;
control_loop.ended_early = k < expected_samples;
control_loop.time = data(:, 1);
control_loop.wheel_state = data(:, 2);
control_loop.angle_state = data(:, 3);
control_loop.momentum_state = data(:, 4);
control_loop.lqr_action = data(:, 5);
control_loop.allocated_voltage = data(:, 6);
control_loop.raw_imu_angle = data(:, 7);
control_loop.raw_imu_angular_velocity = data(:, 8);
control_loop.left_current = data(:, 9);
control_loop.right_current = data(:, 10);
control_loop.left_voltage = data(:, 11);
control_loop.right_voltage = data(:, 12);
control_loop.left_velocity = data(:, 13);
control_loop.right_velocity = data(:, 14);
control_loop.average_velocity = data(:, 15);
control_loop.left_encoder_position = data(:, 16);
control_loop.right_encoder_position = data(:, 17);
control_loop.tuning.Kp = data(1, 18);
control_loop.tuning.integrator_scalar = data(1, 19);
control_loop.tuning.mK = data(1, 20:23);
control_loop.tuning.Az = data(1, 24:27);
control_loop.tuning.Q_diag = data(1, 28:31);
control_loop.tuning.R = data(1, 32);

script_dir = fileparts(mfilename('fullpath'));
fname = fullfile(script_dir, sprintf('control_loop_%s.mat', ...
    datestr(now, 'yyyymmdd_HHMMSS')));
save(fname, 'control_loop');
fprintf('Saved %d of up to %d control-loop samples to %s\n', ...
    k, expected_samples, fname);

clear stm_device

[~, name, ext] = fileparts(fname);
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
