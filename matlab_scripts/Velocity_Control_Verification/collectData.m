clear
clc

% Configure the motor and target speed (rad/s) for this run.
motor_name = "right";
target_speed_rad_s = -15.0;

% setVel prints velocity,current,duty_cycle as comma-separated values every 5 ms.
sample_period_s = 0.005;
expected_samples = 5 / sample_period_s;

stm_device = serialport('COM3', 115200, 'Timeout', 0.5);
configureTerminator(stm_device, "LF");
cleanupObj = onCleanup(@() clear('stm_device'));

% Reset the controller, discard reset output, then clear any remaining bytes.
%writeline(stm_device, "reset");
pause(1.0);
flush(stm_device);

command_str = sprintf("setVel %s %.6f", motor_name, target_speed_rad_s);
writeline(stm_device, command_str);

motor.time = zeros(expected_samples, 1);
motor.velocity = zeros(expected_samples, 1);
motor.current = zeros(expected_samples, 1);
motor.duty_cycle = zeros(expected_samples, 1);
motor.target_velocity = target_speed_rad_s;
motor.name = motor_name;

% Ignore command text and retain lines containing three numeric CSV fields:
% measured velocity [rad/s], current [A], and motor duty_cycle [%].
k = 0;
t0 = tic;
collection_timeout_s = 8.0;
while k < expected_samples && toc(t0) < collection_timeout_s
    rxStr = readline(stm_device);
    values = sscanf(char(rxStr), '%f,%f,%d');
    if numel(values) ~= 3 || any(~isfinite(values))
        continue;
    end

    k = k + 1;
    motor.time(k) = toc(t0);
    motor.velocity(k) = values(1);
    motor.current(k) = values(2);
    motor.duty_cycle(k) = values(3);
end

motor.time = motor.time(1:k);
motor.velocity = motor.velocity(1:k);
motor.current = motor.current(1:k);
motor.duty_cycle = motor.duty_cycle(1:k);

fname = sprintf('%s_SetVel_%.2f_rad_s_%s.mat', motor_name, ...
    target_speed_rad_s, datestr(now, 'yyyymmdd_HHMMSS'));
save(fname, 'motor');
fprintf('Saved %d of %d expected velocity samples to %s\n', ...
    k, expected_samples, fname);

clear stm_device

subplot(3,1,1)
plot(motor.time, motor.velocity, 'LineWidth', 1.2);
ylabel('Velocity [rad/s]');
grid on;

subplot(3,1,2)
plot(motor.time, motor.current, 'LineWidth', 1.2);
ylabel('Current [A]');
grid on;

subplot(3,1,3)
plot(motor.time, motor.duty_cycle, 'LineWidth', 1.2);
ylabel('Duty Cycle [%]');
grid on;