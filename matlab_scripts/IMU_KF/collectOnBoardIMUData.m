clear
clc

% imuKFLog prints time, raw IMU angle and angular velocity, then the
% corresponding Kalman estimates. The firmware samples at 200 Hz for 5 s.
sample_period_s = 0.005;
expected_samples = 5 / sample_period_s;

stm_device = serialport('COM3', 115200, 'Timeout', 0.5);
configureTerminator(stm_device, "LF");
cleanupObj = onCleanup(@() clear('stm_device')); %#ok<NASGU>

% Discard stale serial output before starting this capture.
pause(1.0);
flush(stm_device);
writeline(stm_device, "imuKFLog");

imuData.time = zeros(expected_samples, 1);
imuData.angle = zeros(expected_samples, 1);
imuData.ang_velo = zeros(expected_samples, 1);
imuData.kalman_angle = zeros(expected_samples, 1);
imuData.kalman_ang_velo = zeros(expected_samples, 1);

% Ignore command echo or other text; only accept five numeric CSV columns.
k = 0;
t0 = tic;
collection_timeout_s = 8.0;
while k < expected_samples && toc(t0) < collection_timeout_s
    rxStr = readline(stm_device);
    values = sscanf(char(rxStr), '%f,%f,%f,%f,%f');
    if numel(values) ~= 5 || any(~isfinite(values))
        continue;
    end

    k = k + 1;
    imuData.time(k) = values(1);
    imuData.angle(k) = values(2);
    imuData.ang_velo(k) = values(3);
    imuData.kalman_angle(k) = values(4);
    imuData.kalman_ang_velo(k) = values(5);
end

imuData.time = imuData.time(1:k);
imuData.angle = imuData.angle(1:k);
imuData.ang_velo = imuData.ang_velo(1:k);
imuData.kalman_angle = imuData.kalman_angle(1:k);
imuData.kalman_ang_velo = imuData.kalman_ang_velo(1:k);

script_dir = fileparts(mfilename('fullpath'));
fname = fullfile(script_dir, sprintf('onboard_data/onboard_imu_kf_%s.mat', ...
    datestr(now, 'yyyymmdd_HHMMSS')));
save(fname, 'imuData');
fprintf('Saved %d of %d expected samples to %s\n', ...
    k, expected_samples, fname);

clear stm_device

figure;
tiledlayout(2, 1);

nexttile;
plot(imuData.time, imuData.angle, 'LineWidth', 1.0);
hold on;
plot(imuData.time, imuData.kalman_angle, 'LineWidth', 1.2);
hold off;
ylabel('Angle [rad]');
legend('Accelerometer angle', 'Kalman estimate', 'Location', 'best');
grid on;

nexttile;
plot(imuData.time, imuData.ang_velo, 'LineWidth', 1.0);
hold on;
plot(imuData.time, imuData.kalman_ang_velo, 'LineWidth', 1.2);
hold off;
ylabel('Angular velocity [rad/s]');
xlabel('Time [s]');
legend('Gyroscope angular velocity', 'Kalman estimate', 'Location', 'best');
grid on;
