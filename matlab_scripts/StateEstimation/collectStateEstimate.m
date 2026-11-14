clear
clc

% stateEstimateLog emits time, wheel velocity, fused angle, and p_theta.
% The firmware samples at 5 ms for 5 seconds (200 Hz).
sample_period_s = 0.005;
expected_samples = 5 / sample_period_s;

stm_device = serialport('COM3', 115200, 'Timeout', 0.5);
configureTerminator(stm_device, "LF");
cleanupObj = onCleanup(@() clear('stm_device')); %#ok<NASGU>

% Allow the board to finish any prior output, then discard stale serial data.
pause(1.0);
flush(stm_device);
writeline(stm_device, "stateEstimateLog");

estimate.time = zeros(expected_samples, 1);
estimate.wheel_position = zeros(expected_samples, 1);
estimate.angle = zeros(expected_samples, 1);
estimate.p_theta = zeros(expected_samples, 1);

% Ignore command echo and other text; retain four-field numeric CSV rows.
k = 0;
t0 = tic;
collection_timeout_s = 8.0;
while k < expected_samples && toc(t0) < collection_timeout_s
    rxStr = readline(stm_device);
    values = sscanf(char(rxStr), '%f,%f,%f,%f');
    if numel(values) ~= 4 || any(~isfinite(values))
        continue;
    end

    k = k + 1;
    estimate.time(k) = values(1);
    estimate.wheel_position(k) = values(2);
    estimate.angle(k) = values(3);
    estimate.p_theta(k) = values(4);
end

estimate.time = estimate.time(1:k);
estimate.wheel_position = estimate.wheel_position(1:k);
estimate.angle = estimate.angle(1:k);
estimate.p_theta = estimate.p_theta(1:k);

script_dir = fileparts(mfilename('fullpath'));
fname = fullfile(script_dir, sprintf('state_estimate_%s.mat', ...
    datestr(now, 'yyyymmdd_HHMMSS')));
save(fname, 'estimate');
fprintf('Saved %d of %d expected state samples to %s\n', ...
    k, expected_samples, fname);

clear stm_device

figure;
tiledlayout(3, 1);

nexttile;
plot(estimate.time, estimate.wheel_position, 'LineWidth', 1.2);
ylabel('Wheel Position [rads]');
grid on;

nexttile;
plot(estimate.time, estimate.angle, 'LineWidth', 1.2);
ylabel('Fused angle [rad]');
grid on;

nexttile;
plot(estimate.time, estimate.p_theta, 'LineWidth', 1.2);
ylabel('p_{theta}');
xlabel('Time [s]');
grid on;
