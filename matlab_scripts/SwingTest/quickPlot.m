clear
clc
close all

% ---- Load data ----------------------------------------------------------
% Option A: set a specific file name here, e.g. 'swing_test_log_20261110_141500.mat'
fname = '';

if isempty(fname)
    % Option B: automatically pick the most recent swing_test_log_*.mat
    files = dir('cleaned_swing_test_log_20261110_172725.mat');
    if isempty(files)
        [f, p] = uigetfile('*.mat', 'Select a swing test log');
        if isequal(f, 0), error('No file selected.'); end
        fname = fullfile(p, f);
    else
        [~, idx] = max([files.datenum]);
        fname = fullfile(files(idx).folder, files(idx).name);
    end
end

S = load(fname, 'swing');
swing = S.swing;
fprintf('Loaded %d samples from %s\n', numel(swing.time), fname);

t = swing.time;

% ---- Plot ---------------------------------------------------------------
fig = figure('Name', 'Swing test', 'Color', 'w');
tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, 'Swing test log', 'Interpreter', 'none');

ax(1) = nexttile(tl);
plot(t, double(swing.encoderCount), 'LineWidth', 1.2);
ylabel('Encoder count');
grid on;

ax(2) = nexttile(tl);
plot(t, swing.imu_angle, 'LineWidth', 1.2);
ylabel('IMU angle');
grid on;

ax(3) = nexttile(tl);
plot(t, swing.imu_ang_velo, 'LineWidth', 1.2);
ylabel('IMU angular velocity');
xlabel('Time (s)');
grid on;

% Hide x tick labels on the upper plots and link x-axes so zoom/pan is shared
set(ax(1:2), 'XTickLabel', []);
linkaxes(ax, 'x');
xlim(ax(1), [t(1) t(end)]);