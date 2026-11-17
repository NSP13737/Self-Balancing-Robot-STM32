clear;
clc;
close all;

% 1. Select or specify the .mat file
filePath = 'onboard_data/onboard_imu_kf_20261115_144148.mat';
if ~isfile(filePath)
    [file, path] = uigetfile('*.mat', 'Select the MAT-file to plot');
    if isequal(file, 0), return; end
    filePath = fullfile(path, file);
end

% Load data into a wrapper struct so we can inspect its contents cleanly
raw = load(filePath);
topFields = fieldnames(raw);

% 2. Extract the underlying struct or table if wrapped in a single variable
if numel(topFields) == 1 && (isstruct(raw.(topFields{1})) || istable(raw.(topFields{1})))
    data = raw.(topFields{1});
else
    data = raw;
end

% Convert table to struct for uniform field access
if istable(data)
    data = table2struct(data, 'ToScalar', true);
end

fields = fieldnames(data);
disp('Variables found in file:');
disp(fields);

% 3. Automatically identify the time vector
timeIdx = find(contains(lower(fields), 'time') | strcmpi(fields, 't'), 1);
if isempty(timeIdx)
    error('Could not automatically find a time variable. Found fields: %s', strjoin(fields, ', '));
end

timeField = fields{timeIdx};
time = data.(timeField);

% Filter out the time field to get all plottable data fields
plotFields = fields(~strcmp(fields, timeField));
numPlots = numel(plotFields);

% 4. Plot all signals in a tiled layout
figure('Name', filePath, 'NumberTitle', 'off');
t = tiledlayout(numPlots, 1, 'TileSpacing', 'compact');

for i = 1:numPlots
    fName = plotFields{i};
    y = data.(fName);

    % Only plot numeric vectors/matrices matching the length of time
    if isnumeric(y) && (length(y) == length(time))
        nexttile;
        plot(time, y, 'LineWidth', 1.2);
        ylabel(strrep(fName, '_', '\_')); % Escape underscores for clean labels
        grid on;
    end
end

xlabel('Time [s]');
linkaxes(findall(gcf, 'type', 'axes'), 'x');