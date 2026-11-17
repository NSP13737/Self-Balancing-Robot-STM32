clear;
clc;
close all;

% Remove any persistent serial connections
if ~isempty(instrfind('Status', 'open'))
    fclose(instrfind);
end

%% Serial Setup
% Create connection to STM32 with 115200 baud rate and 0.5 second timeout
stm_device = serialport('COM3', 115200, 'Timeout', 0.5);

% Trigger data-logging on the STM32
writeline(stm_device, "imuKFLog");

% Define expected number of samples (200 Hz * 5 sec)
N = 200 * 5;

% Initialise data structure for all 5 incoming fields
imuData.time        = zeros(N, 1);
imuData.angle       = zeros(N, 1);
imuData.ang_velo    = zeros(N, 1);
imuData.kf_angle    = zeros(N, 1);
imuData.kf_ang_velo = zeros(N, 1);

% Collect the expected number of samples
for i = 1:N
    rxStr = readline(stm_device);
    if (rxStr ~= "")
        separatedString = split(strtrim(rxStr), ',');

        % Ensure all 5 values arrived before parsing
        if numel(separatedString) >= 5
            imuData.time(i)        = str2double(separatedString{1});
            imuData.angle(i)       = str2double(separatedString{2});
            imuData.ang_velo(i)    = str2double(separatedString{3});
            imuData.kf_angle(i)    = str2double(separatedString{4});
            imuData.kf_ang_velo(i) = str2double(separatedString{5});
        end
    end
end

% Close serial connection
clear stm_device;

%% Save Data (matching your onboard_data naming convention)
if ~isfolder('onboard_data')
    mkdir('onboard_data');
end
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
saveFile = fullfile('onboard_data', ['onboard_imu_kf_' timestamp '.mat']);
save(saveFile, 'imuData');
fprintf('Data saved to %s\n', saveFile);

%% Quick Comparison Plot (Raw IMU vs. Kalman Filter)
figure('Name', 'IMU vs Kalman Filter', 'NumberTitle', 'off');
t = tiledlayout(2, 1, 'TileSpacing', 'compact');

% Tile 1: Angle comparison
nexttile;
plot(imuData.time, imuData.angle, 'LineWidth', 1.1, 'DisplayName', 'Raw IMU Angle');
hold on;
plot(imuData.time, imuData.kf_angle, 'LineWidth', 1.5, 'DisplayName', 'Kalman Angle');
ylabel('Angle [deg]');
legend('Location', 'best');
grid on;

% Tile 2: Angular velocity comparison
nexttile;
plot(imuData.time, imuData.ang_velo, 'LineWidth', 1.1, 'DisplayName', 'Raw IMU Ang Velo');
hold on;
plot(imuData.time, imuData.kf_ang_velo, 'LineWidth', 1.5, 'DisplayName', 'Kalman Ang Velo + Bias');
ylabel('Angular Velocity [deg/s]');
xlabel('Time [s]');
legend('Location', 'best');
grid on;

linkaxes(findall(gcf, 'type', 'axes'), 'x');