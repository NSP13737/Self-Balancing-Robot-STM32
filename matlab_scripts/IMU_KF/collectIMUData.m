clear
clc
% remove any persistant serial connections
if ~isempty(instrfind('Status', 'open'))
    fclose(instrfind);
end

%%
% Create connections to STM32 with 115200 baud rate and 0.5 second timeout
stm_device = serialport('COM3', 115200, 'Timeout', 0.5);
% Call the potentiometer data-logging code
writeline(stm_device, "imuLogData");
% Define expected number of samples
N = 200*5; %(Hz * sec)
% Initialise data structure for data
imuData.time = zeros(N,1);
imuData.angle = zeros(N,1);
imuData.ang_velo = zeros(N,1);
imuData.voltage = zeros(N,1);
% Collect the expected number of samples
for i=1:N
    % Read in data from serial connection
    rxStr = readline(stm_device);
    if (rxStr ~= "")
        % Separate string into parts
        separatedString = split(rxStr,',');
        % Convert data to numbers and store in data structure
        imuData.time(i) = str2double(separatedString{1});
        imuData.angle(i) = str2double(separatedString{2});
        imuData.ang_velo(i) = str2double(separatedString{3});
        imuData.voltage(i) = str2double(separatedString{4});
    else
        imuData.time(i) = 0;
        imuData.angle(i) = 0;
        imuData.ang_velo(i) = 0;
        imuData.voltage(i) = 0;
    end
end


% Plot results
% Figure 1: Voltage
figure(1);
plot(imuData.time, imuData.voltage, '+');
grid on;
xlabel('Time (s)');
ylabel('Voltage (V)');
title('Left Wheel Encoder Count vs Time');

% Figure 2: Angle
figure(2);
plot(imuData.time, imuData.angle, '+');
grid on;
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Angle vs Time');

% Figure 3: Angular Velocity
figure(3);
plot(imuData.time, imuData.ang_velo, '+');
grid on;
xlabel('Time (s)');
ylabel('Angular Velocity (deg/s)');
title('Angular Velocity vs Time');


% Close serial connection and clean up
clear stm_device