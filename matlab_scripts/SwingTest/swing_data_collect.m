clear
clc

% Create and connect to device
stm_device = serialport('COM3', 115200, 'Timeout', 0.5);
%configureTerminator(stm_device, "CR/LF");
configureTerminator(stm_device, "LF");
cleanupObj = onCleanup(@() clear('stm_device'));   % closes the port even if the script errors

% Reset STM and catch any print return
writeline(stm_device, "reset");
rxStr = readline(stm_device);
flush(stm_device);

% Send the command ONCE
command_str = "swing_test_log";
writeline(stm_device, command_str);

% Set N as a generous upper-bound buffer (e.g., 3000)
N = 3000;
swing.time          = zeros(N, 1);
swing.encoderCount  = zeros(N, 1, 'int32');
swing.imu_angle     = zeros(N, 1);
swing.imu_ang_velo  = zeros(N, 1);

k = 0;
while k < N
    rxStr = readline(stm_device);

    % Timeout returns [] or <missing> depending on MATLAB version
    if isempty(rxStr) || ismissing(rxStr)
        if k > 0
            fprintf('Test complete. Received %d samples.\n', k);
            break;
        else
            continue;   % still waiting for the first sample
        end
    end

    disp("RAW RX: " + rxStr);

    if strlength(rxStr) == 0
        continue;
    end

    v = sscanf(rxStr, '%f,%d,%f,%f');
    if numel(v) ~= 4
        continue;
    end
    k = k + 1;
    swing.time(k)         = v(1);
    swing.encoderCount(k) = int32(v(2));
    swing.imu_angle(k)    = v(3);
    swing.imu_ang_velo(k) = v(4);
end

% Trim the preallocated arrays down to actual received count
fn = fieldnames(swing);
for i = 1:numel(fn)
    swing.(fn{i}) = swing.(fn{i})(1:k);
end

% Save results
fname = sprintf('swing_test_log_%s.mat', datestr(now, 'yyyymmdd_HHMMSS'));
save(fname, 'swing');
fprintf('Saved %d samples to %s\n', k, fname);