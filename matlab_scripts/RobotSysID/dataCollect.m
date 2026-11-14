clear
clc

% Create and connect to device
stm_device = serialport('COM3', 115200, 'Timeout', 0.5);
configureTerminator(stm_device, "LF");
cleanupObj = onCleanup(@() clear('stm_device'));   % closes the port even if the script errors

% Reset STM and catch any print return
writeline(stm_device, "reset");
rxStr = readline(stm_device);
flush(stm_device);

% Send the command ONCE
command_str = "motorSineLog right";
writeline(stm_device, command_str);

% Preallocate
N = 4000;   % 20 seconds @ 0.005 sec sampling                                % number of samples to collect
motor.time     = zeros(N,1);
motor.voltage  = zeros(N,1);
motor.current  = zeros(N,1);
motor.velocity = zeros(N,1);
motor.torque   = zeros(N,1);

% Keep reading, no more writes
k = 0;
t0 = tic;
while k < N
    rxStr = readline(stm_device);           % blocks until "\n" or timeout
    if strlength(rxStr) == 0, continue; end % timeout -> no data yet
    v = sscanf(rxStr, '%f,%f,%f,%f,%f');
    if numel(v) ~= 5, continue; end         % skip partial/garbled lines
    k = k + 1;
    motor.time(k)     = v(1);
    motor.voltage(k)  = v(2);
    motor.current(k)  = v(3);
    motor.velocity(k) = v(4);
    motor.torque(k)   = v(5);
end

writeline(stm_device, "stop");

% Trim if you exit early
fn = fieldnames(motor);
for i = 1:numel(fn), motor.(fn{i}) = motor.(fn{i})(1:k); end

% Save to .mat file with a timestamped name
fname = sprintf('Right_FreewheelSpin_3s_%s.mat', datestr(now, 'yyyymmdd_HHMMSS'));
save(fname, 'motor');
fprintf('Saved %d samples to %s\n', k, fname);