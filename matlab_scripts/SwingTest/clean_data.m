% 1. Load the swing object
S = load('untrimmed_swing_test_log_20261110_172933.mat', 'swing');
swing = S.swing;

% 2. Define starting time threshold and find valid indices
t_start = 2.16; % <-- Set your desired start time (in seconds)
idx = swing.time >= t_start;

% 3. Trim all time-aligned fields
swing.time           = swing.time(idx);
swing.encoderCount   = swing.encoderCount(idx);
swing.imu_ang_velo   = swing.imu_ang_velo(idx);

% 4. Trim and make all angles negative: -abs(x) guarantees every value is <= 0
swing.imu_angle      = -abs(swing.imu_angle(idx));