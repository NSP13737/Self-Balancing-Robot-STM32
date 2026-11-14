clear; clc;

%% Optimise Q on the training dataset
train = loadData('swing_rig_osc.mat');

cost = @(params) computeCost(params, train.wheelAngle, train.gyroVelocity, train.accAngle, train.N);
params0 = [1e-4; 0; 1e-4; 0; 0; 1e-4];
options = optimoptions('fminunc', 'MaxFunctionEvaluations', 10000);
params_opt = fminunc(cost, params0, options);
L_opt = [params_opt(1) 0 0; params_opt(2) params_opt(3) 0; params_opt(4) params_opt(5) params_opt(6)];
Q_opt = L_opt * L_opt.';

%% Final run of filter on the training data (figure 1)
[sensorData, sensorCoVar] = runKF(train.accAngle, train.gyroVelocity, train.N, params_opt);
plotResults(1, train, sensorData);

%% Validate Dataset 1 (figure 2)
val1 = loadData('swing_rig_osc_2.mat');
[sensorData, sensorCoVar] = runKF(val1.accAngle, val1.gyroVelocity, val1.N, params_opt);
plotResults(2, val1, sensorData);

%% Validate Dataset 2 (figure 3)
val2 = loadData('swing_rig_osc_fast');
[sensorData, sensorCoVar] = runKF(val2.accAngle, val2.gyroVelocity, val2.N, params_opt);
plotResults(3, val2, sensorData);

%% ---------------------------- Functions ----------------------------

function d = loadData(filename)
    % Load a dataset, unpack it and correct the wheel angle offset.
    load(filename);
    d.time         = imuData.time;
    d.accAngle     = imuData.angle;
    d.gyroVelocity = imuData.ang_velo;
    encoderCounts  = imuData.voltage;
    %cpr = 1320.0;
    %wheelAngle = -((encoderCounts - encoderCounts(1)) * (2 * pi / cpr)); % USE IF USING WHEEL AS ENCODER
    wheelAngle = -encoderCounts * (2.0 * pi / (1024.0*4)); % IF USING SWING RIG ENCODER (AND KNOWN UPRIGHT IS 0 COUNTS)

    d.N = length(d.time);
    % Correct angle offset
    wheel_angle_init = mean(wheelAngle(1:200));
    acc_angle_init = mean(d.accAngle(1:200));
    %d.wheelAngle = wheelAngle - (wheel_angle_init - acc_angle_init);
    d.wheelAngle = wheelAngle; %USE IF 0deg IS KNOWN 0 COUNT
end

function plotResults(figNum, d, sensorData)
    figure(figNum);
    subplot(2, 1, 1);
    plot(d.time, d.wheelAngle, '-', 'LineWidth', 1.5);
    hold on;
    plot(d.time, sensorData(:, 2), '--', 'LineWidth', 1.5);
    plot(d.time, d.accAngle, '-', 'LineWidth', 1.2);
    hold off;
    grid on;
    xlabel('Time (s)');
    ylabel('Angle (rad)');
    legend('Wheel angle', 'Kalman filter angle', 'Accelerometer angle');
    subplot(2, 1, 2)
    plot(d.time, d.gyroVelocity, '-', 'LineWidth', 1.5);
    hold on;
    plot(d.time, sensorData(:, 1), '--', 'LineWidth', 1.5);
    hold off;
    grid on;
    xlabel('Time (s)');
    ylabel('Gyro Velo (rad/s)');
    legend('Gyro velocity', 'Kalman filter velocity');
end

function [x_KF, P_KF] = runKF(angle_accel, velocity_gyro, N, params)
    % TODO: Define continuous time model
    Ac = [0 0 0; 1 0 0; 0 0 0]; % see lab7 eq 3
    Bc = zeros(3,3); % our current input does not affect
    C = [0 1 0; 1 0 1];  % see lab7 eq 3
    % TODO: Discretise model using forward Euler method of c2d function
    T = 1/200;
    Ad = [1 0 0; T 1 0; 0 0 1]; % see lab7 eq 5
    Bd = Bc;
    % Define Kalman filter initial conditions
    xm = [0; 0; 0]; % x minus: prior state estimate before accounting for current measurement
    Pm = 1*eye(3); % P minus: prior estimate error covariance
    % Measurement noise covariance
    preComputed_accAngleVar = 0.2492;
    preComputed_gyroVelocityVar = 2.5772e-06;
    R = [preComputed_accAngleVar 0; 0 preComputed_gyroVelocityVar];
    % Process noise covariance from Cholesky decomposition of Q
    L = [params(1) 0 0; params(2) params(3) 0; params(4) params(5) params(6)];
    Q = L * L.';
    % Allocate space to save results
    x_KF = zeros(N,3);
    P_KF = zeros(3,3,N);
    % Run Kalman Filter
    for i=1:N
        yi = [angle_accel(i); velocity_gyro(i)];
        % Correction step
        Kk = Pm*C.' * inv(C*Pm*C.' + R);
        xp = xm + (Kk*(yi - (C*xm)));
        Pp = Pm - (Pm*C.' * inv(C*Pm*C.' + R) * C*Pm);
        % Prediction step
        xm = Ad*xp;
        Pm = Ad*Pp*Ad.' + Q;
        % Store results
        x_KF(i,:) = xp.';
        P_KF(:,:,i) = Pm;
    end
end

function J = computeCost(params, wheelAngle, gyroVelocity, accAngle, N)
    x = runKF(accAngle, gyroVelocity, N, params);
    angle_est = x(:, 2);
    gyro_est  = x(:, 1) + x(:, 3);
    J = norm(wheelAngle - angle_est) + norm(gyroVelocity - gyro_est);
end