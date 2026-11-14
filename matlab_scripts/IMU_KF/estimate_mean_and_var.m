cla reset
clear
clc
%% Load data
load('swing_rig_stationary');
% Unpack data
time = imuData.time;
accAngle = imuData.angle;
gyroVelocity = imuData.ang_velo;
voltage = imuData.voltage;
%wheelAngle = (voltage-2.125)*(240/3.3)*(pi/180); % IF USING WHEEL ENCODER
wheelAngle = mod(voltage,4096) * (2.0 * pi / 4096.0)% IF USING SWING RIG ENCODER (AND KNOWN UPRIGHT IS 0 COUNTS)


N = length(time);
%% Compute gyroscope velocity sample mean and variance
% TODO: Compute sample mean
gyroVelocityMean = mean(gyroVelocity);
% TODO: Compute gyroscope velocity sample variance. Use Bessel's correction
gyroVelocityVar = sum( (gyroVelocity - gyroVelocityMean).^2) / (length(gyroVelocity) - 1)


%% Load data
load('swing_rig_osc');
% Unpack data
time = imuData.time;
accAngle = imuData.angle;
gyroVelocity = imuData.ang_velo;
encoderCounts = imuData.voltage;
% cpr = 1320.0;
% wheelAngle = (encoderCounts - encoderCounts(1)) * (2 * pi / cpr); % USE
% WITH WHEEL AS ENCODER
wheelAngle = mod(voltage,4096) * (2.0 * pi / 4096.0)% IF USING SWING RIG ENCODER (AND KNOWN UPRIGHT IS 0 COUNTS)

N = length(time);

% TODO: Correct angle offset
wheel_angle_init = mean(wheelAngle(1:200));
acc_angle_init = mean(accAngle(1:200));
wheelAngle = wheelAngle - (wheel_angle_init - acc_angle_init);
plot(time,accAngle,time,wheelAngle)

% TODO: Compute the angle measurement 'noise'
angle_noise = accAngle - wheelAngle; 

% TODO: Compute accellerometer angle sample mean
accAngleMean = mean(angle_noise)

% TODO: Compute accellerometer angle sample covariance. Use Bessel's correction
accAngleVar = sum( (angle_noise - accAngleMean).^2) / (length(angle_noise) - 1);

% sensor covariance matrix
R = [accAngleVar 0; 0 gyroVelocityVar]