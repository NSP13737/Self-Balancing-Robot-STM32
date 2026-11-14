clear
clc
clf
close all
clear classDCMotor
%% Flag: set to true/false to include/exclude the linearized simulation
runLinearSim = false;

PIL_bool = 1;
read_integrator_z_val = 0;

if PIL_bool
    %remove any persistant serial connections
    if ~isempty(instrfind('Status','open'))
        fclose(instrfind);
    end
    %Create and connect to device
    stm_device=serialport('COM3',115200,'Timeout',0.5);
    %Reset STM
    writeline(stm_device,"reset");
    %Catch any print return
    rxStr=readline(stm_device);
end



%% Define motor object and define params
balancingRobot = classBalancingRobotFlow;
balancingRobot.mc = 0.833;
balancingRobot.Jc = 0.0056385; %from swing test
balancingRobot.mw = 0.0473*2; %wheel mass
balancingRobot.Jw = 0.339; %avged between motors from free spin test
balancingRobot.l = 0.1 - (0.065/2);
balancingRobot.r = 0.065/2;
balancingRobot.g = 9.8;
balancingRobot.b_phi = 0.3;
balancingRobot.b_theta = 0.0035897;

%% Define linearized robot object and assign the same parameters (Task 1.3 f)
balancingRobotLin = classBalancingRobotFlowLinear;
balancingRobotLin.mc = balancingRobot.mc;
balancingRobotLin.Jc = balancingRobot.Jc;
balancingRobotLin.mw = balancingRobot.mw;
balancingRobotLin.Jw = balancingRobot.Jw;
balancingRobotLin.l = balancingRobot.l;
balancingRobotLin.r = balancingRobot.r;
balancingRobotLin.g = balancingRobot.g;
balancingRobotLin.b_phi = balancingRobot.b_phi;
balancingRobotLin.b_theta = balancingRobot.b_theta;

%% Create DC motor obj
motor = classDCMotor;
% motor parameters
motor.Ra = 7.39325; % avged between both motors
motor.La = 0.1;
motor.N = 30; % obvious
motor.Td = 0.005;
motor.Ko = 0.31185; % avged between both motors
motor.Ki = 0.0059; % avged between both motors
motor.Vmax = 12;
motor.Vmin =-12;
motor.Jm = 0.001;
motor.b = 0.001;
motor.Kp = 0.8

%% Cart Pole control
controllerBalancingRobot = classDiscreteLQRcontrol;
T = 1/200;

%Regulation output
C_r = [1 0 0];
D_r = 0;

Q_int = diag([2 1 1 1])
Q = diag([10 1 1 1]);
R = 0.6
N_int = zeros(4,1);
N = zeros(4,1);

y_ref = 0.0;
controllerBalancingRobot.controlInit_reference_integrator(balancingRobotLin.A(), balancingRobotLin.B(), C_r, D_r, Q_int, R, N_int, T, y_ref);

%% Solve ODE
t_sim = [0:T:10];
x0 = [0; 3*pi/180; 0]; % [phi; theta; p_theta]
wheel_vel = 0;

if (runLinearSim)
    alpha = 0; 
else
    alpha = 5*pi/180;
end
alpha = 0;

%torque
%T = (balancingRobot.mc+balancingRobot.mw)*balancingRobot.g*balancingRobot.r*sin(alpha);

res.t = t_sim(1);
res.x = x0.';
res.lambda = 0; 
%res.arm_current = 0;
%res.t_m = 0;
res.desired_vel = 0;
res.output_vel = 0;
%res.desired_torque = 0;
res.z_states = 0;
res.debug_wheelVel = 0;
res.voltage = 0;

%% Iterative ODE solver

options = odeset('relTol', 1e-6);
for i = 1:length(t_sim)-1
    % Update estimate of wheel velo based on previous wheel pos, and newly
    % solved wheel pos
    wheel_vel = (res.x(end,1) - x0(1)) / T; % scale by the sampling period

    %update states of most recent solved vals
    x0 = res.x(end,:);
    
    lambda0 = res.lambda(end);
    
    t = [t_sim(i),t_sim(i+1)]; % define time to iterate one fwd

    %% NEW CODE FOR VELOCTY WITH PI CONTROL ALLOCATION
    %% Get control input from LQR
    if ~PIL_bool
        desired_vel = controllerBalancingRobot.control(x0.');
        z_state = controllerBalancingRobot.z;
        %% Take ideal control and put through control allocation to Voltage
        V_in = motor.flowControlAllocation(desired_vel, wheel_vel, T);
    else
        %Get control action from STM32
        if ~read_integrator_z_val
            controlCommand = "getAllocatedControl ";
        else
            controlCommand = "getControlAllocationwIntegrator ";
        end
        command_str = strcat(controlCommand,num2str(x0(1))," ",num2str(x0(2)),...
            " ",num2str(x0(3)));
        writeline(stm_device, command_str);
        
        % Read return control action
        rxStr = readline(stm_device);
        %fprintf("Step %d | rxStr: %s\n", i, rxStr);
        if ~read_integrator_z_val
            V_in = str2double(rxStr);
        else
            separatedString = split(rxStr,',');
            V_in = str2double(separatedString{1});
            z_state = str2double(separatedString{2});

        end
        
    end
    
    %% Simulate motor dynamic and robot based on voltage input
    ode_dc_motor = @(t,lambda) [motor.state_derivative(V_in, lambda, wheel_vel)]; %last element is phidot which is omega
    [t, lambda] = ode45(ode_dc_motor, t, lambda0);

    % derive output vel from new solved flux
    output_vel = motor.output_vel(V_in, lambda(end));


    %ode for dynamics
    ode_robot_dynamics = @(t,x) [balancingRobot.state_derivative(x, output_vel, alpha)];
    [t, x] = ode45(ode_robot_dynamics, t, x0, options);
    

    % append next time step solved states to our state storage
    res.t = [res.t; t(end)];
    res.x = [res.x; x(end, :)];
    res.lambda = [res.lambda; lambda(end)];
    %res.arm_current = [res.arm_current; motor.armature_current(lambda(end))];
    %res.t_m = [res.t_m; t_m];
    %res.desired_vel = [res.desired_vel; desired_vel];
    res.output_vel = [res.output_vel; output_vel];
    %res.z_states = [res.z_states; z_state];
    res.debug_wheelVel = [res.debug_wheelVel; wheel_vel];
    res.voltage = [res.voltage; V_in];

end

if PIL_bool
    clear stm_device
end

solved_phi_col = res.x(:,1);
solved_theta_col = res.x(:,2);
solved_dp_phi_col = res.x(:,3);


%% First plot
fig1 = figure (1);
subplot(5,1,1)
plot(res.t, solved_phi_col)
if runLinearSim
    hold on
    plot(resLin.t, solved_phi_col_lin, '--')
    legend('\phi', '\phi (linear)')
    ylim([-3, 0])
else
    legend('\phi')
end
xlabel('Time [sec]')
grid on


subplot(5,1,2)
plot(res.t, solved_theta_col)
if runLinearSim
    hold on
    plot(resLin.t, solved_theta_col_lin, '--')
    legend('\theta', '\theta (linear)')
    ylim([0, 5])
else
    legend('\theta')
end
xlabel('Time [sec]')
grid on


subplot(5,1,3)
plot(res.t, solved_dp_phi_col)
if runLinearSim
    hold on
    plot(resLin.t, solved_dp_phi_col_lin, '--')
    legend('d\phi', 'd\phi (linear)')
    ylim([-4, 4])
else
    legend('d\phi')
end
xlabel('Time [sec]')
grid on

subplot(5,1,4)
%plot(res.t, res.t_m);
plot(res.t, res.desired_vel, res.t, res.debug_wheelVel, '--');
hold on
%plot(res.t, res.desired_torque, '--');
legend('desired velocity', 'measuredWheelVel')
xlabel('Time [sec]')
grid on

subplot(5,1,5)
plot(res.t, res.voltage);
hold on
legend('Applied Motor Voltage')
xlabel('Time [sec]')
grid on

if read_integrator_z_val
    figure(2)
    plot(res.t, res.z_states)
end

% figure(3)
% plot(res.t, res.voltage)
% ylabel("Applied Voltage")