clear
close all

% Parameters
TaskParams.Ts = 0.005;
tvec = 0:TaskParams.Ts:5;  % Simulate for 15 seconds

TaskParams.TorqueX = 0;     % Torque for X-axis beam
TaskParms.TorqueZ = 0;

TaskParams.b_friction = .03159*5; % Friction coefficient (entered actual values)
TaskParams.g = 9.81;
TaskParams.m = 4;          % Mass of the puck/arm
TaskParams.L = 0.4;         % Length of platform
TaskParams.R = 1;           % Mean radius of the platform
TaskParams.B =   2 * pi * (7);      % Amplitude factor for angular frequency
TaskParams.w = TaskParams.B * abs(sin(0.1 * tvec)); % Ensure angular frequency is bounded

[x_pos, x_velocity_est, z_pos, z_velocity_est] = ArmDataWKalman('ArmMovementData.mat');

% Define sinusoidal position trajectory for x_pos
TaskParams.x_pos = TaskParams.R * sin(TaskParams.w .* tvec);
TaskParams.x_velocity = TaskParams.R * TaskParams.w .* cos(TaskParams.w .* tvec);

% Load the hand movement data
% data = load('ArmMovementData.mat'); % Replace with your .mat file name
% time = data.mouse_data(:, 1); % Extract the time column
%
% % % Replace TaskParams.x_pos and TaskParams.x_velocity with Kalman-filtered data
% TaskParams.x_pos = x_pos;
% TaskParams.x_velocity = x_velocity_est;

% % Update TaskParams.Ts and tvec to match the data
% TaskParams.Ts = mean(diff(time)); % Calculate sampling time from the data
% tvec = time'; % Use the time data directly from ArmMovementData

TaskParams.z_pos = z_pos; % If TaskParams.y_pos exists
TaskParams.z_velocity = z_velocity_est; % If y-axis velocity is used

TaskParams.J = 6.38e-3; % TaskParams.m * TaskParams.L / 3; % Moment of inertia (entered actual values)
TaskParams.delT = TaskParams.Ts;               % Time step

% Initial conditions
x1_init = 0;    % Initial X-axis angle (rad)
x2_init = 0;   % Initial X-axis angular velocity (rad/s)
y0x = [x1_init, x2_init];

% Initial conditions
z1_init = 0;    % Initial X-axis angle (rad)
z2_init = 0;   % Initial X-axis angular velocity (rad/s)
y0z = [z1_init, z2_init];

TaskParams.PreviousErrorX = 0;
TaskParams.PreviousErrorZ = 0;

dxprev = TaskParams.x_pos(1); % Initialize previous distance value
dzprev = TaskParams.z_pos(1);

Params_GFB.P = .75;% 000;% 40;   % Derivative gain
Params_GFB.I = 1*0; % 1*0; % Damping
Params_GFB.D = 0.1; % .1; % Stiffness THESE WORK FOR hand input

% This was a wrong aveneue of thought I think
% Params_GFB.K = 5;   % Derivative gain
% Params_GFB.Bd = 0.001; % Damping
% Params_GFB.Md = 1; % Stiffness

prev_intX= 0;
prev_intZ= 0;

% Preallocate variables for results
numSteps = length(tvec);
yyX = zeros(numSteps, length(y0x));
yyZ = zeros(numSteps, length(y0x));
tt = zeros(1, numSteps);

% Store X torque
TorqueX = zeros(1, numSteps); % Preallocate for storing torque
Torque_fbX = zeros(1, numSteps);
Torque_addX = zeros(1, numSteps);
Torque_cancX = zeros(1, numSteps);

TorqueZ = zeros(1, numSteps);
Torque_fbZ = zeros(1, numSteps);
Torque_addZ = zeros(1, numSteps);
Torque_cancZ = zeros(1, numSteps);

CerrorX = zeros(1, numSteps);
CerrorZ = zeros(1, numSteps);

desired_angle_savex = zeros(1, numSteps);
desired_angle_savez = zeros(1, numSteps);



% Main simulation loop
ii = 0;
for tdel = 0:TaskParams.Ts:tvec(end)
    ii = ii + 1;
    TaskParams.Tspan_delta = [0 TaskParams.Ts / 2 TaskParams.Ts] + tdel;

    % Calculate velocity of position data (numerical derivative)
    if ii > 1
        x_pos_prev = TaskParams.x_pos(ii-1);
    else
        x_pos_prev = TaskParams.x_pos(ii); % Handle initial condition
    end
    x_pos_curr = TaskParams.x_pos(ii);
    % Calculate x_pos_velocity during simulation
    % x_pos_velocity = diff(TaskParams.x_pos) / TaskParams.Ts;
    % x_pos_velocity = [x_pos_velocity, x_pos_velocity(end)]; % Match length of tvec
    
    disp([' pre control x1: ', num2str(y0x)]);
    [TaskParams.TorqueX, Tau_PIDx, Tau_cancX,desired_angleX, CerrorX(ii), prev_intX] = ...
        BenceAllController(tdel, y0x, Params_GFB, TaskParams, TaskParams.x_pos,TaskParams.x_velocity, TaskParams.PreviousErrorX, prev_intX);
    
    [TaskParams.TorqueZ, Tau_PIDz, Tau_cancZ,desired_angleZ, CerrorZ(ii), prev_intZ] = ...
        BenceAllController(tdel, y0z, Params_GFB, TaskParams, TaskParams.z_pos,TaskParams.z_velocity, TaskParams.PreviousErrorZ, prev_intZ);

    % Save the torque value
    TorqueX(ii) = TaskParams.TorqueX;
    Torque_fbX(ii) = Tau_PIDx;
    Torque_cancX(ii) = Tau_cancX;
    TaskParams.PreviousErrorX = CerrorX(ii);
    
    Torque_fbZ(ii) = Tau_PIDz;
    Torque_cancZ(ii) = Tau_cancZ;
    TaskParams.PreviousErrorZ = CerrorZ(ii);
    
    TorqueZ(ii) = TaskParams.TorqueZ;
    TaskParams.PreviousErrorZ = CerrorZ(ii);

    desired_angle_savex(ii) = desired_angleX;
    desired_angle_savez(ii) = desired_angleZ;

    % Solve ODE for the current time step
    [tttx, yyyx] = ode45(@(t, y) ode_slidingPuck(t, y, TaskParams, dxprev), [tdel tdel + TaskParams.delT], y0x);

    % Update dxprev for the next iteration
    dxprev = TaskParams.x_pos(ii);
    
    % Store results
    y0x = yyyx(end, :); % Update the initial condition for the next step
    tt(ii) = tttx(1);
    yyX(ii, :) = yyyx(end, :);
    disp(['TorquePID: ', num2str(Torque_fbX(ii)), ' d_x: ', num2str(TaskParams.x_pos(ii)), ' x1: ', num2str(y0x), 'simTime: ', num2str(tdel)]);

end


% Pass Torque and x_pos_velocity to the function
BenceShowResults(tt, yyX, tvec, TaskParams, TorqueX, Torque_fbX, Torque_addX, Torque_cancX, CerrorX, desired_angle_savex)