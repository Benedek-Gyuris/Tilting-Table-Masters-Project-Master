function [Tau, Tau_PID, Tau_canc, desired_angle, prev_e, prev_int] = ...
    BenceAllController(time, statevec, Params_GFB, TaskParams, pos,vel,prev_e, prev_int)
   
    % Unpack state variables
    angle = statevec(1); % X-axis angle

    Angular_vel = statevec(2); % X-axis angular velocity
    
    % System parameters
    b_friction = TaskParams.b_friction;
    m = TaskParams.m;
    g = 9.81; % Gravitational acceleration

    % Interpolate position values from TaskParams.x_pos
    curr_pos = interp1(TaskParams.Ts * (0:length(pos)-1), pos, time, 'linear', 'extrap');

    % Interpolate velocity values from TaskParams.x_velocity
    curr_velocity = interp1(TaskParams.Ts * (0:length(vel)-1), vel, time, 'linear', 'extrap');

    % Compute velocity of position data (numerical derivative)
    d_x = curr_pos;

    % Map velocity to angles (0 to 40 degrees)
    max_velocity = max(TaskParams.x_velocity); % Define the maximum velocity
    mapped_angle = max(0, min(curr_velocity / max_velocity, 1)) * 40; % Angle in degrees

    % Map velocity directly to desired angle range (0 to max_angle)
    max_angle = deg2rad(40); % Maximum angle in radians
    
    % Constrain and scale velocity
    curr_velocity = max(-max_velocity, min(curr_velocity, max_velocity));
    desired_angle = (curr_velocity / max_velocity) * max_angle;
    desired_angle = max(-max_angle, min(desired_angle, max_angle)); % Clamp to ±max_angle
    
    % Feedforward control: cancel out dynamics
    Tau_canc = 0; % m * g * d_x * cos(angle) + b_friction * Angular_vel;
    
    % Avoid divide by zero
    Tau_add = 0;

    % Feedback control to stabilize the table at the desired angle
    % Md = Params_GFB.Md; % Mass
    % Bd = Params_GFB.Bd; % Damping
    % K = Params_GFB.K; % Stiffness
    Kp = Params_GFB.P; % Mass
    Ki = Params_GFB.I; % Damping
    Kd = Params_GFB.D; % Stiffness

    desired_angle = deg2rad(20);
    e = desired_angle-angle; % Error based on desired angle
    e_int = prev_int + e*TaskParams.Ts;
    e_dot = (e - prev_e) / TaskParams.Ts;
    Tau_PID = Kp * e +Ki * e_int +  Kd * e_dot ; % Feedback control torque
    
    %Tau_IMP = Md * e_dot + Bd * e + Kd * e_int;
    
    % Display the updated error
    % disp(TaskParams.PreviousError);

    % Combine components
    Tau = Tau_add + Tau_canc*0 + Tau_PID;

    %Apply torque limit
    TorqueLimit = 200000; % Torque limit in Nm (150 mNm)

    if abs(Tau) > TorqueLimit
        Tau = sign(Tau) * TorqueLimit; % Limit the torque to 150 mNm
    end

    
  
    disp(['error: ', num2str(e), ' desired angle: ', num2str(desired_angle), ' x1: ', num2str(angle)]);


    % Update previous error for integration
    prev_e = e;
    prev_int = e_int;

    
end
