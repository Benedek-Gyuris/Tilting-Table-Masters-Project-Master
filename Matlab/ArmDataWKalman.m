function [x_pos, x_velocity_est, y_pos, y_velocity_est] = ArmDataWKalman(dataFile)
    % Load the .mat file
    data = load(dataFile); % Load the specified .mat file
    mouse_data = data.mouse_data; % Extract the mouse_data matrix

    % Extract time, x_pos, and y_pos
    time = mouse_data(:, 1);  % Time
    x_pos_raw = mouse_data(:, 2); % Raw X Position (0-2000)
    y_pos_raw = mouse_data(:, 3); % Raw Y Position (0-2000)

    % Normalize position data to have 0 at the center of the platform
    x_pos = (x_pos_raw - 1000) / 1000; % Shift to range [-1, 1] with center at 0
    y_pos = (y_pos_raw - 1000) / 1000; % Shift to range [-1, 1] with center at 0

    % Scale positions to match the physical platform size (if needed)
    platform_radius = 0.4; % Example: platform radius in meters
    x_pos = x_pos * platform_radius; % Scale to meters
    y_pos = y_pos * platform_radius; % Scale to meters

    % Calculate sampling time
    dt = mean(diff(time)); % Sampling time (assumes uniform sampling)

    % Parameters for Kalman Filter
    A = [1 dt; 0 1];       % State transition matrix
    H = [1 0];             % Measurement matrix
    Q = [4 0; 0 75];       % Process noise covariance
    R = 10;                % Measurement noise covariance
    P = eye(2);            % Initial estimation error covariance

    % Initialize Kalman filter states
    x_est = [x_pos(1); 0]; % Initial state estimate for x-axis [position; velocity]
    y_est = [y_pos(1); 0]; % Initial state estimate for y-axis [position; velocity]

    % Storage for estimated states
    x_estimates = zeros(length(x_pos), 2);
    y_estimates = zeros(length(y_pos), 2);

    % Kalman Filter Loop for X and Y Positions
    for k = 1:length(x_pos)
        % Kalman Filter for X
        % Prediction step
        x_pred = A * x_est;
        P_pred = A * P * A' + Q;

        % Measurement update
        K = P_pred * H' / (H * P_pred * H' + R); % Kalman gain
        z = x_pos(k);                            % Current measurement
        x_est = x_pred + K * (z - H * x_pred);   % Updated state estimate
        P = (eye(2) - K * H) * P_pred;           % Updated covariance

        % Store the result
        x_estimates(k, :) = x_est';

        % Kalman Filter for Y
        % Prediction step
        y_pred = A * y_est;
        P_pred = A * P * A' + Q;

        % Measurement update
        K = P_pred * H' / (H * P_pred * H' + R); % Kalman gain
        z = y_pos(k);                            % Current measurement
        y_est = y_pred + K * (z - H * y_pred);   % Updated state estimate
        P = (eye(2) - K * H) * P_pred;           % Updated covariance

        % Store the result
        y_estimates(k, :) = y_est';
    end

    % Extract estimated velocities
    x_velocity_est = x_estimates(:, 2); % Estimated velocity for x-axis
    y_velocity_est = y_estimates(:, 2); % Estimated velocity for y-axis
end
