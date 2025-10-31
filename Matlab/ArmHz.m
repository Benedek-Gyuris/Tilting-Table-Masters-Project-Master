% Load the .mat file
data = load(data); % Replace with the name of your .mat file

% Extract time, x pos, and y pos from the file
time =  data.mouse_data(:,1);  % First column is time
x_pos = data.mouse_data(:,2); % Second column is x position
y_pos = data.mouse_data(:,3); % Third column is y position
% Calculate time intervals (sampling time)
dt = diff(time); % Differences between consecutive time points

% Calculate x and y velocity
x_velocity = diff(x_pos) ./ dt; % Velocity in x direction
y_velocity = diff(y_pos) ./ dt; % Velocity in y direction

% Append NaN for the last velocity to match the array size with the original data
x_velocity = [x_velocity; NaN];
y_velocity = [y_velocity; NaN];

% Parameters
dt = mean(diff(time)); % Sampling time
A = [1 dt; 0 1];       % State transition matrix
H = [1 0];             % Measurement matrix
Q = [4 0; 0 75];        % Process noise covariance
R = 10;                % Measurement noise covariance
P = eye(2);            % Initial estimation error covariance
x_est = [x_pos(1); 0]; % Initial state estimate [position; velocity]
y_est = [y_pos(1); 0];

% Storage for estimated states
x_estimates = zeros(length(x_pos), 2);
y_estimates

% Kalman Filter Loop
for k = 1:length(x_pos)
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
end

% Extract estimated velocity
velocity_estimate = x_estimates(:, 2);

% Compute the sampling frequency
dt = mean(diff(time));     % Time step (assumes uniform sampling)
Fs = 1 / dt;               % Sampling frequency

% Perform FFT for X position
L = length(x_pos);         % Length of the signal
Yx = fft(x_pos);           % FFT of x position
P2x = abs(Yx / L);         % Two-sided spectrum
P1x = P2x(1:L/2+1);        % Single-sided spectrum
P1x(2:end-1) = 2*P1x(2:end-1);

% Perform FFT for Y position
Yy = fft(y_pos);           % FFT of y position
P2y = abs(Yy / L);         % Two-sided spectrum
P1y = P2y(1:L/2+1);        % Single-sided spectrum
P1y(2:end-1) = 2*P1y(2:end-1);

% Frequency axis
f = Fs * (0:(L/2)) / L;
%
% % Plot the frequency spectrum for x position
% figure;
% plot(x_pos, y_pos);
% xlabel('X Position');
% ylabel('Y Position');
% title('Hand position on table');
% grid on;
%
% % Make the plot square
% axis square;
%
% figure;
% % Plot the frequency spectrum for y position
% subplot(2, 1, 1);
% plot(f, P1x);
% xlabel('Frequency (Hz)');
% ylabel('|P1(f)|');
% title('Frequency Spectrum of Y Position');
% grid on;
%
% % Plot the frequency spectrum for y position
% subplot(2, 1, 2);
% plot(f, P1y);
% xlabel('Frequency (Hz)');
% ylabel('|P1(f)|');
% title('Frequency Spectrum of Y Position');
% grid on;
%
% % Display results
% disp('X Velocity:');
% disp(x_velocity);
% disp('Y Velocity:');
% disp(y_velocity);
%
% % Plot the results
% figure;
% subplot(2, 1, 1);
% plot(time, x_velocity, 'r');
% title('X Velocity');
% xlabel('Time (s)');
% ylabel('Velocity (units/s)');
%
% subplot(2, 1, 2);
% plot(time, y_velocity, 'b');
% title('Y Velocity');
% xlabel('Time (s)');
% ylabel('Velocity (units/s)');
% % Identify the dominant frequency ranges
% threshold_x = 0.01 * max(P1x); % Threshold at 1% of the max amplitude (X position)
% dominant_freqs_x = f(P1x > threshold_x);
%
% % Plot estimated velocity
% figure;
% plot(time, velocity_estimate, 'g', 'DisplayName', 'Estimated Velocity');
% hold on;
% plot(time, gradient(x_pos)./dt, 'r--', 'DisplayName', 'Computed Velocity');
% legend();
% title('Velocity Estimation with Kalman Filter');
% xlabel('Time (s)');
% ylabel('Velocity (units/s)');
% grid on;

%threshold_y = 0.01 * max(P1y); % Threshold at 1% of the max amplitude (Y position)
%dominant_freqs_y = f(P1y > threshold_y);

% Display the ranges
%disp(['Dominant frequency range (X Position): ', num2str(min(dominant_freqs_x)), ' Hz to ', num2str(max(dominant_freqs_x)), ' Hz']);
%disp(['Dominant frequency range (Y Position): ', num2str(min(dominant_freqs_y)), ' Hz to ', num2str(max(dominant_freqs_y)), ' Hz