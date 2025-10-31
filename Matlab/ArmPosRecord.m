clear; clc;

% Set up parameters
capture_duration = 10; % Duration to capture data (seconds)
sampling_rate = 100; % Sampling rate (Hz)
pause_time = 1 / sampling_rate;
num_samples = capture_duration * sampling_rate;

% Initialize data storage
mouse_data = zeros(num_samples, 3); % Columns: [Time, X, Y]
t0 = tic;

% Create a Robot object to capture mouse movement
robot = java.awt.Robot;

% Capture mouse position data
for i = 1:num_samples
    % Get mouse position
    mouse_point = java.awt.MouseInfo.getPointerInfo().getLocation();
    mouse_data(i, :) = [toc(t0), mouse_point.getX(), mouse_point.getY()];
    pause(pause_time);
end

% Save data to a file (optional)
save('ArmMovementData.mat', 'mouse_data');

% Plot captured data
figure;
plot(mouse_data(:, 2), mouse_data(:, 3), 'b.-');
xlabel('X Position (pixels)');
ylabel('Y Position (pixels)');
title('Mouse Movement');
grid on;
