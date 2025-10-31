% THIS CODE TALKS TO PIDresponse.ino

% fclose(instrfind); % Close all open serial connections
% delete(instrfind); % Delete all serial objects
% clear instrfind;   % Clear the serial object references from MATLAB workspace
% clc
% Initialize serial communication
serialPort = 'COM12'; % Replace with your Arduino's serial port
baudRate = 115200;
arduino = serial(serialPort, 'BaudRate', baudRate, 'Terminator', 'LF');

% Flush serial buffer
if arduino.BytesAvailable > 0
    fread(arduino, arduino.BytesAvailable); % Clear buffer
end
fopen(arduino);
pause(3);

% Now send commands and read data
fprintf(arduino, 'R'); % Send 'R' command
pause(0.5); % Allow Arduino time to process

% Constants
ref_angle_x = 0.349066*2; % Reference angle for Motor 1 (X)
ref_angle_z = 0.349066; % Reference angle for Motor 2 (Z)

% Prepare figure for real-time plotting
figure(1);
hold on;
grid on;
xlabel('Time (s)');
ylabel('Angle (radians)');
title('Desired vs Current Angles');
xTargetPlot = plot(NaN, NaN, 'b', 'LineWidth', 1.5); % Motor 1 Desired
xCurrentPlot = plot(NaN, NaN, 'r', 'LineWidth', 1.5); % Motor 1 Current
zTargetPlot = plot(NaN, NaN, 'g', 'LineWidth', 1.5); % Motor 2 Desired
zCurrentPlot = plot(NaN, NaN, 'k', 'LineWidth', 1.5); % Motor 2 Current
legend('Motor 1 Desired', 'Motor 1 Current', 'Motor 2 Desired', 'Motor 2 Current');

% Initialize data storage
timeData = [];
xTargetData = [];
xCurrentData = [];
zTargetData = [];
zCurrentData = [];
startTime = tic;

while true
    line = fgetl(arduino); % Read a line of data
    if isempty(line)
        continue; % Skip if no data
    end

    % Parse the incoming data
    try
        tokens = split(line, ',');
        xTarget = sscanf(tokens{1}, 'X_Target:%f');
        xCurrent = sscanf(tokens{2}, 'X_Current:%f');
        zTarget = sscanf(tokens{3}, 'Z_Target:%f');
        zCurrent = sscanf(tokens{4}, 'Z_Current:%f');
        
        % Update data storage
        elapsedTime = toc(startTime);
        timeData = [timeData, elapsedTime];
        xTargetData = [xTargetData, xTarget];
        xCurrentData = [xCurrentData, xCurrent];
        zTargetData = [zTargetData, zTarget];
        zCurrentData = [zCurrentData, zCurrent];
        
        % Update plot
        set(xTargetPlot, 'XData', timeData, 'YData', xTargetData);
        set(xCurrentPlot, 'XData', timeData, 'YData', xCurrentData);
        set(zTargetPlot, 'XData', timeData, 'YData', zTargetData);
        set(zCurrentPlot, 'XData', timeData, 'YData', zCurrentData);
        drawnow;

    catch
        % Skip invalid lines
        continue;
    end
    
    % Exit condition (you can define a stop mechanism, like a key press)
    if elapsedTime > 2 % Example: Stop after 15 seconds
        break;
    end
end

% Close the serial connection
fclose(arduino);
delete(arduino);
disp('Data collection stopped.');

% Control Metrics Analysis
% Convert data to degrees
AngleX = rad2deg(xCurrentData);
AngleZ = rad2deg(zCurrentData);
desired_angle_x = xTargetData;
desired_angle_z = zTargetData;

% Compute steady-state values
if length(AngleX) >= 100
    steady_state_x = mean(AngleX(end-100:end)); % Mean of the last 100 values
else
    steady_state_x = mean(AngleX); % Use all available data if fewer than 100 points
end

if length(AngleZ) >= 100
    steady_state_z = mean(AngleZ(end-100:end)); % Mean of the last 100 values
else
    steady_state_z = mean(AngleZ); % Use all available data if fewer than 100 points
end

% Compute steady-state errors
e_ss_x = abs(desired_angle_x - steady_state_x);
e_ss_z = abs(desired_angle_z - steady_state_z);

% Compute settling times
tol_x = 0.02 * desired_angle_x; % 2% tolerance for X
tol_z = 0.02 * desired_angle_z; % 2% tolerance for Z

within_tolerance_x = abs(AngleX - desired_angle_x) <= tol_x;
within_tolerance_z = abs(AngleZ - desired_angle_z) <= tol_z;

T_s_x = NaN; % Initialize as NaN
T_s_z = NaN;

for i = 1:length(within_tolerance_x)
    if all(within_tolerance_x(i:end))
        T_s_x = timeData(i); % Settling time for X
        break;
    end
end

for i = 1:length(within_tolerance_z)
    if all(within_tolerance_z(i:end))
        T_s_z = timeData(i); % Settling time for Z
        break;
    end
end

% Compute rise times
rise_start_x = 0.1 * desired_angle_x; % 10% of steady-state value for X
rise_end_x = 0.9 * desired_angle_x; % 90% of steady-state value for X
rise_start_z = 0.1 * desired_angle_z; % 10% of steady-state value for Z
rise_end_z = 0.9 * desired_angle_z; % 90% of steady-state value for Z

rise_idx_start_x = find(AngleX >= rise_start_x, 1, 'first');
rise_idx_end_x = find(AngleX >= rise_end_x, 1, 'first');
rise_idx_start_z = find(AngleZ >= rise_start_z, 1, 'first');
rise_idx_end_z = find(AngleZ >= rise_end_z, 1, 'first');

if ~isempty(rise_idx_start_x) && ~isempty(rise_idx_end_x)
    T_r_x = timeData(rise_idx_end_x) - timeData(rise_idx_start_x);
else
    T_r_x = NaN; % If rise conditions are not met
end

if ~isempty(rise_idx_start_z) && ~isempty(rise_idx_end_z)
    T_r_z = timeData(rise_idx_end_z) - timeData(rise_idx_start_z);
else
    T_r_z = NaN; % If rise conditions are not met
end

% Final Plot with Annotations
figure(2);
set(gcf, 'Position', [100, 100, 1200, 600]); % Adjust figure size
hold on;
plot(timeData, AngleX, 'b-', 'LineWidth', 2); % X-axis angle
plot(timeData, ones(size(timeData)) .* desired_angle_x, 'r--', 'LineWidth', 2); % X desired angle
plot(timeData, AngleZ, 'g-', 'LineWidth', 2); % Z-axis angle
plot(timeData, ones(size(timeData)) .* desired_angle_z, 'k--', 'LineWidth', 2); % Z desired angle

% Annotations for settling times
if ~isnan(T_s_x)
    % Mark the settling time point for X
    scatter(T_s_x, steady_state_x, 100, 'filled', 'g'); % Green marker
    % Add text annotation for settling time
    text(T_s_x + 0.2, steady_state_x, sprintf('T_s_X: %.2f s', T_s_x), 'FontSize', 12, 'Color', 'g');
end

if ~isnan(T_s_z)
    % Mark the settling time point for Z
    scatter(T_s_z, steady_state_z, 100, 'filled', 'm'); % Magenta marker
    % Add text annotation for settling time
    text(T_s_z + 0.2, steady_state_z, sprintf('T_s_Z: %.2f s', T_s_z), 'FontSize', 12, 'Color', 'm');
end

% Annotations for rise times
if ~isnan(T_r_x)
    % Mark the rise time start and end for X
    scatter(timeData(rise_idx_start_x), rad2deg(rise_start_x), 100, 'filled', 'b'); % Blue marker
    scatter(timeData(rise_idx_end_x), rad2deg(rise_end_x), 100, 'filled', 'b'); % Blue marker
    % Add text annotation for rise time
    text(timeData(rise_idx_end_x) + 0.2, rad2deg(rise_end_x), sprintf('T_r_X: %.2f s', T_r_x), 'FontSize', 12, 'Color', 'b');
end

if ~isnan(T_r_z)
    % Mark the rise time start and end for Z
    scatter(timeData(rise_idx_start_z), rad2deg(rise_start_z), 100, 'filled', 'r'); % Red marker
    scatter(timeData(rise_idx_end_z), rad2deg(rise_end_z), 100, 'filled', 'r'); % Red marker
    % Add text annotation for rise time
    text(timeData(rise_idx_end_z) + 0.2, rad2deg(rise_end_z), sprintf('T_r_Z: %.2f s', T_r_z), 'FontSize', 12, 'Color', 'r');
end

% Labels and legend
xlabel('Time (s)', 'FontSize', 14);
ylabel('Angle (degrees)', 'FontSize', 14);
legend('X-Axis Angle', 'X Desired Angle', 'Z-Axis Angle', 'Z Desired Angle', 'FontSize', 12, 'Location', 'best');
title('Motor Response Analysis', 'FontSize', 16);
grid on;

% Display Metrics
disp('Control Metrics:');
fprintf('Steady-state X: %.2f degrees\n', steady_state_x);
fprintf('Steady-state Z: %.2f degrees\n', steady_state_z);
fprintf('Settling Time X: %.2f s\n', T_s_x);
fprintf('Settling Time Z: %.2f s\n', T_s_z);
fprintf('Rise Time X: %.2f s\n', T_r_x);
fprintf('Rise Time Z: %.2f s\n', T_r_z);
