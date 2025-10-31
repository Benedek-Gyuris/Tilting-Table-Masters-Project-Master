function gg = BenceShowResults(tt, yy, tvec, TaskParams, Torque, Torque_fb, Torque_add, Torque_canc, Error, desired_angle)
    m = TaskParams.m;
    g = 9.81;

    % Extract the state variables for the X-axis
    AngleX = yy(:, 1);          % x1: X-axis angle (rad)
    AngularVelocityX = yy(:, 2); % x2: X-axis angular velocity (rad/s)

    % Extract Cartesian positions from TaskParams
    x_pos = TaskParams.x_pos; % X position trajectory
    curr_x_velocity = TaskParams.x_velocity; % Velocity of position data

    % Create figure for dynamics plots
    gg = figure(1);

    % Subplot for X-axis Angle vs Time
    subplot(5, 1, 1); % Reduced to 5 subplots
    plot(tt, rad2deg(AngleX), 'b-', 'LineWidth', 1.5);
    hold on;
    plot(tvec, rad2deg(desired_angle), 'r--', 'LineWidth', 1.5); % Mapped angle
    xlabel('Time (s)');
    ylabel('Angle (degrees)');
    legend('X-Axis Angle (x1)', 'Mapped Angle');
    title('X-Axis Angle vs Mapped Angle');
    grid on;

    % Subplot for X-axis Angular Velocity vs Time
    subplot(5, 1, 2);
    plot(tt, AngularVelocityX, 'g-', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('X-Axis Angular Velocity (rad/s)');
    title('X-Axis Angular Velocity vs Time');
    grid on;

    % Subplot for X Position over Time
    subplot(5, 1, 3);
    plot(tvec, x_pos, 'r-', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('X Position (m)');
    title('X Position vs Time');
    grid on;

    % Subplot for Velocity of Position Data
    subplot(5, 1, 4); % New subplot for velocity
    plot(tvec, curr_x_velocity, 'c-', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Velocity of Position Data vs Time');
    grid on;

    % Subplot for Error (e) vs Time
    subplot(5, 1, 5); % New subplot for error
    plot(tt, Error, 'k-', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Error (rad)');
    title('Controller Error (e) vs Time');
    grid on;

    % Compute Control Metrics
    % Steady-state value
    steady_state = rad2deg(mean(AngleX(end-100:end))); % Mean of the last 100 values
    ref_angle_deg = rad2deg(desired_angle(end)); % Steady-state reference angle

    % Steady-state error (e_ss)
    e_ss = abs(ref_angle_deg - steady_state);

    % Settling Time (T_s)
   % Define a tolerance of ±2% around the reference angle
    tol = 0.01 * ref_angle_deg;
    
    % Find indices where the angle is within the tolerance band
    within_tolerance = abs(rad2deg(AngleX) - ref_angle_deg) <= tol;
    
    % Identify the settling time: first index where the response remains
    % within tolerance until the end of the simulation
    for i = 1:length(within_tolerance)
        if all(within_tolerance(i:end)) % Check if it stays within tolerance
            T_s = tt(i); % Settling time
            break;
        end
    end
    
    % Rise Time (T_r)
    rise_start = 0.1 * ref_angle_deg;
    rise_end = 0.9 * ref_angle_deg;
    rise_idx_start = find(rad2deg(AngleX) >= rise_start, 1, 'first');
    rise_idx_end = find(rad2deg(AngleX) >= rise_end, 1, 'first');
    T_r = tt(rise_idx_end) - tt(rise_idx_start);

    % Create the large figure
    largeFigure = figure(2);
    set(largeFigure, 'Position', [100, 100, 1200, 600]); % Adjust figure size

    % Plot actual vs reference angles
    plot(tt, rad2deg(AngleX), 'b-', 'LineWidth', 2);
    hold on;
    plot(tvec, rad2deg(desired_angle), 'r--', 'LineWidth', 2);

    % Add annotations for metrics
    plot([0, max(tt)], [steady_state, steady_state], 'k--', 'LineWidth', 1);

    % Mark settling time
    xline(T_s, 'g--', 'LineWidth', 1.5);
    text(T_s, steady_state - 3, sprintf('T_s: %.2fs', T_s), 'FontSize', 12);

    % Mark rise time
    xline(tt(rise_idx_start), 'm--', 'LineWidth', 1.5);
    xline(tt(rise_idx_end), 'm--', 'LineWidth', 1.5);
    text(tt(rise_idx_end), ref_angle_deg - 5, sprintf('T_r: %.2fs', T_r), 'FontSize', 12);

    % Finalize plot
    xlabel('Time (s)', 'FontSize', 14);
    ylabel('Angle (degrees)', 'FontSize', 14);
    legend('X-Axis Angle (x1)', 'Reference Angle', 'FontSize', 12, 'Location', 'best');
    title('X-Axis Angle vs. Reference Angle', 'FontSize', 16);
    grid on;
    axis square
    hold off;
    
    
    figure(3);
    L = TaskParams.L; % Length of the platform
    g = 9.81; % Gravitational constant

    % Compute accelerations
    actual_acceleration = g * L * sin(AngleX); % Based on theta_now
    reference_acceleration = g * L * sin(desired_angle); % Based on theta_reference

    % Create the plot
    plot(curr_x_velocity, actual_acceleration, 'b-', 'LineWidth', 1.5);
    hold on;
    plot(curr_x_velocity, reference_acceleration, 'r--', 'LineWidth', 1.5);
    xlabel('X Velocity (m/s)', 'FontSize', 12);
    ylabel('Acceleration (m/s^2)', 'FontSize', 12);
    legend('Actual Acceleration (gLsin(\theta_{now}))', ...
           'Reference Acceleration (gLsin(\theta_{ref}))', 'FontSize', 12, 'Location', 'best');
    title('Acceleration vs X Velocity', 'FontSize', 14);
    grid on;
    hold off;

    % Create a separate figure for torque plots
    figure(4);

    % Subplot for Feedback Torque (\tau_fb) vs Time
    subplot(4, 1, 1);
    plot(tt, Torque_fb, 'b-', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('\tau_{fb} (Nm)');
    title('Feedback Torque (\tau_{fb}) vs Time');
    grid on;

    % Subplot for Additional Torque (\tau_add) vs Time
    subplot(4, 1, 2);
    plot(tt, Torque_add, 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('\tau_{add} (Nm)');
    title('Additional Torque (\tau_{add}) vs Time');
    grid on;

    % Subplot for Cancellation Torque (\tau_canc) vs Time
    subplot(4, 1, 3);
    plot(tt, Torque_canc, 'g-.', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('\tau_{canc} (Nm)');
    title('Cancellation Torque (\tau_{canc}) vs Time');
    grid on;

    % Subplot for Total Torque vs Time
    subplot(4, 1, 4);
    plot(tt, Torque, 'k-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Torque (Nm)');
    title('Total Torque vs Time');
    grid on;
    
    figure(5);

    [magnitude, phase, frequencies] = generateBodePlot(tt, TaskParams.w, AngleX, desired_angle, TaskParams);

    % Plot Magnitude
    subplot(2, 1, 1);
    semilogx(frequencies, 20 * log10(magnitude), 'LineWidth', 1.5);
    xlabel('Frequency (Hz)');
    ylabel('Magnitude (dB)');
    title('Bode Plot: Acceleration to Hand Position');
    grid on;

    % Plot Phase
    subplot(2, 1, 2);
    semilogx(frequencies, phase * 180 / pi, 'LineWidth', 1.5);
    xlabel('Frequency (Hz)');
    ylabel('Phase (degrees)');
    grid on;
    % Call the function to generate Bode plot

    plotReferenceToActualBode(tt, desired_angle, AngleX);
end
