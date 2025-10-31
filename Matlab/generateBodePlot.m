function [magnitude, phase, frequencies] = generateBodePlot(tt, omega, output_angle, input_position, TaskParams)
    % Parameters
    g = 9.81;

    % Calculate instantaneous frequency
    instantaneous_freq = abs(omega) / (2 * pi);
    frequencies = logspace(log10(min(instantaneous_freq)), log10(max(instantaneous_freq)), 100);

    % Initialize magnitude and phase
    magnitude = zeros(size(frequencies));
    phase = zeros(size(frequencies));

    for i = 1:length(frequencies)
        freq = frequencies(i);

        % Find indices corresponding to current frequency
        freq_idx = find(abs(instantaneous_freq - freq) < 0.01);
        if isempty(freq_idx), continue; end

        % Input and output signals
        input_signal = input_position(freq_idx);
        output_signal = output_angle(freq_idx);

        % Avoid division by zero
        if all(input_signal == 0) || all(output_signal == 0)
            magnitude(i) = NaN;
            phase(i) = NaN;
            continue;
        end

        % Compute peak-to-peak ratios for magnitude
        mag_input = max(input_signal) - min(input_signal);
        mag_output = max(output_signal) - min(output_signal);
        magnitude(i) = mag_output / mag_input;

        % Compute phase shift
        [~, input_peak] = max(input_signal);
        [~, output_peak] = max(output_signal);
        if isempty(input_peak) || isempty(output_peak)
            phase(i) = NaN;
        else
            time_shift = tt(freq_idx(output_peak)) - tt(freq_idx(input_peak));
            phase(i) = -2 * pi * freq * time_shift;
        end
    end

    % Clean up
    valid_idx = ~isnan(magnitude) & ~isnan(phase);
    magnitude = magnitude(valid_idx);
    phase = phase(valid_idx);
    frequencies = frequencies(valid_idx);
end
