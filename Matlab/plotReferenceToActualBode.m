function plotReferenceToActualBode(tt, theta_reference, theta_actual)
    % FFT Sampling
    Ts = mean(diff(tt));
    Fs = 1 / Ts;
    n = length(tt);
    f = (0:n-1) * (Fs / n);

    % Apply Hamming Window
    window = hamming(n)';
    FFT_ref = fft(theta_reference .* window);
    FFT_act = fft(theta_actual .* window);

    % Transfer Function
    H = FFT_act ./ FFT_ref;
    magnitude = abs(H(1:floor(n/2)));
    phase = angle(H(1:floor(n/2)));

    % Positive Frequencies
    positive_freqs = f(1:floor(n/2));

    % Bode Plot
    figure;
    subplot(2, 1, 1);
    semilogx(positive_freqs, 20 * log10(magnitude), 'LineWidth', 1.5);
    title('Bode Plot: Reference Angle to Actual Angle');
    xlabel('Frequency (Hz)');
    ylabel('Magnitude (dB)');
    grid on;

    subplot(2, 1, 2);
    semilogx(positive_freqs, phase * 180 / pi, 'LineWidth', 1.5);
    xlabel('Frequency (Hz)');
    ylabel('Phase (degrees)');
    grid on;
end
