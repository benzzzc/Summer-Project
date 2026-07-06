function time_frequency_response (sys)
% ANALYSE_CONTROL function takes the system object and analyses the time 
% and frequency domain responses

time_info = stepinfo(sys);
ss_value = dcgain(sys);
ss_error = 1 - ss_value;
fprintf('Rise time: %.2f seconds\n', time_info.RiseTime);
fprintf('Settling time: %.2f seconds\n', time_info.SettlingTime);
fprintf('Overshoot: %.2f \n', time_info.Overshoot);
fprintf('Peak value: %.2f\n', time_info.Peak);
fprintf('Final Settled Value: %.6f\n', ss_value);
fprintf('Steady-State Error:  %.6f\n', ss_error);

% Find the Bandwidth, tells you how fast the system responds 
bw_rad = bandwidth(sys);
bw_hz = bw_rad / (2 * pi);
fprintf('System bandwdth: %.2f rad/s / %.2f Hz\n',bw_rad, bw_hz);

% Plot the tiem and frequency responses
figure('Position', [100, 100, 1200, 500]); % Wide canvas for side-by-side plots
    
% Left Side: Step Response
subplot(1, 2, 1);
step(sys);
grid on;
title('Time Domain: Closed-Loop Step Response');

% Right Side: Bode Plot (Frequency Response)
subplot(1, 2, 2);
bode(sys);
grid on;
title('Frequency Domain: Bode Intensity & Phase');

end