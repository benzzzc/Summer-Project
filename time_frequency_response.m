function time_frequency_response(sys, controller_name)
    % TIME_FREQUENCY_RESPONSE function provides step info for time domain
    % and frequncy response info
    
    time_info = stepinfo(sys);
    ss_value = dcgain(sys);
    ss_error = 1 - ss_value;
    
    fprintf('\n=== %s Analysis ===\n', controller_name);
    fprintf('Rise time: %.2f seconds\n', time_info.RiseTime);
    fprintf('Settling time: %.2f seconds\n', time_info.SettlingTime);
    fprintf('Overshoot: %.2f %%\n', time_info.Overshoot);
    fprintf('Peak value: %.2f\n', time_info.Peak);
    fprintf('Final Settled Value: %.6f\n', ss_value);
    fprintf('Steady-State Error:  %.6f\n', ss_error);
    
    bw_rad = bandwidth(sys);
    bw_hz = bw_rad / (2 * pi);
    fprintf('System bandwidth: %.2f rad/s / %.2f Hz\n', bw_rad, bw_hz);
    fprintf('=========================\n');
    
    % Create plots
    window_title = sprintf('%s - Time & Frequency Response', controller_name);
    figure('Name', window_title, 'Position', [100, 100, 1200, 500], 'Color', 'w'); 
        
    % Left Side: Step Response
    subplot(1, 2, 1);
    step(sys);
    grid on;
    % 3. Inject the name into the Plot Title
    title(sprintf('%s: Closed-Loop Step Response', controller_name));
    
    % Right Side: Bode Plot 
    subplot(1, 2, 2);
    bode(sys);
    grid on;
    title(sprintf('%s: Bode Magnitude & Phase', controller_name));

end