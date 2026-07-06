function [GM, PM] = bode_nyquist(sys_OL)
% BODE_NYQUIST analyses the stability margins of any given open-loop system

    % Calculate the raw margins
    [GM_raw, PM, w_gm, w_pm] = margin(sys_OL);
    GM_dB = 20 * log10(GM_raw);
    
    % Print results cleanly to the command window
    fprintf('\n--- Stability Margins ---\n');
    fprintf('Gain Margin:  %.2f dB (at %.2f rad/s)\n', GM_dB, w_gm);
    fprintf('Phase Margin: %.2f°  (at %.2f rad/s)\n', PM, w_pm);
    fprintf('-------------------------\n');
    
    % Assign output
    GM = GM_dB;
    
    % Plot the Open-Loop Bode Diagram
    figure('Name', 'Open-Loop Bode Stability Margins', 'Color', 'w');
    margin(sys_OL);
    grid on;
    
    % Plot the Nyquist Diagram
    figure('Name', 'Nyquist Diagram of Open-Loop System', 'Color', 'w');
    nyquist(sys_OL);
    grid on;
    
    % Zoom in
    axis([-3 2 -2 2]);
end