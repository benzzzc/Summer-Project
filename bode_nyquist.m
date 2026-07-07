function [GM, PM] = bode_nyquist(sys_OL, controller_name)
% BODE_NYQUIST analyses the stability margins of any given open-loop system
    
    % Calculate the raw margins
    [GM_raw, PM, w_gm, w_pm] = margin(sys_OL);
    GM_dB = 20 * log10(GM_raw);
    
    % Print results cleanly to the command window
    fprintf('\n--- %s Stability Margins ---\n', controller_name);
    fprintf('Gain Margin:  %.2f dB (at %.2f rad/s)\n', GM_dB, w_gm);
    fprintf('Phase Margin: %.2f°  (at %.2f rad/s)\n', PM, w_pm);
    fprintf('-------------------------\n');
    
    % Assign output
    GM = GM_dB;
    
    % Build the dynamic strings for the figure windows and plot titles
    bode_title = sprintf('%s: Open-Loop Bode Stability Margins', ...
        controller_name);
    nyquist_title = sprintf('%s: Nyquist Diagram of Open-Loop System', ...
        controller_name);
    
    % Plot the Open-Loop Bode Diagram
    figure('Name', bode_title, 'Color', 'w');
    margin(sys_OL);
    grid on;
    title(bode_title); % Updates the text inside the plot area
    
    % Plot the Nyquist Diagram
    figure('Name', nyquist_title, 'Color', 'w');
    nyquist(sys_OL);
    grid on;
    title(nyquist_title); % Updates the text inside the plot area
    
    % Zoom in
    axis([-3 2 -2 2]);
end