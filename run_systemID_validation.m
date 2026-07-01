function run_systemID_validation(sys_true, sys_est)
    % RUN_SYSTEMID_VALIDATION simulates the known model and the estimated 
    % model from system ID to ensure that the requirements have been 
    % captured

    % Create new input signal, u(t) = Asin(2*pi*f*t)
    t_val = 0:0.01:20; % 20 seconds
    u_val = zeros(size(t_val));
    u_val(t_val >= 2 & t_val < 10) = 5; % 5N Step input between 2 and 10s
    u_val(t_val >= 10) = 5 * sin(2*pi*1.0 * t_val(t_val >= 10)); % ensures 
    % only the time values less than 10 are sine waves

    % Simulate the system objects
    y_true = lsim(sys_true, u_val, t_val);
    y_est  = lsim(sys_est, u_val, t_val);

    % Calculate absolute tracking error
    y_error = abs(y_true - y_est);

    % 3. Generate the Validation Plots
    figure('Name', 'Model Validation Dashboard', 'NumberTitle', 'off');

    % Top Subplot: Input Force Profile
    subplot(3,1,1);
    plot(t_val, u_val, 'k', 'LineWidth', 1.5);
    title('Validation Input Profile (Unseen Signal Force)');
    ylabel('Force (N)');
    grid on;

    % Middle Subplot: Trajectory Comparison
    subplot(3,1,2);
    plot(t_val, y_true, 'b', 'LineWidth', 1); hold on;
    plot(t_val, y_est, 'r--', 'LineWidth', 1.5);
    title('Dynamic System Response Comparison');
    ylabel('Position (m)');
    legend('True Physical Plant', 'Grey-Box Estimated Model', 'Location', 'best');
    grid on;

    % Bottom Subplot: Numerical Tracking Error
    subplot(3,1,3);
    plot(t_val, y_error, 'r', 'LineWidth', 1.5);
    title('Absolute Discrepancy (Error) Between Models');
    xlabel('Time (s)');
    ylabel('Error (m)');
    grid on;
end