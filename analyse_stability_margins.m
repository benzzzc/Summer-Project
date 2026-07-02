function [GM, PM] = analyse_stability_margins(sys, K)
% ANALYSE_STABILITY_MARGINS function finds the open loop gain and finds the
% gain and phase margins

    % Construct open loop gain 
    % L(s) = K (sI - A)^-1 Ba, use the ss to tf rule to find ss model

    Aa = sys.A;
    Ba = sys.B;
    sys_OL_gain = ss(Aa, Ba, K, 0);
    
    % Calculate the margins
    [GM_raw, PM, w_gm, w_pm] = margin(sys_OL_gain);
    GM_dB = 20 * log10(GM_raw);

    fprintf('Gain Margin:  %.2f dB (at %.2f rad/s)\n', GM_dB, w_gm);
    fprintf('Phase Margin: %.2f°  (at %.2f rad/s)\n', PM, w_pm);
    GM = GM_dB;

    figure('Name', 'Open-Loop Loop Gain Stability Margins');
    margin(sys_OL_gain);
    grid on;
    
    % Nyquist plot
    figure('Name', 'Nyquist Diagram of Open-Loop System');
    nyquist(sys_OL_gain);
    axis([-12 2 -10 10]);    
    grid on;
end