function [sys_CL_hinf, sys_penalty, K_hinf] = hinf_controller(sys)
    % HINF_CONTROLLER function takes in the estimated state space system
    % and creates a Hinf controller

    % Define the weights for penalising the tracking error, Dc gain,
    % crossover freuency and high frequency gain, we care not for the high
    % frequency as the motor cannot move that fast anyways
    W1 = makeweight(100, 1.5, 0.05);

    % Define the weights for penalising the Actuator effort, roughly
    % equated to maximum force of 10N
    W2 = 0.1;

    % Define the weights for penalising High frequency noise, DC gain,
    % crossover frequency and high frequency gain. Large penalty on high
    % frequency gain as we want to dampen any high freuency vibration, low
    % for the low frequency as we want them to pass
    W3 = makeweight(0.01, 20, 10);
    
    % Synthesize controller
    % K_hinf  : The actual controller to put on your microchip
    % sys_penalty : The weighted penalty system (used only for sigma plots)
    % the T_zw
    % gamma   : The performance score (needs to be < 1)
    disp('Synthesizing H-infinity Controller...');
    [K_hinf, sys_penalty, gamma, ~] = mixsyn(sys, W1, W2, W3);
    fprintf('Synthesis complete. Gamma = %.4f\n', gamma);

    % Multiply the raw plant by the controller to get the Open Loop
    OL_hinf = sys * K_hinf; 
    
    % Wrap negative feedback around it 
    sys_CL_hinf = feedback(OL_hinf, 1);
end