function [sys_CL_hinf, sys_penalty, K_hinf,gamma] = hinf_controller(sys)
    % HINF_CONTROLLER function takes in the estimated state space system
    % and creates a Hinf controller

    % W1: For DC gain penalise low frequency, steady state errors
    W1 = makeweight(100, 4, 0.1); 
    
    % W2: Hard physical limit of the motor (1 / 15 Newtons)
    W2 = 1/15;                     
    
    % W3: Tell the system to roll off and ignore high-frequency noise past
    % 40 rad/s
    W3 = makeweight(0.1, 40.0, 10);

    % Synthesize controller
    % K_hinf  : The actual controller to put on your microchip
    % sys_penalty : The weighted penalty system (used only for sigma plots)
    % the T_zw
    % gamma   : The performance score (needs to be < 1)
    [K_hinf, sys_penalty, gamma, ~] = mixsyn(sys, W1, W2, W3);
    fprintf('Gamma = %.4f\n', gamma);

    % Multiply the raw plant by the controller to get the Open Loop
    OL_hinf = sys * K_hinf; 
    
    % Wrap negative feedback around it 
    sys_CL_hinf = feedback(OL_hinf, 1);
end