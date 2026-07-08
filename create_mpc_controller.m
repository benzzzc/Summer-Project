function mpcobj = create_mpc_controller(sys_plant, Ts)
% CREATE_MPC_CONTROLLER function creates the controller
% Inputs:
%   sys_plant: Continuous open-loop plant
%   Ts: Sample time (e.g., 0.01s)

    % Initialise MPC object
        mpcobj = mpc(sys_plant, Ts);

    % Set Horizons (100 steps * 0.01s = 1 second prediction)
        mpcobj.PredictionHorizon = 100;
        mpcobj.ControlHorizon = 10;

    % Set Physical Actuator Constraints (+/- 15 Newtons)
        mpcobj.MV(1).Min = -15;
        mpcobj.MV(1).Max = 15;
        
    % Set Tuning Weights
        mpcobj.Weights.OV = 1;       % Priority on target tracking
        mpcobj.Weights.ManipulatedVariablesRate = 0.1; % Priority on smooth motor moves
end