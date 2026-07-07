%% Getting the state space model and simulink model 

clear;

% Create 'secret' parameters for the model, will use system ID to find
% these
m = 1;  % kg
b = 2;   % Ns/m
k = 16;   % N/m

% State space representation of the model
A = [0 1;
    -k/m -b/m];

B = [0;
     1/m];

C = [1 0];

D = [0];

sys_plant = ss(A,B,C,D);

% Find the natural frequency, critical behaviour 
wn_rad = sqrt(k/m);
fn_hz = wn_rad / (2*pi); 

% Set up chirp block param, signal sweep allows us to collect the important
% frequencies
chirp_f_start = 0.01;        
chirp_f_target = fn_hz * 5;   

out = sim("plant.slx");

%% System Identification (Grey-Box Method)

% Get data
t = out.tout;
Ts = 0.01; % fixed sample time
u = out.simin;

% Extract only the state being measured
y_pos = out.simout(:, 2);

% System Id data object
data = iddata(y_pos, u, Ts);

% Set the unkowns as guesses, 1
A_guess = [0,  1; 
          -1, -1]; 
B_guess = [0; 
           1];

% Create state space model with identifiable parameters
sys_template = idss(A_guess, B_guess, C, D, 'Ts', 0);

% Define the system structure
% true means find
sys_template.Structure.A.Free = [false, false; 
                                 true,  true]; 

sys_template.Structure.B.Free = [false; 
                                 true];

sys_template.Structure.C.Free = [false, false];

sys_template.Structure.D.Free = [false];

% Run estimation and extract the estimated matrices
estimated_grey = ssest(data, sys_template);
[A_est, B_est] = ssdata(estimated_grey);

% Find parameters
m_est = 1 / B_est(2);
b_est = -A_est(2,2) * m_est;
k_est = -A_est(2,1) * m_est;

% Display the results
fprintf('Estimated mass: %.4f kg\n', m_est);
fprintf('Estimated damping: %.4f Ns/m\n', b_est);
fprintf('Estimated spring constant: %.4f N/m\n', k_est);


%% Check observability and Controllability
sys_plant_ID = ss(A_est,B_est,C,D);
Co = ctrb(sys_plant);
c_rank = rank(Co);
Ob = obsv(sys_plant);
o_rank = rank(Ob);

%% Plot the open-loop step response
figure;
step(sys_plant_ID);
grid on;
title('Open-Loop Plant Step Response');

%% Validate system identification model using a differnet input

run_systemID_validation(sys_plant, sys_plant_ID);

%% LQR

% Form the augmented system with augmented state integral error, be careful
% of the dimensions. 

Aa = [A_est [0;0];
      -C 0];

Ba = [B_est;
      0];

Ca = [C 0];

sys_OL_aug = ss(Aa, Ba, Ca, 0);

% Define the weighting matrices
% Use Bryson rule to find the starting point for trail and error, go higher
% if needed, as it is 1/value^2

% Penalises position, velocity and integral error respectively
Q = [100 0 0;
     0 0.25 0;
     0 0 400];

% Penalises control effort - remember we can only have one control input
R = [1/(15^2)];

N = [0;
     0;
     0]; 

% Find the K matrices Kx (for states) and Ki (inegratal control error)
K_LQR = lqr(sys_OL_aug, Q, R, N);
Kx = K_LQR(:, 1:2);
Ki = K_LQR(3);

% Construct the state space representation of the augmented system with 
% closed loop feedback

% A_cl = Aa - (Ba * K);
A_cl = [A_est - (B_est * Kx), -B_est * Ki;
        -C,                    0];

% Defines how the reference target enters the system
Br = [0; 
      0; 
      1];


sys_CL_LQR = ss(A_cl, Br, Ca, 0);


%% Analyse the LQR controller

time_frequency_response(sys_CL_LQR, 'LQR Controller');

% Construct the LQR Open Loop Gain: L(s) = K*(sI - A)^-1*B and plot the
% bode and nyquist plot
sys_LQR_loop = ss(Aa, Ba, K_LQR, 0);
[GM_LQR, PM_LQR] = bode_nyquist(sys_LQR_loop, 'LQR Controller');

%% Construct Hinf controller

[sys_CL_hinf, sys_penalty, K_hinf, gamma] = hinf_controller(sys_plant_ID);

%% Analyse the Hinf controller

time_frequency_response(sys_CL_hinf, 'H infinity Controller');

% Construct the H-infinity Open Loop Gain: L(s) = P(s)*K(s) and plot the
% bode and nyquist plot
sys_hinf_loop = sys_plant_ID * K_hinf; 
[GM_hinf, PM_hinf] = bode_nyquist(sys_hinf_loop, 'H infinity Controller');

% Plot the Sigma Plot for robustness check
figure('Name', 'H-Infinity Robustness Check', 'Color', 'w');
sigma(sys_penalty);
grid on;

% Add a reference line at 0 dB (which equals a gain of 1, or Gamma = 1)
yline(0, 'r--', 'Gamma = 1 (Performance Limit)', 'LineWidth', 2);
title('Singular Value Plot of Weighted Closed-Loop System');

%% --- MPC Generation and Analysis ---

% 1. Create the Controller (Assuming your continuous plant is sys_plant_ID)
Ts = 0.01;
my_mpc = create_mpc_controller(sys_plant_ID, Ts);

% 2. Setup the Time-Domain Simulation
T_sim_seconds = 10;
T_steps = T_sim_seconds / Ts;  % Number of discrete steps
target_ref = ones(T_steps, 1); % Step command of 1 meter

% 3. Run the Non-Linear Simulation
% y_mpc = Position, t_mpc = Time, u_mpc = Motor Force
[y_mpc, t_mpc, u_mpc] = sim(my_mpc, T_steps, target_ref);

% 4. Plot to verify Constraint Handling
figure('Name', 'MPC Constraint Handling', 'Color', 'w');

% Position Plot
subplot(2,1,1);
plot(t_mpc, y_mpc, 'LineWidth', 2);
yline(1, 'k--', 'Target');
grid on; title('MPC: System Position'); xlabel('Time (s)');

% Motor Effort Plot
subplot(2,1,2);
plot(t_mpc, u_mpc, 'r', 'LineWidth', 2);
yline(15, 'k--', 'Max Push (+15 N)', 'Color', 'r');
yline(-15, 'k--', 'Max Pull (-15 N)', 'Color', 'r');
grid on; title('MPC: Motor Effort (Notice the clipping!)'); xlabel('Time (s)');



