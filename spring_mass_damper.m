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

%% MPC Synthesis 

% Create the Controller 
Ts = 0.01;
my_mpc = create_mpc_controller(sys_plant_ID, Ts);

% In command line to test stability, review(my_mpc) does all of the crazy 
% stuff for you

%% 1 Meter reference tracking test

% Setup Simulation Parameters                             
t_ref = 0:Ts:5;                           
r_ref = 0.7 * ones(length(t_ref), 1);  % milply ones by something to change ref         

% Simulate the Linear controllers

% Setup reference tracking systems
% LQR closed loop with input command tracking matrix           
sys_cl_u_LQR = ss(A_cl, [0; 0; 1], [-Kx, -Ki], 0); 

% H-infinity tracking configuration
sys_cl_u_hinf = feedback(K_hinf, sys_plant_ID);

% Simulate LQR and H-infinity responses, for both input and output
[y_lqr_r, ~] = lsim(sys_CL_LQR, r_ref, t_ref);
[u_lqr_r, ~] = lsim(sys_cl_u_LQR, r_ref, t_ref);

[y_hinf_r, ~] = lsim(sys_CL_hinf, r_ref, t_ref);
[u_hinf_r, ~] = lsim(sys_cl_u_hinf, r_ref, t_ref);

% Simulate MPC

% Extract discrete system matrices for manual loop
sys_d = c2d(sys_plant_ID, Ts);
Ad = sys_d.A; Bd = sys_d.B; Cd = sys_d.C;

x_plant_mpc = [0; 0];               
xc_mpc = mpcstate(my_mpc); 
y_mpc_r = zeros(length(t_ref), 1);
u_mpc_r = zeros(length(t_ref), 1);

for k = 1:length(t_ref)
    y_mpc_r(k) = Cd * x_plant_mpc;
    
    % MPC computes the optimal move keeping constraints in mind
    u_mpc_r(k) = mpcmove(my_mpc, xc_mpc, y_mpc_r(k), r_ref(k));
    
    % Update physical plant
    x_plant_mpc = Ad * x_plant_mpc + Bd * u_mpc_r(k);
end

% Plot reference tracking figures

figure('Name', 'Reference Tracking Test Step Response', ...
    'Color', 'w', 'Position', [100, 100, 900, 750]);

% Top Plot: Position Tracking Performance
subplot(2,1,1);
plot(t_ref, y_lqr_r, 'b', 'LineWidth', 1.5); hold on;
plot(t_ref, y_hinf_r, 'g', 'LineWidth', 1.5);
plot(t_ref, y_mpc_r, 'r', 'LineWidth', 2);
plot(t_ref, r_ref, 'k--', 'LineWidth', 1.2); % Target line
grid on; 
title('System Displacement (y): Reference Tracking'); 
xlabel('Time (s)'); ylabel('Position (m)');
legend('LQR', 'H-Infinity',  'MPC', 'Target Reference', ...
    'Location', 'Southeast');

% Bottom Plot: Actuator Force and Saturation Realities
subplot(2,1,2);
plot(t_ref, u_lqr_r, 'b', 'LineWidth', 1.5); hold on;
plot(t_ref, u_hinf_r, 'g', 'LineWidth', 1.5);
plot(t_ref, u_mpc_r, 'r', 'LineWidth', 2);
yline(15, 'k-', 'Max Motor Limit (+15 N)', 'LineWidth', 1.2);
yline(-15, 'k-', 'Max Motor Limit (-15 N)', 'LineWidth', 1.2);
grid on; 
title('Motor Command (u): Constraint Adherence'); 
xlabel('Time (s)'); ylabel('Force (N)');
legend('LQR Demand', 'H-Infinity Demand', 'MPC Response', ...
    'Limits');

%% Disturbance rejection, high frequency disturbance 

% Setup Simulation Parameters                             
t_ref = 0:Ts:7;                           
r_ref = zeros(length(t_ref), 1); 

% Create the Disturbance Signal (Physical input force)
d_dist = zeros(length(t_ref), 1);

% The Bump: Positive 10N force between 1s and 1.5s (Half-sine wave)
bump_duration = 0.5;
bump_idx = (t_ref >= 1) & (t_ref <= 1 + bump_duration);
d_dist(bump_idx) = 10 * sin(pi * (t_ref(bump_idx) - 1) / bump_duration);

% The Pothole: Negative 10N force between 4s and 4.5s
pothole_duration = 0.5;
pothole_idx = (t_ref >= 4) & (t_ref <= 4 + pothole_duration);
d_dist(pothole_idx) = -10 * sin(pi * (t_ref(pothole_idx) - 4) / pothole_duration);

% Setup LQR for Disturbance
% Augment B and C matrices because LQR has an integrator state, so that the
% disturbance only hits the physical states 
B_dist_LQR = [sys_plant_ID.B; 0];
C_y_LQR = [sys_plant_ID.C, 0];

% Map disturbance 'd' to output 'y' and motor 'u'
sys_cl_y_LQR_dist = ss(A_cl, B_dist_LQR, C_y_LQR, 0);
sys_cl_u_LQR_dist = ss(A_cl, B_dist_LQR, [-Kx, -Ki], 0);

% Simulate LQR Disturbance Rejection
[y_lqr_dist, ~] = lsim(sys_cl_y_LQR_dist, d_dist, t_ref);
[u_lqr_dist, ~] = lsim(sys_cl_u_LQR_dist, d_dist, t_ref);

% Setup H-Infinity for Disturbance
% Block diagram math: Transfer function from Disturbance to Position
sys_cl_y_hinf_dist = feedback(sys_plant_ID, K_hinf);

% Block diagram math: Transfer function from Disturbance to Motor Command
sys_cl_u_hinf_dist = -K_hinf * feedback(sys_plant_ID, K_hinf);

% Simulate H-infinity Disturbance Rejection
[y_hinf_dist, ~] = lsim(sys_cl_y_hinf_dist, d_dist, t_ref);
[u_hinf_dist, ~] = lsim(sys_cl_u_hinf_dist, d_dist, t_ref);

% Simulate MPC
sys_d = c2d(sys_plant_ID, Ts);
Ad = sys_d.A; Bd = sys_d.B; Cd = sys_d.C;

x_plant_mpc = [0; 0];               
xc_mpc = mpcstate(my_mpc); 
y_mpc_dist = zeros(length(t_ref), 1);
u_mpc_dist = zeros(length(t_ref), 1);

for k = 1:length(t_ref)
    y_mpc_dist(k) = Cd * x_plant_mpc;
    
    % MPC computes the optimal move to stay at r_ref(k)
    u_mpc_dist(k) = mpcmove(my_mpc, xc_mpc, y_mpc_dist(k), r_ref(k));
    
    % The physical plant feels BOTH the motor command and the road disturbance
    x_plant_mpc = Ad * x_plant_mpc + Bd * (u_mpc_dist(k) + d_dist(k));
end

% Plotting
figure('Name', 'Disturbance Rejection: Bump and Pothole', 'Color', 'w', 'Position', [100, 50, 900, 900]);

% Subplot 1: Position
subplot(3,1,1);
plot(t_ref, y_lqr_dist, 'b', 'LineWidth', 1.5); hold on;
plot(t_ref, y_hinf_dist, 'g', 'LineWidth', 1.5);
plot(t_ref, y_mpc_dist, 'r', 'LineWidth', 2);
yline(0, 'k--', 'Target', 'LineWidth', 1.2); 
grid on; 
title('System Displacement (y): Returning to Zero'); 
legend('LQR', 'H-Infinity', 'MPC', 'Target', 'Location', 'best');

% Subplot 2: Motor Command
subplot(3,1,2);
plot(t_ref, u_lqr_dist, 'b', 'LineWidth', 1.5); hold on;
plot(t_ref, u_hinf_dist, 'g', 'LineWidth', 1.5);
plot(t_ref, u_mpc_dist, 'r', 'LineWidth', 2);
yline(15, 'k-', 'Max Motor Limit (+15 N)', 'LineWidth', 1.2);
yline(-15, 'k-', 'Max Motor Limit (-15 N)', 'LineWidth', 1.2);
grid on; 
title('Motor Command (u): Fighting the Disturbance'); 
legend('LQR Demand', 'H-Infinity Demand', 'MPC Response', 'Upper Limit', 'Lower Limit', 'Location', 'best');

% Subplot 3: Disturbance Profile
subplot(3,1,3);
plot(t_ref, d_dist, 'k', 'LineWidth', 1.5);
grid on; 
title('External Disturbance Force (d): Bump and Pothole');
legend('Disturbance Force', 'Location', 'best');