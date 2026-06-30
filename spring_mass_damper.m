%% Getting the state space model and simulink model 

% Create 'secret' parameters for the model, will use system ID to find
% these
m = 10;  % kg
b = 5;   % Ns/m
k = 8;   % N/m

% State space representation of the model
A = [0 1;
    -k/m -b/m];

B = [0;
     1/m];

C = [1 0];

D = 0;

sys_plant = ss(A,B,C,D);

% Find the natural frequency 
wn_rad = sqrt(k/m);
fn_hz = wn_rad / (2*pi); 

% Set up chirp block param
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

%% LQR 

% Get system model from the estimated parameters

m = m_est;
b = b_est;
k = k_est;

A = A_est;
B = B_est;

sys_plant = ss(A,B,C,D);

% Check observability and Controllability
Co = ctrb(sys_plant);
c_rank = rank(Co);
Ob = obsv(sys_plant);
o_rank = rank(Ob);






