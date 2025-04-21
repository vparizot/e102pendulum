%final proj e102
clear
close all

%% Define Variables
g = 9.8; % gravity
lp = 0.5; %length of pendulum
alpha = 0.5; %rad/s^2


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% STEP 1: Find A, B, C, D
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Define A,B,C,D Matrix
A = [0 1 0 0; g/lp 0 0 0; 0 0 0 1; 0 0 0 0];

B = [0; -1/lp ; 0; 1];

C = [1 0 0 0; 0 0 1 0];

D = [zeros(2,1)];

Bprime = [0; 1; 0; 0];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% STEP 2: Establish the stability, controllability and observability of 
%% the linearized plant.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% STABILITY: Eigenvalues of A
eigenA = eig(A);  % = 4.4272, -4.4272, 0, 0
% there's a real eigenvalue, so not stable

%% CONTROLLABILITY: Find mc for controlability
mc = [B A*B A^2*B A^3*B];
rankMc = rank(mc); % = 4, full rank so controllable

%% OBSERVABILITY: Find observability matric
mo = [C; C*A; C*A^2; C*A^3];
rankMo = rank(mo); % = 4, full rank, so observable bc same as # of variables in state space

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% STEP 3: Design a state feedback control system with integral action and 
%% an observer for the linearized plant. Use pole placement design.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Define Augmented System w/ state vars, zeta & zeta_dot
% augmentedZeta = [xI; xbar];
Aa = [zeros(1,1) -C(2,:); zeros(4,1) A]; % use 2nd row of c bc we only care about posistion for timing
Ba = [0; B];
Ca = [zeros(2,1) C];
Da = [D];

%% Find wn and z for 2nd order with s, displacement
syms omegan
z = 0.999; %Set damping high for minimal overshoot
ts = 4/(z*omegan) == 10; % for 2 percent settl time
wn = solve(ts, omegan);
% wn = soln.omegan %solved w/ omegan

b = 2*z*wn;
c = wn^2;
p1 = eval((-b+sqrt(b^2-4*c)) / 2);
p2 = eval((-b-sqrt(b^2-4*c)) / 2);

%% place poles & find K's
des_poles = [p1 p2 real(p1)*10 (real(p1)+0.02)*10 (real(p1)+.01)*10]; % by overshoot & settling time to find; note 5 poles bc of augmented xi
obs_poles = 5*[p1 p2 real(p1)*10 (real(p1)+0.02)*10]; % 5x times faster than desired poles (could do up to 10x, note only 4 poles bc use regular A and C for observer
augmentedK = place(Aa, Ba, des_poles) % for augmented, you design observer
% augmentedK = 1.0e+04 * 4.4622   -2.5503   -0.6517   -4.1172   -1.2864
Ki = augmentedK(1); % this should be a positive value 
K = augmentedK(2:5);

L = place(A', C', obs_poles) %% use regular C and A

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% STEP 4: Build a Simulink model of the nonlinear cart-pendulum plant
%% together with the control system.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% STEP 5: Use the control design parameters from the designs in Part I 
%% and run the simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% STEP 6: Fine-tune the design to achieve the desired specifications.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% STEP 7: Increment the angular acceleration disturbance w = alpha(t) to 
%% find the largest allowable disturbance that maintains closed loop stability.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%