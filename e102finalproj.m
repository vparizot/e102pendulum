%final proj e102
clear
close all

%% Define Variables
g = 9.8; % gravity
L = 0.5;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% STEP 1: Find A, B, C, D
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Define Matrix
A = [0 1 0 0; g/L 0 0 0; 0 0 0 1; 0 0 0 0];

B = [0; -1/L ; 0; 1];

C = [1 0 0 0; 0 0 1 0];

D = [0];

Bprime = [0; 1; 0; 0];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% STEP 2: Establish the stability, controllability and observability of 
%% the linearized plant.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% eigenvalues of A
eigenA = eig(A);  % = 4.4272, -4.4272, 0, 0
% there's a real eigenvalue, so not stable

%% Find mc for controlability
mc = [B A*B A^2*B A^3*B];
rankMc = rank(mc); % = 4, full rank so controllable

%% Find observability matric
mo = [C; C*A; C*A^2; C*A^3]
rankMo = rank(mo); % = 4, full rank, so observable bc same as # of variables in state space

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% STEP 3: Design a state feedback control system with integral action and 
%% an observer for the linearized plant. Use pole placement design.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% define augmented system w/ state vars, zeta & zeta_dot
% augmentedZeta = [xI; xbar];
Aa = [zeros(2,4) -C; zeros(4,4) A];
Ba = [0; B]    
Ca = [zeros(2,4) C]
Da = [D]









% 
% A = [...]; B = [...]; Cr = [0 0 1 0];
% 
% Aa = [A, zeros(4,1);
%       -Cr, 0];
% Ba = [B; 0];
% 
% desired_poles = [-3, -3.5, -4, -4.5, -5];
% Ka = place(Aa, Ba, desired_poles);
% 
% K = Ka(1:4);
% Ki = Ka(5);




