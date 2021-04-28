clear all;

%% Add paths

addpath('Supporting_Functions')

% ************ Change to own Casadi path ************
addpath('C:\Users\Casper and Adis\Desktop\casadi-windows-matlabR2016a-v3.5.5')
% ***************************************************
import casadi.*


%% ============================================== MPC. setup ===================================
Hp = 24;                                % prediction horizon   
Hu = Hp;                                % control horizion
nS = 10;                                 % number of states
nT = 2;                                 % number of tanks
nP = 8;                                 % number of pipe sections
nU = 2;                                 % number of control inputs
nD = 2;
opti = casadi.Opti();                   % opti stack 
warmStartEnabler = 1;                   % warmstart for optimization
%% ============================================ Constraint limits ==============================
U_ub   = [8.3;15]/60;                      % input bounds
U_lb   = [3.4;6]/60;
dU_ub  = [4.5;4.5]/60;
dU_lb  = [-4.5;-4.5]/60;

Xt_1_ub  = 7.02;                          % state bounds tank
Xt_2_ub  = 6.43;  

Xt_lb  = 1.8;
Xp_ub  = 0.5;                           % state bounds pipes                          
Xp_lb  = -10;
% Combine into system bounds

X_ub   = [Xt_1_ub, Xp_ub*ones(1,nP), Xt_2_ub]'; 
X_lb   = [Xt_lb, Xp_lb*ones(1,nP), Xt_lb]'; 

%% ========================================= Optimization variables ============================
X  = opti.variable(nS,Hp+1);            % state - volume 
U  = opti.variable(nU,Hp);              % input - pumpflow 
deltaU = opti.variable(nU,Hp);
S  = opti.variable(nT,Hp);              % slack - overflow volume
S_ub = opti.variable(nT, Hu);           % slack - relaxing the chance constraint

%% ========================================= Optimization parameters ===========================
D  = opti.parameter(nD,Hp);             % disturbance - rain inflow
X0 = opti.parameter(nS);                % initial state - level
U0 = opti.parameter(nU);                % the previous control
T  = opti.parameter(1);                 % MPC model_level sampling time
Reference  = opti.parameter(nS,Hp);        % reference

sigma_X = opti.parameter(nT,Hp);
sigma_U = opti.parameter(nT,Hp);              %LQR gain

%% ====================================== System parameters ====================================
%p = [0.0344584980456826,0.0864650413052119,0.00653614397630376,-0.00280609998794716,0.0550243659248174];     %4 states
p = [0.0591715867309953,0.139652097300738,0.000840889029721471,-0.00505537833798277,0.200000000000000];
phi = [1/4.908738521234052,1/4.908738521234052];

%% =========================================== Objective =======================================
% Weights
Decreasing_cost = diag((nT*Hp):-1:1)*10000000;
sum_vector = zeros(nT * Hp,1)+1;
P = eye(nT * Hp,nT * Hp) * 100000000000 + Decreasing_cost;
Q = zeros(nS, nS);
Q(1,1) = 10;                                                               % cost of tank1 state
Q(nS,nS) = 10;                                                               % cost of tank2 state               
Q = kron(eye(Hp),Q);
R = eye(nU * Hp,nU * Hp) * 1000;

% Rearrange X and U
X_obj = vertcatComplete( X(:,1:end-1) - Reference);
deltaU_obj = vertcatComplete(deltaU);
U_obj = vertcatComplete(U);
S_obj = vertcatComplete(S);

% Objective function
objective = X_obj'*Q*X_obj + S_obj'* P * sum_vector + deltaU_obj'*R*deltaU_obj + 10000000*sum(sum(S_ub'));% + U_obj'*R*U_obj;
opti.minimize(objective);

%% ============================================ Dynamics =======================================

% function variables
dt = casadi.MX.sym('dt',1);             % sampling time 
x = casadi.MX.sym('x',nS);              % state
u = casadi.MX.sym('u',nU);              % input
d = casadi.MX.sym('d',nD);              % disturbance

% system matricies
A       = BuildA_MX(nS, p, phi, dt);                                           % builds two tank topology with nS-2 pipe sections
B       = BuildB_MX(nS, p, phi, dt);
Bd      = BuildBd_MX(nS,2,p,phi,dt);                                           % allows d to enter in tank1 and in pipe section 2
Delta   = BuildDelta_MX(nS, p, dt);

% function
system_dynamics = A*x + B*u + Bd*d + Delta;
% Discrete dynamics
F_system = casadi.Function('F_DW', {x, u, d, dt}, {system_dynamics}, {'x[k]', 'u[k]', 'd[k]', 'dt'}, {'x[k+1]'});

% make struct to get when MPC is run
A_num       = casadi.Function('eval_A',{dt},{A},{'dt'},{'A'});
B_num       = casadi.Function('eval_B',{dt},{B},{'dt'},{'B'});
Bd_num      = casadi.Function('eval_Bd',{dt},{Bd},{'dt'},{'Bd'});
Delta_num   = casadi.Function('eval_Delta',{dt},{Delta},{'dt'},{'Delta'});
sys = struct('A', A_num, 'B', B_num, 'Bd',Bd_num,'Delta',Delta_num,'F_system',F_system,'X_lb', X_lb, 'X_ub' ,X_ub, 'U_lb', U_lb, 'U_ub', U_ub);


%% ======================================== Constraints ========================================
% Initial state                             
opti.subject_to(X(:,1)==X0);           

% Defining control horizon.
for i=Hu+1:1:Hp
    opti.subject_to(U(:,i)==U(:,Hu))
end

% Dynamic constraints
for i=1:Hp                             
   opti.subject_to(X(:,i+1)==F_system(X(:,i), U(:,i) + S(:,i), D(:,i), T));
   if i == 1
       opti.subject_to(deltaU(:,i)==U(:,i) - U0)
   else
       opti.subject_to(deltaU(:,i)==U(:,i) - U(:,i-1));
   end
   opti.subject_to(dU_lb <= (U(:,i) - U(:,i-1)) <= dU_ub);                  % bounded slew rate
   opti.subject_to(X_lb(2:5)<=X(2:5,i)<=X_ub(2:5));  
end

for i = 1:1:nT
    opti.subject_to(S(i,:)>=zeros(1,Hp));                                   % slack variable is always positive - Vof >= 0
end

%% ================================= Add Chance constraints ===============================
% Precompute sigma_X for chance constraint, Open Loop MPC:
var_x_prev = casadi.MX.sym('xvp',nS,nS);     
var_D = casadi.MX.sym('vd',nD,nD);              
var_model = casadi.MX.sym('vm',nS,nS); 
var_U = casadi.MX.sym('vu',nU,nU);
lqr_K = casadi.MX.sym('lqr_k',nU,nS);

% function
var_x = (A-B*lqr_K)*var_x_prev*(A-B*lqr_K)' + Bd*var_D*Bd' + B*var_U*B' + var_model;
var_x_ol = (A)*var_x_prev*(A)' + Bd*var_D*Bd' + B*var_U*B' + var_model;
% discrete dynamics
F_variance = casadi.Function('F_var', {var_x_prev, var_D, var_model, var_U, lqr_K, dt}, {var_x}, {'vx[k]', 'vd', 'vm', 'vu', 'lqrK','dt'}, {'vx[k+1]'});
F_variance_ol = casadi.Function('F_var', {var_x_prev, var_D, var_model, var_U, dt}, {var_x_ol}, {'vx[k]', 'vd', 'vm', 'vu','dt'}, {'vx[k+1]'});

% add constraints
for i = 1:1:Hp
   opti.subject_to(X_lb(1)<=X(1,i)<=X_ub(1) + S_ub(1,i) - sqrt(sigma_X(1,i))*norminv(0.95));
   opti.subject_to(X_lb(6)<=X(6,i)<=X_ub(6) + S_ub(2,i) - sqrt(sigma_X(2,i))*norminv(0.95));
   opti.subject_to(zeros(nT,1) <= S_ub(:,i) <= sqrt(sigma_X(:,i))*norminv(0.95));                                 % Slack variable is always positive - Vof >= 0
end

for i = 1:1:Hu
    opti.subject_to(U_lb(1) <= U(1,i) <= U_ub(1) - sqrt(sigma_U(1,i))*norminv(0.95));
    opti.subject_to(U_lb(2) <= U(2,i) <= U_ub(2) - sqrt(sigma_U(2,i))*norminv(0.95));% bounded input  
end



%% ====================================== Solver settings ==================================
% opti.set_initial(X, 1);                                                    % first guess
%opti.set_initial(S, 0);
%opti.set_inivar_x_prev = casadi.MX.sym('x',nS,nS);tial(U, U_lb);

% Solver options
opts = struct;
opts.ipopt.print_level = 0;                                                     % print enabler to command line
opts.print_time = false;
opts.expand = true;                                                             % makes function evaluations faster
%opts.ipopt.hessian_approximation = 'limited-memory';
opts.ipopt.max_iter = 100;                                                      % max solver iteration
opti.solver('ipopt',opts);         

if warmStartEnabler == 1
    % Parametrized Open Loop Control problem with WARM START
    OCP = opti.to_function('OCP',{X0,U0,D,opti.lam_g,opti.x,T,Reference,sigma_X,sigma_U},{U,S,S_ub,opti.lam_g,opti.x},{'x0','u0','d','lam_g','x_init','dt','ref','sigma_x','sigma_u'},{'u_opt','s_opt','S_ub_opt','lam_g','x_init'});
elseif warmStartEnabler == 0
    % Parametrized Open Loop Control problem without WARM START 
    OCP = opti.to_function('OCP',{X0,U0,D,T,Reference,sigma_X,sigma_U},{U,S,S_ub},{'x0','u0','d','dt','ref','sigma_x','sigma_u'},{'u_opt','s_opt','S_ub_opt'});
end

%load('Lab_Experimetn_SMPC_With _Realistic_Disturbance\Data\X_ref_sim.mat');
load('Lab_Experimetn_SMPC_With _Realistic_Disturbance\Data\D_sim_ens.mat');
load('Lab_Experimetn_SMPC_With _Realistic_Disturbance\Data\mean_disturbance.mat');
load('Lab_Experimetn_SMPC_With _Realistic_Disturbance\Data\average_dist_variance_Hp.mat');

D_sim = [D_sim_ens(1,:);zeros(1,size(D_sim_ens,2)); D_sim_ens(21,:)];
clear D_sim_ens;


