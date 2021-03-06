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
nT = 2;                                 % number of tanks
nP = 8;                                 % number of pipe sections
nS = nT + nP;                           % number of states
nU = 2;                                 % number of control inputs
nD = 2;                                 % number of disturbance inputs
opti = casadi.Opti();                   % opti stack 
Ts = 5;                                 % MPC period
tau = 2;                                % MPC calculation time
warmStartEnabler = 1;                   % warmstart for optimization
%% ============================================ Constraint limits ==============================
% Input bounds - Devide by 60 to get L/sec
U_ub   = [8.3;15]/60;                   
U_lb   = [3.4;6]/60;
dU_ub  = [4.5;4.5]/60;
dU_lb  = [-4.5;-4.5]/60;
% State bounds Tank
Xt_1_ub  = 7.02;                          
Xt_2_ub  = 6.43;  
% State bounds pipes
Xt_lb  = 1.5;
Xp_ub  = 0.5;                                                     
Xp_lb  = -10;
% Combine into state bounds
X_ub   = [Xt_1_ub, Xp_ub*ones(1,nP), Xt_2_ub]'; 
X_lb   = [Xt_lb, Xp_lb*ones(1,nP), Xt_lb]'; 

%% ========================================= Optimization variables ============================
X  = opti.variable(nS,Hp+1);            % state - volume 
U  = opti.variable(nU,Hp);              % input - pumpflow 
deltaU = opti.variable(nU,Hp);
S  = opti.variable(nT,Hp);              % slack - overflow volume

%% ========================================= Optimization parameters ===========================
D  = opti.parameter(nD,Hp);             % disturbance - rain inflow
X0 = opti.parameter(nS);                % initial state - level
U0 = opti.parameter(nU);                % the previous control
S0 = opti.parameter(nT);
D0 = opti.parameter(nD);
T  = opti.parameter(1);                 % MPC model_level sampling time
Reference  = opti.parameter(nS,Hp);        % reference

%% ====================================== System parameters ====================================
p = [0.0578290979772847,0.137832091474361,0.000100000000000000,-0.00513392718034462,0.100000000000000];
phi = [1/4.908738521234052,1/4.908738521234052];

%% =========================================== Objective =======================================
% Weights
Decreasing_cost = diag((nT*Hp):-1:1)*1000000000;
sum_vector = zeros(nT * Hp,1)+1;
P = eye(nT * Hp,nT * Hp) + Decreasing_cost;
Q = zeros(nS, nS);
Q(1,1) = 1;                                                                 % cost of tank1 state
Q(nS,nS) = 1;                                                               % cost of tank2 state               
Q = kron(eye(Hp),Q);
R = eye(nU * Hp,nU * Hp) * 0.1;

% Rearrange X and U
X_obj = vertcatComplete( X(:,1:end-1) - Reference);
deltaU_obj = vertcatComplete(deltaU);
U_obj = vertcatComplete(U);
S_obj = vertcatComplete(S);

% Objective function
objective = X_obj'*Q*X_obj + S_obj'* P * sum_vector + deltaU_obj'*R*deltaU_obj;% + U_obj'*R*U_obj;
opti.minimize(objective);

%% ============================================ Dynamics =======================================

% function variables
x = casadi.MX.sym('x',nS);              % state
uk = casadi.MX.sym('uk',nU);              % input
ukm1 = casadi.MX.sym('ukm1',nU);              % old input
dk = casadi.MX.sym('dk',nD);              % disturbance
dkm1 = casadi.MX.sym('dkm1',nD);              % disturbance
uofk = casadi.MX.sym('uofk',nT); 
uofkm1 = casadi.MX.sym('uofkm1',nT); 

A       = BuildAContinues(nS, p, phi);                                           % builds two tank topology with nS-2 pipe sections
Ad      = BuildA(nS, p, phi, Ts);
B       = BuildBContinues(nS, p, phi);
Bd      = BuildBdContinues(nS,2,p,phi);                                          % allows d to enter in tank1 and in pipe section 2
Bof     = BuildBofContinues(nS,phi);
Delta   = BuildDeltaContinues(nS, p);

[Bk,Bkm1]       = ComputationTimeCompensationB(A,B,Ts,tau);
[Bdk,Bdkm1]       = ComputationTimeCompensationB(A,Bd,Ts,tau);
[Bofk,Bofkm1]   = ComputationTimeCompensationB(A,Bof,Ts,tau);
[Deltak,Deltakm1]       = ComputationTimeCompensationDelta(A,Delta,Ts,tau);
newDelta = Deltak+Deltakm1;


% function
system_dynamics_compensated = Ad*x + Bk*uk + Bkm1*ukm1 + Bofk*uofk + Bofkm1*uofkm1 + Bdk*dk + Bdkm1*dkm1 + newDelta;
% Discrete dynamics
F_system = casadi.Function('F_DW', {x, uk, ukm1, dk, dkm1, uofk, uofkm1}, {system_dynamics_compensated}, {'x[k]', 'u[k]','u[k-1]', 'd[k]','d[k-1]', 'uof[k]','uof[k-1]'}, {'x[k+1]'});

% make struct to get when MPC is run
sys = struct('Ts',Ts);


%% ======================================== Constraints ========================================
% Initial state                             
opti.subject_to(X(:,1)==X0);           

% Defining control horizon.
for i=Hu+1:1:Hu
    opti.subject_to(U(:,i)==U(:,Hu))
end

% Dynamic constraints
for i=1:Hp
   if i == 1
       opti.subject_to(deltaU(:,i)==U(:,i) - U0)
       opti.subject_to(X(:,i+1)==F_system(X(:,i), U(:,i), U0, D(:,i), D0, S(:,i), S0));
   else
       opti.subject_to(deltaU(:,i)==U(:,i) - U(:,i-1));
       opti.subject_to(X(:,i+1)==F_system(X(:,i), U(:,i), U(:,i-1), D(:,i), D(:,i-1), S(:,i), S(:,i-1)));
   end
   opti.subject_to(X_lb<=X(:,i)<=X_ub);  
   
   opti.subject_to(dU_lb <= (U(:,i) - U(:,i-1)) <= dU_ub);                  % bounded slew rate
end

for i = 1:1:nT
    opti.subject_to(S(i,:)>= 0);                                            % slack variable is always positive - Vof >= 0
end

for i = 1:1:Hu
    opti.subject_to(U_lb(1) <= U(1,i) <= U_ub(1));
    opti.subject_to(U_lb(2) <= U(2,i) <= U_ub(2));
end



%% ====================================== Solver settings ==================================
% opti.set_initial(X, 1);                                                    % first guess
%opti.set_initial(S, 0);
%opti.set_inivar_x_prev = casadi.MX.sym('x',nS,nS);tial(U, U_lb);

% Solver options
opts = struct;
% opts.ipopt.print_level = 0;                                                     % print enabler to command line
% opts.print_time = false;
opts.expand = true;                                                             % makes function evaluations faster
%opts.ipopt.hessian_approximation = 'limited-memory';
opts.ipopt.max_iter = 100;                                                      % max solver iteration
opti.solver('ipopt',opts);         

if warmStartEnabler == 1
    % Parametrized Open Loop Control problem with WARM START
    OCP = opti.to_function('OCP',{X0,U0,S0,D,D0,opti.lam_g,opti.x,Reference},{U,S,opti.lam_g,opti.x},{'x0','u0','s0','d','d0','lam_g','x_init','ref'},{'u_opt','s_opt','lam_g','x_init'});
elseif warmStartEnabler == 0
    % Parametrized Open Loop Control problem without WARM START 
    OCP = opti.to_function('OCP',{X0,U0,S0,D,D0,Reference},{U,S,S_ub},{'x0','u0','s0','d','d0','ref'},{'u_opt','s_opt'});
end

%load('Lab_Experimetn_SMPC_With _Realistic_Disturbance\Data\X_ref_sim.mat');

load('Distrubance_Data\D_sim_ens.mat');
load('Distrubance_Data\mean_disturbance.mat');
load('Distrubance_Data\average_dist_variance_Hp.mat');

D_sim = [D_sim_ens(1,:);zeros(1,size(D_sim_ens,2)); D_sim_ens(21,:)];
clear D_sim_ens;


