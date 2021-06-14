clear all;

%% Add paths
addpath('Supporting_Functions')

% ************ Change to own Casadi path ************
addpath('C:\Users\Casper and Adis\Desktop\casadi-windows-matlabR2016a-v3.5.5')
% ***************************************************
import casadi.*


%% ============================================== MPC. setup ===================================
Hp = 108;                                % prediction horizon   
Hu = Hp;                                % control horizion
nT = 2;                                 % number of tanks
nP = 8;                                 % number of pipe sections
nS = nT + nP;                           % number of states
nU = 2;                                 % number of control inputs
nD = 2;                                 % number of disturbance inputs
opti = casadi.Opti();                   % opti stack 
warmStartEnabler = 1;                   % warmstart for optimization
%% ============================================ Constraint limits ==============================
% Input bounds - Devide by 60 to get L/sec
U_ub   = [8.3;13.2]/60;                   
U_lb   = [3.4;6]/60;
dU_ub  = [0.5;0.0667]/60;
dU_lb  = [-0.5;-0.0667]/60;
% State bounds Tank
Xt_1_ub  = 6.99;                          
Xt_2_ub  = 6.50;  
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
T  = opti.parameter(1);                 % MPC model_level sampling time
Ub_adjust = opti.parameter(nT);
Reference  = opti.parameter(nS,Hp);        % reference

%% ====================================== System parameters ====================================
p = [0.0578290979772847,0.137832091474361,0.000100000000000000,-0.00513392718034462,0.100000000000000];
phi = [1/4.908738521234052,1/4.908738521234052];

%% =========================================== Objective =======================================
% Weights
tankWight = zeros((nT*Hp),1);
tankWight(2:2:end) = 1;
tankWight(1:2) = tankWight(1:2)  + 10;
tankWight = tankWight + ((nT*Hp):-1:1)';
Decreasing_cost = diag(tankWight) * 1000;

sum_vector = zeros(nT * Hp,1)+1;
P = eye(nT * Hp,nT * Hp) + Decreasing_cost;
Q = zeros(nS, nS);
Q(1,1) = 10;                                                               % cost of tank1 state
Q(nS,nS) = 10;                                                             % cost of tank2 state               
Q = kron(eye(Hp),Q);
R = zeros(nU, nU);
R(nU,nU) = 50;                                                                       
R = kron(eye(Hp),R);

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
dt = casadi.MX.sym('dt',1);             % sampling time 
x = casadi.MX.sym('x',nS);              % state
u = casadi.MX.sym('u',nU);              % input
d = casadi.MX.sym('d',nD);              % disturbance
uof = casadi.MX.sym('uof',nT); 

A       = BuildA_MX(nS, p, phi, dt);                                           % builds two tank topology with nS-2 pipe sections
B       = BuildB_MX(nS, p, phi, dt);
Bd      = BuildBd_MX(nS,2,p,phi,dt);                                           % allows d to enter in tank1 and in pipe section 2
Delta   = BuildDelta_MX(nS, p, dt);
Bof     = BuildBof_MX(nS,phi,dt);

% function
system_dynamics = A*x + B*u + Bd*d + Delta + Bof * uof;
% Discrete dynamics
F_system = casadi.Function('F_DW', {x, u, d, uof, dt}, {system_dynamics}, {'x[k]', 'u[k]', 'd[k]', 'uof[k]', 'dt'}, {'x[k+1]'});

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
for i=Hu+1:1:Hu
    opti.subject_to(U(:,i)==U(:,Hu))
end

% Dynamic constraints
for i=1:Hp                             
   opti.subject_to(X(:,i+1)==F_system(X(:,i), U(:,i), D(:,i),S(:,i), T));
   opti.subject_to(X_lb<=X(:,i)<=X_ub);  
   
   if i == 1
       opti.subject_to(deltaU(:,i)==U(:,i) - U0)
   else
       opti.subject_to(deltaU(:,i)==U(:,i) - U(:,i-1));
   end
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
opts.ipopt.print_level = 0;
opts.ipopt.max_iter = 100;
opti.solver('ipopt',opts);         

if warmStartEnabler == 1
    % Parametrized Open Loop Control problem with WARM START
    OCP = opti.to_function('OCP',{X0,U0,D,opti.lam_g,opti.x,T,Reference,Ub_adjust},{U,S,opti.lam_g,opti.x,objective},{'x0','u0','d','lam_g','x_init','dt','ref','ub_adjustment'},{'u_opt','s_opt','lam_g','x_init','Obj'});
elseif warmStartEnabler == 0
    % Parametrized Open Loop Control problem without WARM START 
    OCP = opti.to_function('OCP',{X0,U0,D,T,Reference,Ub_adjust},{U,S,objective},{'x0','u0','d','dt','ref','ub_adjustment'},{'u_opt','s_opt','Obj'});
end

%load('Lab_Experimetn_SMPC_With _Realistic_Disturbance\Data\X_ref_sim.mat');

load('Distrubance_Data\D_sim_ens.mat');
load('Distrubance_Data\mean_disturbance.mat');
load('Distrubance_Data\average_dist_variance_Hp.mat');

D_sim = [D_sim_ens(6,:);zeros(1,size(D_sim_ens,2)); D_sim_ens(24,:)];
clear D_sim_ens;


