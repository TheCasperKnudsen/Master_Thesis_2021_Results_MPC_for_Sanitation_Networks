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
%opti=Opti('conic');                     % Conic stack 
warmStartEnabler = 1;                   % warmstart for optimization
%% ============================================ Constraint limits ==============================
% Input bounds - Devide by 60 to get L/sec
U_ub   = [8;13.2]/60;                   
U_lb   = [3.4;6]/60;
dU_ub  = [0.5;0.0667]/60;
dU_lb  = [-0.5;-0.0667]/60;

% State bounds Tank
Xt_1_ub  = 6.99;                          
Xt_2_ub  = 6.50;            % New tank2 upper bound   
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
S_ub = opti.variable(nT, Hu);           % slack - relaxing the chance constraint

%% ========================================= Optimization parameters ===========================
D  = opti.parameter(nD,Hp);             % disturbance - rain inflow
X0 = opti.parameter(nS);                % initial state - level
U0 = opti.parameter(nU);                % the previous control
T  = opti.parameter(1);                 % MPC model_level sampling time
Ub_adjust = opti.parameter(nT);         % Used to make "hypothetical" upper bound
Reference  = opti.parameter(nS,Hp);        % reference

sigma_X = opti.parameter(nT,Hp);
sigma_U = opti.parameter(nT,Hp);              %LQR gain

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
objective = X_obj'*Q*X_obj + S_obj'* P * sum_vector + deltaU_obj'*R*deltaU_obj + 100*sum(sum(S_ub'));% + U_obj'*R*U_obj;
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
for i=Hu+1:1:Hp
    opti.subject_to(U(:,i)==U(:,Hu))
end

% Dynamic constraints
for i=1:Hp                             
   opti.subject_to(X(:,i+1)==F_system(X(:,i), U(:,i), D(:,i), S(:,i), T));
   if i == 1
       opti.subject_to(deltaU(:,i)==U(:,i) - U0)
   else
       opti.subject_to(deltaU(:,i)==U(:,i) - U(:,i-1));
   end
   opti.subject_to(dU_lb <= (U(:,i) - U(:,i-1)) <= dU_ub);                  % bounded slew rate
   opti.subject_to(X_lb(2:nS-1)<=X(2:nS-1,i)<=X_ub(2:nS-1));  
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
var_x = A*var_x_prev*A' + Bd*var_D*Bd' + B*var_U*B' + var_model;
%var_x_ol = (A)*var_x_prev*(A)' + Bd*var_D*Bd' + B*var_U*B' + var_model;
% discrete dynamics
F_variance = casadi.Function('F_var', {var_x_prev, var_D, var_model, var_U, lqr_K, dt}, {var_x}, {'vx[k]', 'vd', 'vm', 'vu', 'lqrK','dt'}, {'vx[k+1]'});
%F_variance_ol = casadi.Function('F_var', {var_x_prev, var_D, var_model, var_U, dt}, {var_x_ol}, {'vx[k]', 'vd', 'vm', 'vu','dt'}, {'vx[k+1]'});

% add constraints
for i = 1:1:Hp
   opti.subject_to(X_lb(1)<= X(1,i) <=X_ub(1) + S_ub(1,i) - sqrt(sigma_X(1,i))*norminv(0.95));
   opti.subject_to(X_lb(nS)<=X(nS,i)<=X_ub(nS) + S_ub(2,i) - sqrt(sigma_X(2,i))*norminv(0.95));
   if i == 1
        opti.subject_to(zeros(nT,1) <= S_ub(:,i) <= sqrt(sigma_X(:,i))*norminv(0.95) + Ub_adjust);
   else
        opti.subject_to(zeros(nT,1) <= S_ub(:,i) <= sqrt(sigma_X(:,i))*norminv(0.95));     % Slack variable is always positive - Vof >= 0
   end
end

for i = 1:1:Hu
    opti.subject_to(U_lb(1) <= U(1,i) <= U_ub(1));
    opti.subject_to(U_lb(2) <= U(2,i) <= U_ub(2));% bounded input  
end



%% ====================================== Solver settings ==================================
% opti.set_initial(X, 1);                                                    % first guess
%opti.set_initial(S, 0);
%opti.set_inivar_x_prev = casadi.MX.sym('x',nS,nS);tial(U, U_lb);

% Solver options
opts = struct;                                                     % print enabler to command line
%opts.print_time = false;
opts.expand = true;    

% Ipopsolver 
opts.ipopt.print_level = 0;
opts.ipopt.max_iter = 100;
opti.solver('ipopt',opts);         

% opts.qpsol = 'qrqp';     
% opti.solver('sqpmethod',opts);
% opts.error_on_fail = 0;


if warmStartEnabler == 1
    % Parametrized Open Loop Control problem with WARM START
    OCP = opti.to_function('OCP',{X0,U0,D,opti.lam_g,opti.x,T,Reference,Ub_adjust, sigma_X,sigma_U},{U,S,S_ub,opti.lam_g,opti.x,objective},{'x0','u0','d','lam_g','x_init','dt','ref','ub_adjustment','sigma_x','sigma_u'},{'u_opt','s_opt','S_ub_opt','lam_g','x_init','Obj'});
elseif warmStartEnabler == 0
    % Parametrized Open Loop Control problem without WARM START 
    OCP = opti.to_function('OCP',{X0,U0,D,T,Reference,Ub_adjust, sigma_X,sigma_U},{U,S,S_ub,objective},{'x0','u0','d','dt','ref','ub_adjustment','sigma_x','sigma_u'},{'u_opt','s_opt','S_ub_opt','Obj'});
end

%load('Lab_Experimetn_SMPC_With _Realistic_Disturbance\Data\X_ref_sim.mat');

load('Distrubance_Data\D_sim_ens.mat');
load('Distrubance_Data\mean_disturbance.mat');
load('Distrubance_Data\average_dist_variance_Hp.mat');

D_sim = [D_sim_ens(1,:);zeros(1,size(D_sim_ens,2)); D_sim_ens(21,:)];
%clear D_sim_ens;


