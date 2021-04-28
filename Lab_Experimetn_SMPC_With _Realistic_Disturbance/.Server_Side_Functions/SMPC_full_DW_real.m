function [output]  = SMPC_full_DW(X0,time)
% define persistent variables
eml.extrinsic('evalin');
persistent x_init;
persistent lam_g;
persistent OCP;
persistent Hp;
persistent warmStartEnabler;
persistent mean_disturbance;
persistent average_dist_variance_Hp;
persistent F_variance;
persistent X_ref;
persistent U0;
persistent sigma_x;
persistent sigma_u;
persistent X_pre;
persistent sys;
persistent K;
persistent measCovPipe;
persistent modelCovPipe;
% and others
dT = 10;                 % Sample time in minutes
simulink_frequency = 2;  % Sampling frequency in seconds
time = int64(round(time));

if isempty(lam_g) 
    % init persistent variables
    lam_g = 1;
    x_init = 0.001;
    
    % initialize MPC
    U0 = [3;4.5];
    X_pre = X0/100;
    
    % get optimization problem and warmStartEnabler
    OCP = evalin('base','OCP');
    Hp = evalin('base','Hp');
    mean_disturbance = evalin('base','mean_disturbance');
    mean_disturbance = mean_disturbance(1:2:3,:);
    average_dist_variance_Hp = evalin('base', 'average_dist_variance_Hp');
    warmStartEnabler = evalin('base','warmStartEnabler');
    
    %Temp
    load('System_Identification/Noise_Identification/Results/model_and_messurement_cov_matrices.mat','measCovPipe','modelCovPipe');
    modelCovPipe = blkdiag(0.001,BuildModelCovPipe4Aug(modelCovPipe),0.001);
    modelCovPipe = diag(diag(modelCovPipe));
    make_LQR_real      % Calculate LQR
    
    F_variance = evalin('base','F_variance');
end
% initialize CC tube controller

make_sigma_X_real  % Precompute sigma_X for chance constraint, Open Loop MPC

X0 = X0/100;                                                               % Unit convertion from mm to dm


%Create forcast from disturbance reference
disturbance = zeros(2,Hp);
reference = ones(10,Hp)*2;
for i=0:1:Hp-1
    start_index = time+1+i*dT*simulink_frequency;
    end_index = start_index+dT*simulink_frequency-1;
    disturbance(:,i+1) = mean(mean_disturbance(:,start_index:end_index),2)/60;
end




% run openloop MPC
if warmStartEnabler == 1
    % Parametrized Open Loop Control problem with WARM START
    [u , S, S_ub, lam_g, x_init] = (OCP(X0,U0,disturbance, lam_g, x_init, dT,reference,sigma_x,sigma_u));
elseif warmStartEnabler == 0
    % Parametrized Open Loop Control problem without WARM START 
    [u , S, S_ub] = (OCP(X0 ,U0, disturbance, dT, reference,sigma_x,sigma_u));
end



% create outputs
u_full = full(u);
S_full = full(S);
S_ub_full = full(S_ub);
lqr_contribution =min(1/60*ones(2,1), max(-1/60*ones(2,1),  K*(X0-X_pre)));

output = [min(sys.U_ub+1, max(sys.U_lb, u_full(:,1) - lqr_contribution)) ; S_full(:,1)]*60;           % Saturate the outputs
output = [output; 2*ones(2,1)*100; S_ub_full(:,1)];

% Set vairables for next iteration
U0 = u_full(:,1);
X_pre = full(sys.F_system(X0, u_full(:,1), disturbance(:,1), dT));
for i =1:9:10
    if X_pre(i) > sys.X_ub(i)
        X_pre(i) = sys.X_ub(i);
    elseif X_pre(i) < sys.X_lb(i)
        X_pre(i) = sys.X_lb(i);
    end
end
end
