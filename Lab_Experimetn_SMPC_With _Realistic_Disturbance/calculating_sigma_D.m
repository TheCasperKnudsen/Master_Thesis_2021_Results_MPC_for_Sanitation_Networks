clear all; clc;
load('Lab_Experimetn_SMPC_With _Realistic_Disturbance/Data/D_sim_ens');

%% ======== Setup for MPC =========
D_sim_ens = D_sim_ens/60;
Hp = 24;
dT = 5;     %seconds
simulink_frequency = 2;   %1/s

MPC_prediction_horizon_in_steps = Hp*dT*simulink_frequency;

%% ===== Calculating variance of disturbances =====
ensambles_T1 = [D_sim_ens(1:10,:)];
mean_disturbance_T1 = mean(ensambles_T1);
variance_disturbance_T1 = var(ensambles_T1);
variance_prediction_T1 = zeros(1,size(D_sim_ens,2)-MPC_prediction_horizon_in_steps);
for i = 1:size(D_sim_ens,2)-MPC_prediction_horizon_in_steps
    variance_prediction_T1(i) = mean(variance_disturbance_T1(i:i+MPC_prediction_horizon_in_steps));
end

ensambles_pipe = [D_sim_ens(21:30,:)];
mean_disturbance_pipe = mean(ensambles_pipe);
variance_disturbance_pipe = var(ensambles_pipe);
variance_prediction_pipe = zeros(1,size(D_sim_ens,2)-MPC_prediction_horizon_in_steps);
for i = 1:size(D_sim_ens,2)-MPC_prediction_horizon_in_steps
    variance_prediction_pipe(i) = mean(variance_disturbance_pipe(i:i+MPC_prediction_horizon_in_steps));
end

mean_disturbance = [mean_disturbance_T1;D_sim_ens(11,:);mean_disturbance_pipe]*60;
average_dist_variance_Hp = [variance_prediction_T1; variance_prediction_pipe];

save('Lab_Experimetn_SMPC_With _Realistic_Disturbance/Data/mean_disturbance.mat','mean_disturbance');
save('Lab_Experimetn_SMPC_With _Realistic_Disturbance/Data/average_dist_variance_Hp.mat','average_dist_variance_Hp');
%%
figure
for i = 1:10
plot(ensambles_T1(i,:),'--');
hold on;
end
plot(mean_disturbance_T1);
hold on;
plot(variance_disturbance_T1);
hold on;
plot(variance_prediction_T1);
legend('Realization 1','Realization 2', 'Mean disturbance', 'Dist Variance','Dist prediction avrg. variance');
figure
plot(mean_disturbance(1,:));
hold on;
plot(mean_disturbance(3,:));

figure
plot(average_dist_variance_Hp(1,:));
hold on;
plot(average_dist_variance_Hp(2,:));

