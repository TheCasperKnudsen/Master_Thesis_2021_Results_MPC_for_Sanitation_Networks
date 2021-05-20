clear all; clc;
load('Lab_Experimetn_SMPC_With_Realistic_Disturbance/Data/D_sim_ens');
load('Lab_Experimetn_SMPC_With_Realistic_Disturbance/Data/mean_disturbance');

deviation_set = zeros(30,size(mean_disturbance,2));

for i = 1:size(mean_disturbance,2)
    deviation_set(1:10,i) = abs(D_sim_ens(1:10,i) - mean_disturbance(1,i));
    deviation_set(21:30,i) = abs(D_sim_ens(21:30,i) - mean_disturbance(3,i));
end

total_ens_deviation = sum(deviation_set,2);

[M,max_I1] = max(total_ens_deviation(1:10));
[M,max_I2] = max(total_ens_deviation(21:30));
most_deviated_realization_indices = [max_I1;20+max_I2]

figure
subplot(2,1,1)
plot(mean_disturbance(1,:));
hold on;
plot(D_sim_ens(1,:));
hold on;
plot(D_sim_ens(most_deviated_realization_indices(1),:));
legend('Mean disturbance','Old realization', 'Most deviated realization')

subplot(2,1,2)
plot(mean_disturbance(3,:));
hold on;
plot(D_sim_ens(21,:));
hold on;
plot(D_sim_ens(most_deviated_realization_indices(2),:));
legend('Mean disturbance','Old realization', 'Most deviated realization')