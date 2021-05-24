clear all
clc
load('System_Identification/GravityPipe_Parameter_Estimation/results/Lat_inflow_4_aug_states_28-Apr-2021.mat','estimatedParameters');
parameters8 = estimatedParameters;

maxDelta8pipe(1) = 1/(parameters8(2)+parameters8(3));
maxDelta8pipe(2) = 1/(parameters8(5)+ parameters8(3));

clear estimatedParameters
load('System_Identification/GravityPipe_Parameter_Estimation/results/Lab_lateral_inflow_0_aug_states_21-Apr-2021.mat','estimatedParameters');
parameters4 = estimatedParameters;
maxDelta4pipe(1) = 1/(parameters4(2)+parameters4(3));
maxDelta4pipe(2) = 1/(parameters4(5)+ parameters4(3));

