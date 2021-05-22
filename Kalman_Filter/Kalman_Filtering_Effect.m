%% ================================================ Run Kalman =============================================
clear all
clc
load('System_Identification/GravityPipe_Parameter_Estimation/results/Lat_inflow_4_aug_states_28-Apr-2021.mat');
%Lab_lateral_inflow_0_aug_states_21-Apr-2021.mat
X = zeros(size(h,1)*2,size(h,2));

for time = 1:1:size(h,2)
    SystemMeas =h(:,time);
    ControlInput = Q(1,time);
    Disturbance = Q(4,time);
    [EstimatedX] = Kalman_Filter_Pipes_4Aug(ControlInput,Disturbance,SystemMeas);
    X(:,time) = EstimatedX;
end


figure
k = 1;
for i = 1:8 
    subplot(8,1,i)
    plot(X(i,:));
    
    if i == 2 | i == 4| i == 5 | i ==7
        hold on;
        plot(h(k,:),'r');
        k = k+1;
    end
    
    
    if i == 8
        axis([0,40000,0,0.2])
        clear k
    else 
        axis([0,40000,0,0.20])
    end
end