%% ================================================ Run Kalman =============================================
clear all
clc
load('System_Identification/GravityPipe_Parameter_Estimation/results/Lab_lateral_inflow_0_aug_states_21-Apr-2021.mat');
%Lab_lateral_inflow_0_aug_states_21-Apr-2021.mat
%X = zeros(size(h,1)*2,size(h,2));

for time = 1:1:size(h,2)
    SystemMeas =h(2:2:4,time);
    ControlInput = Q(1,time);
    Disturbance = Q(4,time);
    [EstimatedX,ErrorCo,KalmanGain] = Kalman_Filter_Pipes_4Aug_demo(ControlInput,Disturbance,SystemMeas);
    X(:,time) = EstimatedX;
end

ErrorCo
KalmanGain
figure
k = 1;
time = 0.5:0.5:size(h,2)/2;
Font_scale = 14;
for i = 1:4 
    subplot(4,1,i)
    plot(time,X(i,:),'r');
    hold on;
    plot(time,h(i,:),'b');

    
    y_lab = ylabel(['$h_{p' num2str(i) '}$ [dm]'],'interpreter','latex');
    set(y_lab, 'FontSize', Font_scale);
    if i == 8
        axis([0,4000,0,0.2])
        clear k
        x_lab = xlabel(['Time [s]'],'interpreter','latex');
        set(x_lab, 'FontSize', Font_scale);
    elseif i == 1
        legend('Estimated States','Measured States');
        axis([500,3000,0,0.20])
    else
        axis([500,3000,0,0.20])
    end
    set(gca,'FontSize',Font_scale);
end