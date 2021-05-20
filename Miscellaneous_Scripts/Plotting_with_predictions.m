clear all;
%close all;
clc;


% while ~exist('data','var')
%     [dataName, filePath] = uigetfile('*.mat','Select data structure');
%     fullPath = fullfile(filePath, dataName);
% 
%  
%     if exist(fullPath, 'file')
%         loaded = load(fullPath);
%         try
%             data = loaded.ans.Data';
%         catch
%             warning('Use the propper data file!');
%         end
%         
%         if ~exist('data','var')
%             warningMessage = sprintf('Warning: wrong file:\n%s\n%s', fullPath, 'Choose simulink generated file!');
%             uiwait(errordlg(warningMessage));
%         end
%     else
%         warningMessage = sprintf('Warning: mat file does not exist:\n%s', fullPath);
%         uiwait(errordlg(warningMessage));
%         return;
%     end
% end

SMPC_init_DW_real

load('Lab_Experimetn_SMPC_With_Realistic_Disturbance\results\17.5.2021\SMPC_DW_Realistic.mat');
data = ans.Data';
clear ans;
predictions = load('Lab_Experimetn_SMPC_With_Realistic_Disturbance\results\17.5.2021\Saved_predictions17-May-2021.mat').out;
% ===== Predictions structure =====
Control_input_pump_1 = predictions(1:6:size(predictions,1),:);
Control_input_pump_2 = predictions(2:6:size(predictions,1),:);
Overflow_1 = predictions(3:6:size(predictions,1),:);
Overflow_2 = predictions(4:6:size(predictions,1),:);
Tightening_1 = predictions(5:6:size(predictions,1),:);
Tightening_2 = predictions(6:6:size(predictions,1),:);

X_previous = zeros(10,5);
U_previous = zeros(2,5);
Overflow_previous = zeros(2,5);

for i=1:1:size(data,2)
    if ~isnan(data(1:5,i))
        startDataIndex = i;
        break;
    end
end
endDataIndex = size(data,2);
%%
load('.\Lab_Experimetn_SMPC_With_Realistic_Disturbance\Data\D_sim_ens.mat');
D_sim = D_sim_ens(1:20:21,:);
if size(data,1) < 22
    data(22,:) = 3*100;
    data(23,:) = 3*100;
end
%% ================== Plotting =======================
pump1_aux_ref = 16;
pump1_aux_flow = 13;
pump2_aux_ref = 17;
pump2_aux_flow = 12;
tank1_ref = 22;
tank1_mes = 1;
tank2_ref = 23;
tank2_mes = 7;
pump1_ref = 14;
pump1_flow = 8;
pump2_ref = 15;
pump2_flow = 9;
pipe_meas = 2:5;

figure

ax(1) = subplot(3,2,1);
plot(data(pump1_aux_ref,startDataIndex:endDataIndex));
hold on
plot(data(pump1_aux_flow,startDataIndex:endDataIndex));
leg = legend('$d_{1}$ reference', '$d_{1}$ flow');
set(leg,'Interpreter','latex');
xlim([0, endDataIndex-1]);
title('Disturbance on tank 1','interpreter','latex')
%
ax(2) = subplot(3,2,2);
plot(data(pump2_aux_ref,startDataIndex:endDataIndex));
hold on
plot(data(pump2_aux_flow,startDataIndex:endDataIndex));
leg = legend('$d_{2}$ reference', '$d_{2}$ flow');
set(leg,'Interpreter','latex');
xlim([0, endDataIndex-1]);
title('Disturbance in the middle of the pipe','interpreter','latex')
%
ax(3) = subplot(3,2,3);
plot(data(tank1_ref,startDataIndex:endDataIndex));
hold on
plot(data(tank1_mes,startDataIndex:endDataIndex));
hold on
plot(702*ones(1,endDataIndex-startDataIndex+1),'g--');
hold on
plot(150*ones(1,endDataIndex-startDataIndex+1),'g--');
leg = legend('$T_{1}$ reference', '$T_{1}$ level');
set(leg,'Interpreter','latex');
xlim([0, endDataIndex-1]);
title('Tank 1 level','interpreter','latex')
%
ax(4) = subplot(3,2,4);
plot(data(tank2_ref,startDataIndex:endDataIndex));
hold on
plot(data(tank2_mes,startDataIndex:endDataIndex));
hold on
plot(670*ones(1,endDataIndex-startDataIndex+1),'g--');
hold on
plot(150*ones(1,endDataIndex-startDataIndex+1),'g--');
leg = legend('$T_{2}$ reference', '$T_{2}$ level');
set(leg,'Interpreter','latex');
xlim([0, endDataIndex-1]);
title('Tank 2 level','interpreter','latex')
%
ax(5) = subplot(3,2,5);
plot(data(pump1_ref,startDataIndex:endDataIndex));
hold on
plot(data(pump1_flow,startDataIndex:endDataIndex));
leg = legend('$q_{1}$ reference', '$q_{1}$ flow');
set(leg,'Interpreter','latex');
xlim([0, endDataIndex-1]);
title('Pump 1','interpreter','latex')
%
ax(6) = subplot(3,2,6);
plot(data(pump2_ref,startDataIndex:endDataIndex));
hold on
plot(data(pump2_flow,startDataIndex:endDataIndex));
leg = legend('$q_{2}$ reference', '$q_{2}$ flow');
set(leg,'Interpreter','latex');
xlim([0, endDataIndex-1]);
title('Pump 2','interpreter','latex')
linkaxes(ax, 'x')

%% ======= Run Kalman for the entire experiment ========
X_zeros = zeros(8,size(data,2));
meas = data(pipe_meas,:);
meas(meas > 50 | meas <= 0) = 0.01;
for i = 1:size(data,2)
    X_zeros(:,i) = Kalman_Filter_Pipes_4Aug(data(pump1_flow,i), data(pump2_aux_flow,i),meas(:,i));
end

%% ======= Plotting the predictions =========

time = 22110;
if mod(time,10) ~= 0 || time < 40
    warningMessage = sprintf('Warning: time has to be a factor of 10 and >= 40');
    uiwait(errordlg(warningMessage));
    return;
end
MPC_run_step = (time - 40)/10 + 1
X0 = [data(tank1_mes,time+1); X_zeros(:,time+1); data(tank2_mes,time+1)];
Control_input_pumps = [Control_input_pump_1(MPC_run_step,:);Control_input_pump_2(MPC_run_step,:)];
Overflow = [Overflow_1(MPC_run_step,:);Overflow_2(MPC_run_step,:)];

% Defining time variables for plotting and disturbance selection
dT = 5;  %s
simulink_frequency = 2;

% Disturbance selection:
disturbance = zeros(2,Hp);
for i=0:1:Hp-1
    start_index = time+1+i*dT*simulink_frequency;
    end_index = start_index+dT*simulink_frequency-1;
    disturbance(:,i+1) = mean(mean_disturbance(1:2:3,start_index:end_index),2)/60;
end

% MPC prediction:
X_prediction = zeros(nS, Hp+1);
X_prediction(:,1) = X0/100;
for j=1:1:Hp
    X_prediction(:,j+1) = full(F_system(X_prediction(:,j), Control_input_pumps(:,j), disturbance(:,j), Overflow(:,j), dT ));
end
X_prediction = X_prediction*100;
Control_input_pumps = Control_input_pumps*60;
subplot(3,2,1)
plot(time:10:time+(Hp-1)*10,disturbance(1,:)*60,'k','LineWidth',1.75)
subplot(3,2,3)
plot(time:10:time+Hp*10,X_prediction(1,:),'k','LineWidth',1.75);
hold on;
subplot(3,2,5)
stairs(time:10:time+(Hp-1)*10,Control_input_pumps(1,:),'k','LineWidth',1.75);
hold on;
subplot(3,2,2)
plot(time:10:time+(Hp-1)*10,disturbance(2,:)*60,'k','LineWidth',1.75)
subplot(3,2,4)
plot(time:10:time+Hp*10,X_prediction(10,:),'k','LineWidth',1.75);
hold on;
subplot(3,2,6)
stairs(time:10:time+(Hp-1)*10,Control_input_pumps(2,:),'k','LineWidth',1.75);
hold on;