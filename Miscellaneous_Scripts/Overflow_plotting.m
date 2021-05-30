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
Font_scale = 14

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
tank1_ex = 18;
tank2_ex = 19;
slack_1 = 24;
slack_2 = 25;

% Defining time variables for plotting and disturbance selection
dT = 5;  %s
simulink_frequency = 2;

figure

ax(1) = subplot(4,1,1);
plot(data(pump1_aux_ref,startDataIndex:endDataIndex));
hold on
plot(data(pump1_aux_flow,startDataIndex:endDataIndex));
hold on
plot(mean_disturbance(1,startDataIndex:endDataIndex),'k--','LineWidth',1.75);
xlim([0, endDataIndex-1]);
ttl = title('Disturbance on tank 1','interpreter','latex');
ttl.FontSize = Font_scale;

%
ax(2) = subplot(4,1,2);
plot(data(tank1_ref,startDataIndex:endDataIndex));
hold on
plot(data(tank1_mes,startDataIndex:endDataIndex));
hold on
xlim([0, endDataIndex-1]);
ttl = title('Tank 1 level','interpreter','latex');
ttl.FontSize = Font_scale;
%
ax(3) = subplot(4,1,3);
plot(data(pump1_ref,startDataIndex:endDataIndex));
hold on
plot(data(pump1_flow,startDataIndex:endDataIndex));
xlim([0, endDataIndex-1]);
ttl = title('Pump 1','interpreter','latex');
ttl.FontSize = Font_scale;

% Overflow calculation and ploting
ax(4) = subplot(4,1,4);
initial_level = data(tank1_ex,5)/100;
tank1_calculated_of_volume = (data(tank1_ex,:)/100-initial_level)*(1/phi(1));
tank1_calculated_of_volume(1) = 0;

plot(tank1_calculated_of_volume(startDataIndex:endDataIndex),'r');
hold on
% plot(cumsum(data(slack_1,startDataIndex:endDataIndex)/60));
% hold on

plot(cumsum(data(slack_1,startDataIndex:endDataIndex))/120,'b');
hold on;
xlim([0, endDataIndex-1]);
ttl = title('Tank 1 overflow','interpreter','latex');
ttl.FontSize = Font_scale;
%
% ax(4) = subplot(3,2,4);
% plot(data(tank2_ref,startDataIndex:endDataIndex));
% hold on
% plot(data(tank2_mes,startDataIndex:endDataIndex));
% hold on
% xlim([0, endDataIndex-1]);
% ttl = title('Tank 2 level','interpreter','latex');
% ttl.FontSize = Font_scale;
% %
% ax(5) = subplot(3,2,5);
% plot(data(pump1_ref,startDataIndex:endDataIndex));
% hold on
% plot(data(pump1_flow,startDataIndex:endDataIndex));
% xlim([0, endDataIndex-1]);
% ttl = title('Pump 1','interpreter','latex');
% ttl.FontSize = Font_scale;
% %
% ax(6) = subplot(3,2,6);
% plot(data(pump2_ref,startDataIndex:endDataIndex));
% hold on
% plot(data(pump2_flow,startDataIndex:endDataIndex));
% xlim([0, endDataIndex-1]);
% ttl = title('Pump 2','interpreter','latex');
% ttl.FontSize = Font_scale;
linkaxes(ax, 'x')

%% ======= Run Kalman for the entire experiment ========
X_zeros = zeros(8,size(data,2));
meas = data(pipe_meas,:);
meas(meas > 50 | meas <= 0) = 0.01;
for i = 1:size(data,2)
    X_zeros(:,i) = Kalman_Filter_Pipes_4Aug(data(pump1_flow,i), data(pump2_aux_flow,i),meas(:,i));
end

%% ======= Plotting the predictions =========
% Defining time variables for plotting and disturbance selection
dT = 5;  %s
simulink_frequency = 2;

time = 21870;%22110;
if mod(time,10) ~= 0 || time < 40
    warningMessage = sprintf('Warning: time has to be a factor of 10 and >= 40');
    uiwait(errordlg(warningMessage));
    return;
end
MPC_run_step = (time - 40)/10 + 1
X0 = [data(tank1_mes,time+1); X_zeros(:,time+1); data(tank2_mes,time+1)];
Control_input_pumps = [Control_input_pump_1(MPC_run_step,:);Control_input_pump_2(MPC_run_step,:)];
Overflow = [Overflow_1(MPC_run_step,:);Overflow_2(MPC_run_step,:)];

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
subplot(4,1,1)
plot(time:10:time+(Hp-1)*10,disturbance(1,:)*60,'k','LineWidth',1.75);
leg = legend('$d_{1}$ reference', '$d_{1}$ flow','$d_{1}$ mean dist.','$d_{1}$ prediction');
set(leg,'Interpreter','latex');
leg.Location = 'SouthOutside';
leg.Orientation = 'Horizontal';
leg.FontSize = Font_scale;
set(gca,'FontSize',Font_scale);
y_lab=ylabel('$u_{d1} [\frac{L}{min}]$','Interpreter','Latex');
y_lab.FontSize = Font_scale;

subplot(4,1,2)
plot(time:10:time+Hp*10,X_prediction(1,:),'k','LineWidth',1.75);
hold on;
plot(702*ones(1,endDataIndex-startDataIndex+1),'g--');
hold on
plot(150*ones(1,endDataIndex-startDataIndex+1),'g--');
leg = legend('$T_{1}$ reference', '$T_{1}$ level', '$T_{1}$ prediction');
set(leg,'Interpreter','latex');
leg.Location = 'SouthOutside';
leg.Orientation = 'Horizontal';
ttl.FontSize = Font_scale;
leg.FontSize = Font_scale;
set(gca,'FontSize',Font_scale);
y_lab=ylabel('$h_{T1} [mm]$','Interpreter','Latex');
y_lab.FontSize = Font_scale;

subplot(4,1,3)
stairs(time:10:time+(Hp-1)*10,Control_input_pumps(1,:),'k','LineWidth',1.75);
leg = legend('$q_{1}$ reference', '$q_{1}$ flow', '$q_{1}$ prediction');
set(leg,'Interpreter','latex');
leg.Location = 'SouthOutside';
leg.Orientation = 'Horizontal';
hold on;
ttl.FontSize = Font_scale;
leg.FontSize = Font_scale;
set(gca,'FontSize',Font_scale);
y_lab=ylabel('$u_{1} [\frac{L}{min}]$','Interpreter','Latex');
y_lab.FontSize = Font_scale;

subplot(4,1,4);

% plot(tank1_calculated_of_volume(startDataIndex:endDataIndex));
% hold on
% plot(cumsum(data(slack_1,startDataIndex:endDataIndex)/60));
% hold on

plot(time:10:time+(Hp-1)*10, cumsum(Overflow(1,:)*60)/12,'k','LineWidth',1.75);
xlim([0, endDataIndex-1]);
ttl = title('Tank 1 overflow','interpreter','latex');
ttl.FontSize = Font_scale;
x_lab=xlabel('time [$0.5$ s]','Interpreter','Latex');
x_lab.FontSize = Font_scale;
leg = legend('$T_1$ measured overflow','$T_1$ MPC implemented overflow', '$T_1$ predicted overflow');
set(leg,'Interpreter','latex');
leg.Location = 'SouthOutside';
leg.Orientation = 'Horizontal';
leg.FontSize = Font_scale;
set(gca,'FontSize',Font_scale);
y_lab=ylabel('$Of_1 [L]$','Interpreter','Latex');
y_lab.FontSize = Font_scale;
% subplot(3,2,2)
% plot(time:10:time+(Hp-1)*10,disturbance(2,:)*60,'k','LineWidth',1.75);
% leg = legend('$d_{2}$ reference', '$d_{2}$ flow','$d_{2}$ mean dist.' ,'$d_{2}$ prediction');
% set(leg,'Interpreter','latex');
% set(leg,'Interpreter','latex');
% ttl.FontSize = Font_scale;
% leg.FontSize = Font_scale;
% set(gca,'FontSize',Font_scale);
% subplot(3,2,4)
% plot(time:10:time+Hp*10,X_prediction(10,:),'k','LineWidth',1.75);
% hold on;
% plot(670*ones(1,endDataIndex-startDataIndex+1),'g--');
% hold on
% plot(150*ones(1,endDataIndex-startDataIndex+1),'g--');
% leg = legend('$T_{2}$ reference', '$T_{2}$ level', '$T_{2}$ prediction');
% set(leg,'Interpreter','latex');
% set(leg,'Interpreter','latex');
% ttl.FontSize = Font_scale;
% leg.FontSize = Font_scale;
% set(gca,'FontSize',Font_scale);
% subplot(3,2,6)
% stairs(time:10:time+(Hp-1)*10,Control_input_pumps(2,:),'k','LineWidth',1.75);
% leg = legend('$q_{2}$ reference', '$q_{2}$ flow','$q_{2}$ prediction');
% set(leg,'Interpreter','latex');
% hold on;
% x_lab=xlabel('time [$0.5$ s]','Interpreter','Latex');
% set(leg,'Interpreter','latex');
% ttl.FontSize = Font_scale;
% leg.FontSize = Font_scale;
% set(gca,'FontSize',Font_scale);