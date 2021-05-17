clear all;
%close all;
clc;

%%
while ~exist('data','var')
    [dataName, filePath] = uigetfile('*.mat','Select data structure');
    fullPath = fullfile(filePath, dataName);

 
    if exist(fullPath, 'file')
        loaded = load(fullPath);
        try
            data = loaded.ans.Data';
        catch
            warning('Use the propper data file!');
        end
        
        if ~exist('data','var')
            warningMessage = sprintf('Warning: wrong file:\n%s\n%s', fullPath, 'Choose simulink generated file!');
            uiwait(errordlg(warningMessage));
        end
    else
        warningMessage = sprintf('Warning: mat file does not exist:\n%s', fullPath);
        uiwait(errordlg(warningMessage));
        return;
    end
end

for i=1:1:size(data,2)
    if ~isnan(data(1:5,i))
        startDataIndex = i;
        break;
    end
end
endDataIndex = size(data,2);
%%
load('.\Lab_Deterministic_MPC_full_system_KW\X_ref_sim.mat');
load('.\Lab_Deterministic_MPC_full_system_KW\D_sim.mat');
D_sim = D_sim(1:2:3,:);
if size(data,1) < 22
    data(22,:) = X_ref_sim(1,startDataIndex:endDataIndex)*100;
    data(23,:) = X_ref_sim(2,startDataIndex:endDataIndex)*100;
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
plot(702*ones(1,endDataIndex-startDataIndex+1),'g--');
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