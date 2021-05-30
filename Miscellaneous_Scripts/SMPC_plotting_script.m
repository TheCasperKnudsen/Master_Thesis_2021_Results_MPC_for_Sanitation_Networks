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

load('.\Lab_Experimetn_SMPC_With_Realistic_Disturbance\Data\D_sim_ens.mat');
D_sim = D_sim_ens(1:20:21,:);
if size(data,1) < 22
    data(22,:) = 3*100;
    data(23,:) = 3*100;
end

Font_scale = 14;
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


ax(1) = subplot(3,1,1);
plot(data(pump1_aux_ref,startDataIndex:endDataIndex));
hold on
plot(data(pump1_aux_flow,startDataIndex:endDataIndex));
leg = legend('','$d_{1}$ reference', '$d_{1}$ flow');
set(leg,'Interpreter','latex');
xlim([0, endDataIndex-1]);
ttl = title('Disturbance on tank 1','interpreter','latex');
ttl.FontSize = Font_scale;
leg.FontSize = Font_scale;
y_lab = ylabel('Flow value [$\frac{L}{min}$]','Interpreter','Latex');
ylim([4.5,14]);
set(y_lab, 'FontSize', Font_scale);
set(gca,'FontSize',Font_scale);
%
ax(3) = subplot(3,1,2);
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
ttl = title('Tank 1 level','interpreter','latex');
ttl.FontSize = Font_scale;
leg.FontSize = Font_scale;
y_lab = ylabel('Tank level [$mm$]','Interpreter','Latex');
ylim([130,710]);
set(y_lab, 'FontSize', Font_scale);
set(gca,'FontSize',Font_scale);
%
%
ax(5) = subplot(3,1,3);
plot(data(pump1_ref,startDataIndex:endDataIndex));
hold on
plot(data(pump1_flow,startDataIndex:endDataIndex));
leg = legend('$q_{1}$ reference', '$q_{1}$ flow');
set(leg,'Interpreter','latex');
xlim([0, endDataIndex-1]);
ttl = title('Pump 1','interpreter','latex')
ttl.FontSize = Font_scale;
leg.FontSize = Font_scale;
x_lab=xlabel('time [$0.5$ s]','Interpreter','Latex'); %or h=get(gca,'xlabel')
set(x_lab, 'FontSize', Font_scale);
y_lab = ylabel('Flow value [$\frac{L}{min}$]','Interpreter','Latex');
ylim([2,10]);
set(y_lab, 'FontSize', Font_scale);
set(gca,'FontSize',Font_scale);
%
%

figure 

ax(2) = subplot(3,1,1);
plot(data(pump2_aux_ref,startDataIndex:endDataIndex));
hold on
plot(data(pump2_aux_flow,startDataIndex:endDataIndex));
leg = legend('$d_{2}$ reference', '$d_{2}$ flow');
set(leg,'Interpreter','latex');
xlim([0, endDataIndex-1]);
ttl = title('Disturbance in the middle of the pipe','interpreter','latex');
ttl.FontSize = Font_scale;
leg.FontSize = Font_scale;
y_lab = ylabel('Flow value [$\frac{L}{min}$]','Interpreter','Latex');
ylim([2.5,17]);
set(y_lab, 'FontSize', Font_scale);
set(gca,'FontSize',Font_scale);
%
ax(4) = subplot(3,1,2);
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
ttl = title('Tank 2 level','interpreter','latex')
ttl.FontSize = Font_scale;
leg.FontSize = Font_scale;
y_lab = ylabel('Tank level [$mm$]','Interpreter','Latex');
ylim([130,710]);
set(y_lab, 'FontSize', Font_scale);
set(gca,'FontSize',Font_scale);
%
ax(6) = subplot(3,1,3);
plot(data(pump2_ref,startDataIndex:endDataIndex));
hold on
plot(data(pump2_flow,startDataIndex:endDataIndex));
leg = legend('$q_{2}$ reference', '$q_{2}$ flow');
set(leg,'Interpreter','latex');
xlim([0, endDataIndex-1]);
ylim([7,17]);
ttl = title('Pump 2','interpreter','latex');
ttl.FontSize = Font_scale;
leg.FontSize = Font_scale;
x_lab=xlabel('time [$0.5$ s]','Interpreter','Latex'); %or h=get(gca,'xlabel')
set(x_lab, 'FontSize', Font_scale);
y_lab = ylabel('Flow value [$\frac{L}{min}$]','Interpreter','Latex');
set(y_lab, 'FontSize', Font_scale);
set(gca,'FontSize',Font_scale);
linkaxes(ax, 'x')