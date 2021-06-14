clear all; clc; close all;
%% Load data
load('SMPC_DW_Realistic_.mat');

data = ans.Data;
data = data';

MPC_predicted_overflow_tank1 = data(24,:);  % [L/min]
tank1_ex_level = data(18,:)/100;            % [dm]
inflow_to_tank1 = data(13,:);               % [L/min]
outflow_from_tank1 = data(8,:);             % [L/min]

figure
plot(tank1_ex_level);
figure
plot(outflow_from_tank1);

start_of_overflow =1015 %25 %815;
end_of_overflow =1650 %624 %1650;
%% Estimate overflow for tank 1
close all;

differnece_in_tank1_flow = (inflow_to_tank1 - outflow_from_tank1);
differnece_in_ex_tank1_level = tank1_ex_level(start_of_overflow:end_of_overflow) - tank1_ex_level(start_of_overflow);
ex_tank1_area = (3^2 * pi) - (1.25^2 *pi);

figure 
plot(differnece_in_tank1_flow(:,start_of_overflow:end_of_overflow));
hold on;
plot(MPC_predicted_overflow_tank1(:,start_of_overflow:end_of_overflow));

volume_from_differnece = cumsum(differnece_in_tank1_flow(:,start_of_overflow:end_of_overflow))/120;
volume_from_prediciton = cumsum(MPC_predicted_overflow_tank1(:,start_of_overflow:end_of_overflow))/120;
volume_from_ex_tank1   = differnece_in_ex_tank1_level*ex_tank1_area;

figure 
plot(volume_from_differnece);
hold on;
plot(volume_from_prediciton);
hold on;
plot(volume_from_ex_tank1);
legend({'Calc from Difference','Calc form Predictions','Calc from Tank'},'Location','northwest')
title('Estimated overflow volume')
xlabel('Time [0.5 sec]') 
ylabel('Volume [L]') 
