clear all; clc; close all;
%% Load data
load('Correction_the_pump_flows.mat');

data = ans.Data;
data = data';

tank1_in_level = data(1,:)/100;             % [dm]
inflow_to_tank1 = data(13,:);               % [L/min]
outflow_from_tank1 = data(8,:);             % [L/min]

tank2_in_level = data(7,:)/100;             % [dm]
outflow_from_tank2 = data(9,:);             % [L/min]

buffertank_level = data(end,:)/100;         % [dm]
lat_inflow = data(12,:);                    % [L/min]

internal_area_tank1_and_tank2 = 1.25^2 * pi;
%% Calc conr factor aux_pump1.
start_t1_inflowt1 = 1000;
end_t1_inflowt1 = 1300;
timedifference = end_t1_inflowt1 - start_t1_inflowt1;

volume_aux_pump = cumsum(inflow_to_tank1(start_t1_inflowt1:end_t1_inflowt1))/120;
volume_tank1_inflow = (tank1_in_level(:,start_t1_inflowt1:end_t1_inflowt1) - tank1_in_level(:,start_t1_inflowt1)) *internal_area_tank1_and_tank2;
figure 
plot(volume_tank1_inflow);
hold on;
plot(volume_aux_pump);
legend('Calc from tank','Calc form pump')

correction_factor_aux_pump1 = (volume_tank1_inflow(end) - volume_aux_pump(end))*2*60/(timedifference)


new_volume_aux_pump = cumsum(inflow_to_tank1(start_t1_inflowt1:end_t1_inflowt1)+correction_factor_aux_pump1)/120;
figure 
plot(volume_tank1_inflow);
hold on;
plot(new_volume_aux_pump);
legend('Calc from tank','Calc form pump')
%%