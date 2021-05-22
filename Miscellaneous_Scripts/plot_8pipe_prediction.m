%% Calculate one step residuals
close all;
clear
clc

%% Load Data
load('System_Identification/GravityPipe_Parameter_Estimation/results/Lat_inflow_4_aug_states_28-Apr-2021.mat');
NumberOfPipeStates = N_states + N_augmented_states;
parameters = estimatedParameters;
x_est(:,1) = estimatedInitialStates;

TimeStep = dataTimeStep;
startTime = 1;
EndTime = size(data,2);
sampleTimes = startTime*TimeStep:TimeStep:EndTime*TimeStep;

measurements = [h(:,startTime:EndTime); T2(:,startTime:EndTime)];
experimentLength = EndTime-startTime+1;
inputFlow = Q(1,startTime:EndTime);
disturbanceFlow = Q(4,startTime:EndTime);

%% Setup system model
%System Model
NumberOfStates = 2 + NumberOfPipeStates;

% Paramaters identified for the lab setup
phi = [1,1/4.908738521234052];
DeltaT = 0.5;

% System matrices - x(n+1) = A x(n) + B u(n) + Bd ud(n) + Delta
A = BuildA(NumberOfStates,parameters,phi,DeltaT);
B = BuildB(NumberOfStates,parameters,phi,DeltaT);
Bd = BuildBd(NumberOfStates,3,parameters,phi,DeltaT);
Delta = BuildDelta(NumberOfStates, parameters,DeltaT);
C = BuildCfor4Aug;

% Assuming that the first tank state is uncorrelated, since it
% only actuated by a pump, we remove it:
A = A(2:end-1,2:end-1);
B = B(2:end-1,1);
Bd = Bd(2:end-1,2);
Delta =  Delta(2:end-1,:);
C = C(2:5,2:9);

%% Free runnig estimate
for k = 1:1:experimentLength-1
    x_est(:,k+1) = (A*x_est(:,k) + B*inputFlow(:,k) + Bd*disturbanceFlow(:,k) + Delta)';
end

%% Plot 
k = 1;
for i = 1:8 
    subplot(8,1,i)
    plot(x_est(i,:));
    
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