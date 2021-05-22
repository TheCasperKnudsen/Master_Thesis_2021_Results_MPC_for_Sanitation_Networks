%% Calculate one step residuals
close all;
clear
clc

%% Load Data
load('System_Identification/GravityPipe_Parameter_Estimation/results/Lat_inflow_4_aug_states_28-Apr-2021.mat');
NumberOfPipeStates = N_states + N_augmented_states;
parameters = estimatedParameters;


%% Setup system model
%System Model
NumberOfStates = 2 + NumberOfPipeStates;
x(:,1) = zeros(NumberOfPipeStates,1);

% Paramaters identified for the lab setup
phi = [1,1/4.908738521234052];
maxDeltaT = 1/(parameters(2)+parameters(3))

stepLength = 30;
DeltaT = 8;
%% Free runnig estimate
for i = 1:1:5
    
    % System matrices - x(n+1) = A x(n) + B u(n) + Bd ud(n) + Delta
    A = BuildA(NumberOfStates,parameters,phi,DeltaT);
    B = BuildB(NumberOfStates,parameters,phi,DeltaT);
    Bd = BuildBd(NumberOfStates,4,parameters,phi,DeltaT);
    Delta = BuildDelta(NumberOfStates, parameters,DeltaT);
    C = BuildCfor4Aug;

    % Assuming that the first tank state is uncorrelated, since it
    % only actuated by a pump, we remove it:
    A = A(2:end-1,2:end-1);
    B = B(2:end-1,1);
    Bd = Bd(2:end-1,2);
    Delta =  Delta(2:end-1,:);
    C = C(2:5,2:9);
    
    stepLengthInTime = stepLength*DeltaT;
    time = 0:DeltaT:stepLengthInTime;
    inputFlow = ones(1,stepLength)*3;
    
    
    for k = 1:1:stepLength
    x(:,k+1) = (A*x(:,k) + B*inputFlow(:,k))';
    end
    
    for i = 1:1:NumberOfPipeStates    
    ax(i) = subplot(NumberOfPipeStates,1,i);
    hold on;
    plot(time,x(i,:));
    axis([0,90,0,2])
    end

   DeltaT = DeltaT +0.2;
end
%linkaxes(ax,'x')