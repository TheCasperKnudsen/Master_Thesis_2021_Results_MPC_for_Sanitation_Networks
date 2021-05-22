function [EstimatedX] = Kalman_Filter_Pipes_4Aug(ControlInput,Disturbance,SystemMeas)
%KALMAN_FILTER Summary of this function goes here
%   Detailed explanation goes here
% Can be changed to: [EstimatedX,ErrorCo,KalmanGain] = Kalman_Filter_Pipes_4Aug(ControlInput,Disturbance,SystemMeas)
% Initialize 
persistent PredictedP
persistent PredictedX
persistent NumberOfPipeStates
persistent parameters
persistent Rm
persistent Qm

if isempty(PredictedP)
    load('System_Identification/GravityPipe_Parameter_Estimation/results/Lat_inflow_4_aug_states_28-Apr-2021.mat','estimatedParameters','N_states','N_augmented_states');
    load('System_Identification/Noise_Identification/Results/model_and_messurement_cov_matrices.mat','measCovPipe','modelCovPipe');
    NumberOfPipeStates = N_states + N_augmented_states;
    parameters = estimatedParameters;
    Qm = BuildModelCovPipe4Aug(modelCovPipe);
    Rm = measCovPipe;
    PredictedP = eye(NumberOfPipeStates)*1000;
    PredictedX = [SystemMeas;SystemMeas];
end

%System Model
NumberOfStates = 2 + NumberOfPipeStates;

% Paramaters identified for the lab setup
phi = [1,1/4.908738521234052];
DeltaT = 0.5;

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

%% Filter portion
    
%Compute KalmanGain
    KalmanGain = PredictedP * C' / (C * PredictedP * C' + Rm);

%Compute EstimatedX
    EstimatedX = PredictedX + KalmanGain * (SystemMeas - C * PredictedX);

%Compute ErrorCo
    ErrorCo = PredictedP - KalmanGain * C * PredictedP;
    
%Prediction Step
    PredictedX = A * EstimatedX + B * ControlInput  + Bd * Disturbance + Delta;
    PredictedP = A * ErrorCo * A' + Qm;
end

