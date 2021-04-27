%% Calculate one step residuals
close all;
clear
clc

% The identifaction consists of x parts
% Load data and used for prameter identification
% Setup of system model

%% Load Data
load('System_Identification/GravityPipe_Parameter_Estimation/results/Lab_lateral_inflow_0_aug_states_21-Apr-2021.mat');

TimeStep = dataTimeStep;
startTime = 2;
EndTime = size(data,2);
sampleTimes = startTime*TimeStep:TimeStep:EndTime*TimeStep;

measurements = [h(:,startTime:EndTime); T2(:,startTime:EndTime)];
experimentLength = EndTime-startTime+1;
NumberOfPipeStates = N_states;
pumpFlows = input';
inputFlow = pumpFlows(1:2:3,startTime:EndTime);
disturbanceFlow = pumpFlows(2,startTime:EndTime);


%% Setup Model
% The pipe model is constructed as shown in the figure with a tank at
% either end.
DisplayImage('pipe_sketch.jpg');
NumberOfStates = 2 + NumberOfPipeStates;

% Paramaters identified for the lab setup
p = estimatedParameters;
phi = [1,1/4.908738521234052];
DeltaT = 0.5;

% System matrices - x(n+1) = A x(n) + B u(n) + Bd ud(n) + Delta
A = BuildA(NumberOfStates,p,phi,DeltaT);
B = BuildB(NumberOfStates,p,phi,DeltaT);
Bd = BuildBd(NumberOfStates,2,p,phi,DeltaT);
Delta = BuildDelta(NumberOfStates, p,DeltaT);

% Assuming that the first tank state is uncorrelated, since it
% only actuated by a pump, we remove it:
A = A(2:end,2:end);
B = B(2:end,:);
Bd = Bd(2:end,2);
Delta =  Delta(2:end,:);

%% 1 step prediction
    x_est(:,1) = measurements(:,startTime);
for k = 1:1:experimentLength-1
    x_est(:,k+1) = (A*measurements(:,k) + B*inputFlow(:,k) + Bd*disturbanceFlow(:,k) + Delta)';
end

% Figur plotter
for index = 1:1:NumberOfStates-1
    subplot(NumberOfStates-1,1,index);
    
    plot(sampleTimes, x_est(index,:) ,'b','LineWidth',1);
    hold on;
    plot(sampleTimes, measurements(index,:) ,'r','LineWidth',1);
    
    
    if index == NumberOfStates-1
        ylabel(['$x_{T2} [dm]$'],'interpreter','latex');
        xlabel(['Time [s]'],'interpreter','latex');
    else
        legendInfo = sprintf('$x_{%i}$ [dm]',index);
        ylabel(legendInfo,'interpreter','latex');
    end
    if index == 1
        title('One Step Predictions','interpreter','latex')
    end
    grid on;
end


%% Calculate Residuals
residuals = (measurements-x_est);

%Plot
figure
for index = 1:1:NumberOfStates-1
    
    subplot(size(output,2),1,index);
    plot(sampleTimes,residuals(index,:)','b','LineWidth',1);
    
    hold on;
    if index == NumberOfStates-1
        ylabel(['$r_{T2} [dm]$'],'interpreter','latex');
        xlabel(['Time [s]'],'interpreter','latex');
    else
        legendInfo = sprintf('$r_{%i}$ [dm]',index);
        ylabel(legendInfo,'interpreter','latex');
    end
    if index == 1
        title('Reduals of: One Step Predictions and Meassurements','interpreter','latex')
    end
    grid on;
end

%% Calcualte covariance of the Model noise.
modelCov = cov(residuals');

%% Find residual distribution

%Assuming normal independant residauls
figure
nt_residual_sigmaHat = [];
numberOfBins = 200;
t = -0.08:0.0001:0.08;

for i = 1:1:NumberOfStates-1
    %Make initial guess for the variance
    sigmaHat = std(residuals(i,:));
    normdist = @(x,p) normpdf(x, 0, p);
    sigmaHat = mle(residuals(i,:),'pdf',normdist,'start',sigmaHat);
    pdf = normpdf(t,0,sigmaHat);
    
    hold on;
    subplot(NumberOfStates-1,1,i);
    if i == NumberOfStates-1
        %Plot
        plot(t,pdf);
        hold on
        histogram(residuals(i,:),numberOfBins,'Normalization','pdf');
        xlabel(['$r_{T2} [dm]$'],'interpreter','latex');
        %axis([-0.05 0.05 0 0.9])
    else
        %Plot
        plot(t,pdf);
        hold on
        histogram(residuals(i,:),numberOfBins,'Normalization','pdf');
        xlabel(['$r$' num2str(i) ' $[dm]$' ],'interpreter','latex');
        %axis([-0.025 0.025 0 0.25])
        
    end
    ylabel(['$P[R$' num2str(i) '$= r$' num2str(i) '$_{k}]$'],'interpreter','latex');
    grid on;
    
    %Save data
    nt_residual_sigmaHat = [nt_residual_sigmaHat,sigmaHat];
end

%% Load truncated data:

Analysis_of_Truncated_Data;

%%

for i = 1:1:NumberOfStates-1
    modelCov(i,i) = model_sigmaHat(i).^2;
end
    measCov = diag((nt_residual_sigmaHat - model_sigmaHat).^2);
    
    modelCov
    measCov