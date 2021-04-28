%% Load truncated data:
load('System_Identification/Data/Rounded_meas_lab_data.mat');
startTime =3;
endTime = size(dataStructure.data,2);
TimeStep = dataTimeStep;
sampleTimes = startTime*TimeStep:TimeStep:endTime*TimeStep;
experimentLength = endTime-startTime+1;

ht = dataStructure.data(3:6,startTime:endTime);
ht(1,:) = hampel(ht(1,:),8);
ht(2,:) = hampel(ht(2,:),8);
ht(3,:) = hampel(ht(3,:),8);
ht(4,:) = hampel(ht(4,:),8);
T2t = dataStructure.data(8,startTime:endTime);
measurementsTruncated = [ht; T2t];
measurementsTruncated = uConv(measurementsTruncated,"mmTodm");

inputFlow = dataStructure.data(9:10,startTime:endTime);
inputFlow = uConv(inputFlow,"L/minToL/s");

disturbanceFlow = dataStructure.data(13,startTime:endTime);
disturbanceFlow = uConv(disturbanceFlow,"L/minToL/s");
%% 1 step prediction
    x_estTruncated(:,1) = measurementsTruncated(:,startTime);
for k = 1:1:experimentLength-1
    x_estTruncated(:,k+1) = (A*measurementsTruncated(:,k) + B*inputFlow(:,k) + Bd*disturbanceFlow(:,k) + Delta)';
end

% Figur plotter
figure
for index = 1:1:NumberOfStates-1
    subplot(NumberOfStates-1,1,index);
    
    plot(sampleTimes, x_estTruncated(index,:) ,'b','LineWidth',1);
    hold on;
    plot(sampleTimes, measurementsTruncated(index,:) ,'r','LineWidth',1);
    
    
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

residuals = (measurementsTruncated-x_estTruncated);
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

%% Plot Residuals of the truncated system
figure
for i = 1:1:NumberOfStates-1
    subplot(NumberOfStates-1,1,i);
    if i == NumberOfStates-1
        histogram(residuals(i,:),numberOfBins,'Normalization','pdf');
        xlabel(['$r_{T2} [dm]$'],'interpreter','latex');
        %axis([-0.05 0.05 0 0.9])
    else
        histogram(residuals(i,:),numberOfBins,'Normalization','pdf');
        xlabel(['$r$' num2str(i) ' $[dm]$' ],'interpreter','latex');
        %axis([-0.025 0.025 0 0.25])
    end
    ylabel(['$P[R$' num2str(i) '$= r$' num2str(i) '$_{k}]$'],'interpreter','latex');
    grid on;
end



%% Find residual distribution for truncated measurement noise
figure
messuremnt_sigmaHat = [];


for i = 1:1:NumberOfStates-1

    residual_col = residuals(i,:)';
    noise_residual = residual_col(residual_col < 0.005);
    noise_residual = noise_residual(noise_residual > -0.005);
    noise_residual(noise_residual ~= 0) = 0;

    for t = 0:1:11
        temp_residual_high = residual_col(residual_col > 0.005 + 0.01 * t & residual_col < 0.015 + 0.01 * t) - 0.01 * ( t+1) ;
        temp_residual_high(temp_residual_high ~= (t + 1)) = 0.01 *(t + 1);

        temp_residual_low=  residual_col(residual_col < - 0.005 - 0.01 * t & residual_col > -0.015 - 0.01 *t) + 0.01 * ( t+1);
        temp_residual_low( temp_residual_low ~= -(t + 1) ) = -0.01 *(t + 1);
        noise_residual = [noise_residual; temp_residual_high; temp_residual_low];
    end
  
    %Initial variance guess
    sigmaHat = std(noise_residual);
    %Improve the initial variance guess
    normdist = @(x,p) normpdf(x, 0, p);
    sigmaHat = mle(noise_residual,'pdf',normdist,'start',sigmaHat);
    t = -0.08:0.0001:0.08;
    pdf = normpdf(t,0,sigmaHat);
    
    %Plot
    subplot(NumberOfStates-1,1,i);
    if i == NumberOfStates-1
        plot(t,pdf);
        hold on;
        histogram(noise_residual,20,'Normalization','pdf');
        xlabel(['$v$' num2str(i) '$_k [dm]$' ],'interpreter','latex');
        axis([-0.03 0.03 0 50])
    else
        plot(t,pdf);
        hold on;
        histogram(noise_residual,20,'Normalization','pdf');
        xlabel(['$v$' num2str(i) '$_k [dm]$' ],'interpreter','latex');
        axis([-0.025 0.025 0 300])
    end
    ylabel(['$P[V$' num2str(i) '$= v$' num2str(i) '$_{k}]$'],'interpreter','latex');
    grid on;
    messuremnt_sigmaHat = [messuremnt_sigmaHat, sigmaHat];
end

%% Find residual distribution for model noise 

figure
model_sigmaHat = [];


for i = 1:1:NumberOfStates-1
    
    if i == NumberOfStates-1;
        model_residual = residuals(i,:)';
    else
        residual_col = residuals(i,:)';
        model_residual = residual_col(residual_col < 0.005);
        model_residual = model_residual(model_residual > -0.005);
        
        for t = 0:1:15
            model_residual= [model_residual; residual_col(residual_col > 0.005 + 0.01 * t & residual_col < 0.015 + 0.01 * t) - 0.01 * (t+1)];
            model_residual= [model_residual; residual_col(residual_col < - 0.005 - 0.01 * t & residual_col > -0.015 - 0.01 * t) + 0.01 * (t+1)];
        end
    end
    
    %Initial variance guess
    sigmaHat = std(model_residual);
    %Improve the initial variance guess
    normdist = @(x,p) normpdf(x, 0, p);
    sigmaHat = mle(model_residual,'pdf',normdist,'start',sigmaHat);
    t = -0.08:0.0001:0.08;
    pdf = normpdf(t,0,sigmaHat);


    [bins,edges] = histcounts(residuals(1,:),numberOfBins,'Normalization','pdf');
    %Plot
    subplot(NumberOfStates-1,1,i);
    if i == NumberOfStates-1
        plot(t,pdf);
        hold on;
        histogram(model_residual,20,'Normalization','pdf');
        xlabel(['$w_{T2} [dm]$'],'interpreter','latex');
        axis([-0.006 0.006 0 100])
    else
        plot(t,pdf);
        hold on;
        histogram(model_residual,edges,'Normalization','pdf');
        xlabel(['$w$' num2str(i) ' $_k[dm]$' ],'interpreter','latex');
        axis([-0.006 0.006 0 800])
    end
    ylabel(['$P[W$' num2str(i) '$= w$' num2str(i) '$_{k}]$'],'interpreter','latex');
    grid on;
    model_sigmaHat = [model_sigmaHat,sigmaHat];
end