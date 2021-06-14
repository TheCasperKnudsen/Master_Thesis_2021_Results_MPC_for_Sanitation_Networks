%% Load Data
load('System_Identification/Data/Experiment_data_stochastic_MPC_full_DW_26-03.mat');
data = ans.Data';
% Devide by 60 to convert form L/min to L/sec
controlInputs = data(8:9,:)/60;
references = data(14:15,:)/60;

%% Calculate difference:
residuals = (controlInputs-references);

%Plot
figure
for index = 1:1:2
    
    subplot(2,1,index);
    plot(residuals(index,:),'b','LineWidth',1);
    legendInfo = sprintf('$r_{%i}$ [dm]',index);
    ylabel(legendInfo,'interpreter','latex');
    if index == 1
        title('Reduals of: One Step Predictions and Meassurements','interpreter','latex')
    end
    grid on;
end

%% Find variance:
cov = zeros(2,2);

figure
numberOfBins = 200;
Font_scale = 14;
t = -0.03:0.0001:0.03;
for i=1:1:2
    sigmaHat = std(residuals(i,:));
    normdist = @(x,p) normpdf(x, 0, p);
    sigmaHat = mle(residuals(i,:),'pdf',normdist,'start',sigmaHat);
    pdf = normpdf(t,0,sigmaHat);
   
    %Plot
    hold on;
    subplot(2,1,i);
    plot(t,pdf);
    hold on
    histogram(residuals(i,:),numberOfBins,'Normalization','pdf');
    xax = xlabel(['$a$' num2str(i) ' $[L/min]$' ],'interpreter','latex');
    set(xax, 'FontSize', Font_scale);
    %axis([-0.025 0.025 0 0.25])
    yax = ylabel(['$P[A$' num2str(i) '$= a$' num2str(i) '$_{k}]$'],'interpreter','latex');
    set(yax, 'FontSize', Font_scale);
    grid on;
    % Save Variance
    cov(i,i) = sigmaHat^2;
end

cov
