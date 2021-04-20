%% Initial model
plotEnbaler = 1;
Nx = N_states;
if plotEnbaler == 1
    j=0;
    figure
    for i = 1:1:Nx
        ax(i) = subplot(size(output,2)-1,1,i);
        plot(0:dataTimeStep:size(output(:,1),1)*dataTimeStep-dataTimeStep,output(:,i),'b','LineWidth',0.5)
        hold on
        plot(0:dataTimeStep:size(output(:,1),1)*dataTimeStep-dataTimeStep,y_init.OutputData(:,i),'r','LineWidth',0.5)
        leg = legend('HF Model','Lin. Model','Location','NorthEast');
        set(leg, 'Interpreter', 'latex');
        ylabel(['$h$' num2str(i) '[$' dataStructure.level_convert_to '$]'],'interpreter','latex');
        if i == 1
            title('Initial model','interpreter','latex')
        end
        grid on;
        j = j+1;
    end
    xlabel(['Time [' dataStructure.data_time_unit ']'],'interpreter','latex');
    linkaxes(ax, 'x')
end
%% Estimated system comparison
clear ax;
if plotEnbaler == 1
    j=0;
    figure
    for i = 1:1:Nx
        ax(i) = subplot(size(output,2)-1,1,i);
        plot(0:dataTimeStep:size(output(:,1),1)*dataTimeStep-dataTimeStep,output(:,i),'b','LineWidth',0.5)
        hold on
        plot(0:dataTimeStep:size(output(:,1),1)*dataTimeStep-dataTimeStep,y_final.OutputData(:,i),'r','LineWidth',0.5)
        leg = legend('HF Model','Lin. Model','Location','NorthEast');
        set(leg, 'Interpreter', 'latex');
        ylabel(['$h$' num2str(i) '[$' dataStructure.level_convert_to '$]'],'interpreter','latex');
        if i == 1
            title('Estimated model','interpreter','latex')
        end
        grid on;
        j = j+1;
    end
    xlabel(['Time [' dataStructure.data_time_unit ']'],'interpreter','latex');
    linkaxes(ax, 'x')
end

clear ax;
figure
ax(1) = subplot(size(output,2),1,[1 2]);
plot(0:dataTimeStep:size(output(:,1),1)*dataTimeStep-dataTimeStep,input(:,1),'b','LineWidth',0.5)
hold on
leg = legend('Pipe inflow','Location','NorthEast');
%leg = legend('Tank outflow','Location','NorthEast');
set(leg, 'Interpreter', 'latex');
ylabel(['$Q$' num2str(1) '[$' dataStructure.flow_convert_to '$]'],'interpreter','latex');
title('Flow inputs','interpreter','latex')
grid on;
xlabel(['Time [' dataStructure.data_time_unit ']'],'interpreter','latex');

ax(2) = subplot(size(output,2),1,[4 5]);
plot(0:dataTimeStep:size(output(:,1),1)*dataTimeStep-dataTimeStep,input(:,2),'b','LineWidth',0.5)
hold on
%leg = legend('Pipe inflow','Location','NorthEast');
leg = legend('Tank outflow','Location','NorthEast');
set(leg, 'Interpreter', 'latex');
ylabel(['$Q$' num2str(2) '[$' dataStructure.flow_convert_to '$]'],'interpreter','latex');
grid on;
xlabel(['Time [' dataStructure.data_time_unit ']'],'interpreter','latex');
linkaxes(ax, 'x')


if Nx<N_optimization_variables

    Qout_est = y_final.OutputData(:,N_states+1);

    figure
    plot(Q(3,:),'b','LineWidth',0.5)
    hold on
    plot(Qout_est,'r','LineWidth',0.5)
    ylabel(['$Q_{out}$  [$' dataStructure.flow_convert_to '$]'],'interpreter','latex');
    xlabel(['Time [' dataStructure.data_time_unit ']'],'interpreter','latex');
    leg = legend('Data','Model','Location','NorthEast');
    set(leg, 'Interpreter', 'latex');
    title('Estimated model - flow','interpreter','latex')
end
% 
