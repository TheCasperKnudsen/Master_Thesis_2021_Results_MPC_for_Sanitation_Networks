%% Initial model
plotEnbaler = 1;
Nx = N_states;
Font_scale = 14;
if plotEnbaler == 1
    j=0;
    figure
    for i = 1:1:Nx
        ax(i) = subplot(size(output,2)-1,1,i);
        plot(0:dataTimeStep:size(output(:,1),1)*dataTimeStep-dataTimeStep,output(:,i),'b','LineWidth',0.5)
        hold on
        plot(0:dataTimeStep:size(output(:,1),1)*dataTimeStep-dataTimeStep,y_init.OutputData(:,i),'r','LineWidth',0.5)
        leg = legend('HF Model','Lin. Model','Location','NorthEast');
        leg.FontSize = Font_scale;
        set(leg, 'Interpreter', 'latex');
        y_lab = ylabel(['$h$' num2str(i) '[$' dataStructure.level_convert_to '$]'],'interpreter','latex');
        set(y_lab, 'FontSize', Font_scale);
        if i == 1
            ttl = title('Initial model','interpreter','latex');
            ttl.FontSize = Font_scale;
        end
        grid on;
        j = j+1;
    end
    x_lab = xlabel(['Time [' dataStructure.data_time_unit ']'],'interpreter','latex');
    set(x_lab, 'FontSize', Font_scale);
    linkaxes(ax, 'x')
    
    set(gca,'FontSize',Font_scale);
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
        leg = legend('Measurement','Lin. Model','Location','NorthEast');
        set(leg, 'Interpreter', 'latex');
        leg.FontSize = Font_scale;
        y_lab = ylabel(['$h$' num2str(i) '[$' dataStructure.level_convert_to '$]'],'interpreter','latex');
        set(y_lab, 'FontSize', Font_scale);
        if i == 1
            ttl = title('Estimated model','interpreter','latex');
            ttl.FontSize = Font_scale;
        end
        grid on;
        %set(gca,'FontSize',Font_scale);
        j = j+1;
    end
    x_lab = xlabel(['Time [' dataStructure.data_time_unit ']'],'interpreter','latex');
    set(x_lab, 'FontSize', Font_scale);
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
