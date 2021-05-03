% Seperating the output variables:
U = output(1:2,:)/60;
Overflow = output(3:4,:)/60;
X_ref = output(5:6,:);
S_ub = output(7:8,:);
dT = 5  %s
simulink_frequency = 2;

% Disturbance selection:
disturbance = zeros(2,Hp);
for i=0:1:Hp-1
    start_index = time+1+i*dT*simulink_frequency;
    end_index = start_index+dT*simulink_frequency-1;
    disturbance(:,i+1) = mean(mean_disturbance(:,start_index:end_index),2)/60;
end

% MPC prediction:
X_prediction(:,1) = X0/100;
for j=1:1:Hp
    X_prediction(:,j+1) = full(F_system(X_prediction(:,j), U(:,j), disturbance(:,j), dT ));
end

% Previous state values:
X_previous = circshift(X_previous,-1,2)
X_previous(:,5) = X0;

ax(1) = subplot(2,1,1);
% Past state plot
plot(min(time-5*dT,0):dT:time,X_previous(1,:),'r','LineWidth',1);
hold on;
% Prediction plot
plot(time+dT:dT:time+dT*Hp-dT,full(X_prediction(1,:)),'b','LineWidth',1.75);
hold on;
% % Reference plot
% plot(0:1:Hp-1+i,ones(1,size(0:1:Hp-1+i,2))*R_sim,'k','LineWidth',1);
% hold on;
% Plot slacks
plot(time+dT:dT:time+dT*Hp-dT,Overflow(1,:),'magenta');
leg = legend('Past level', 'Predicted level','Overflow', 'Location','southeast');
set(leg,'Interpreter','latex');
ylim([0,7.5]);
xlabel('k','Interpreter','latex');
ylabel('$h_k$','Interpreter','latex');
title('Prediction with chance constraint and high reference');
if time <= 5
    xlim([0,37]);
else
    xlim([i-5,i-5+37]);
end


% ax(2) = subplot(2,1,2);
% stairs(0:1:i-1,full(U_sim(:,1:i)),'b','LineWidth',1);
% hold on;
% stairs(i-1:1:Hp-2+i,full(U_sim_single_step(:,1:end)),'g','LineWidth',1.75);
% hold on;
% plot(i-1:1:i,[full(U_sim_single_step(:,1)),full(U_sim_single_step(:,1))],'r','LineWidth',2);
% hold on;
% plot(0:1:i-1,full(Q_of_sim(:,1:i)),'cyan','LineWidth',1)
% hold on;
% plot(i-1:1:Hp-2+i,full(Q_of_single_step(:,1:end)),'cyan--','LineWidth',1)
% hold off;
% leg = legend('Past action','Predicted action','Implemented action','Overflow','Location','northwest');
% set(leg,'Interpreter','latex');
% ylim([0,0.03]);
% xlabel('k','Interpreter','latex');
% ylabel('$u_k$','Interpreter','latex')
% if i <= 15
%     xlim([0,37]);
% else
%     xlim([i-15,i-15+37]);
% end