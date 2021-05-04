% Seperating the output variables:
Control_input_pumps = zeros(2,Hp);
Control_input_pumps(1,:) = output(9:9+Hp,:)'/60;
Control_input_pumps(2,:) = output(9+Hp+1:9+Hp+1+Hp,:)'/60;
Overflow = zeros(2,Hp);
Overflow(1,:) = output(10+2*Hp+1:10+3*Hp+1,:)'/60;
Overflow(2,:) = output(10+3*Hp+2:10+4*Hp+2,:)'/60;
X_ref = output(5:6,:);
S_ub = output(7:8,:);
dT = 5  %s
simulink_frequency = 2;

% Disturbance selection:
disturbance = zeros(2,Hp);
for i=0:1:Hp-1
    start_index = time+1+i*dT*simulink_frequency;
    end_index = start_index+dT*simulink_frequency-1;
    disturbance(:,i+1) = mean(mean_disturbance(1:2:3,start_index:end_index),2)/60;
end

% MPC prediction:
X_prediction = zeros(nS, Hp+1);
X_prediction(:,1) = X0/100;
for j=1:1:Hp
    X_prediction(:,j+1) = full(F_system(X_prediction(:,j), Control_input_pumps(:,j), disturbance(:,j), Overflow(:,j), dT ));
end

% Previous state values:
X_previous = circshift(X_previous,-1,2);
U_previous = circshift(U_previous,-1,2);

X_previous(:,5) = X0;
U_previous(:,5) = Control_input_pumps(:,1);

ax(1) = subplot(2,1,1);
% Past state plot
plot(time-5*dT,0:dT:time,X_previous(1,:),'r','LineWidth',1);
hold on;
% Prediction plot
plot(time+dT:dT:time+dT*Hp-dT,X_prediction(1,:),'b','LineWidth',1.75);
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
    xlim([time-5,time-5+37]);
end


ax(2) = subplot(2,1,2);
stairs(time-5*dT:dT:time,U_previous(1,:),'r','LineWidth',1);
hold on;
stairs(time+dT:dT:time+dT*Hp-dT,U(1,:),'b','LineWidth',1.75);
hold on;
stairs(time+dT:dT:time+dT*Hp-dT,Overflow(1,:),'g','LineWidth',1.75);
hold on;
leg = legend('Past action','Predicted action','Overflow','Location','northwest');
set(leg,'Interpreter','latex');
ylim([0,0.03]);
xlabel('k','Interpreter','latex');
ylabel('$u_k$','Interpreter','latex')
if time <= 5
    xlim([0,37]);
else
    xlim([time-5,time-5+37]);
end