% Select tank 1 or 2
tank_select = 2;

% Seperating the output variables:
Control_input_pumps = zeros(2,Hp);
Control_input_pumps(1,:) = output(9:9+Hp-1,:)';
Control_input_pumps(2,:) = output(9+Hp:9+2*Hp-1,:)';
Overflow = zeros(2,Hp);
Overflow(1,:) = output(9+2*Hp:9+3*Hp-1,:)';
Overflow(2,:) = output(9+3*Hp:9+4*Hp-1,:)';
adjustment = output(end-1:end,:);
X_ref = output(5:6,:);
S_ub = output(7:8,:);
dT = 5;  %s
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

% Calculate the  tank2 inflow:
tank2_inflow = (p(5)/p(1))*60.*X_prediction(9,:);

% Previous state values:
X_previous = circshift(X_previous,-1,2);
U_previous = circshift(U_previous,-1,2);
Overflow_previous = circshift(Overflow_previous,-1,2);

X_previous(:,5) = X0/100;
U_previous(:,5) = Control_input_pumps(:,1)*60;
Overflow_previous(:,5) = Overflow(:,1)*60;

ax(1) = subplot(2,1,1);
hold off;
% Past state plot
plot(time-4*dT:dT:time,X_previous(10,:),'r','LineWidth',1);
hold on;
% Prediction plot
plot(time:dT:time+dT*Hp,X_prediction(10,:),'b','LineWidth',1.75);
hold on;
plot(time:dT:time+dT, adjustment(tank_select)*ones(1,2),'k','LineWidth',1.75);
hold on;
% % Reference plot
% plot(0:1:Hp-1+i,ones(1,size(0:1:Hp-1+i,2))*R_sim,'k','LineWidth',1);
% hold on;
% Plot slacks
%plot(time:dT:time+dT*Hp-dT,Overflow(tank_select,:),'magenta');
leg = legend('Past level', 'Predicted level', 'Location','southeast');
set(leg,'Interpreter','latex');
ylim([0,7.5]);
xlabel('k','Interpreter','latex');
ylabel('$h_k$','Interpreter','latex');
title('Prediction with chance constraint and high reference');
if time <= 5
    xlim([-25,125]);
else
    xlim([time-25,time-25+150]);
end


ax(2) = subplot(2,1,2);
hold off;
stairs(time-4*dT:dT:time,U_previous(tank_select,:),'r','LineWidth',1);
hold on;
stairs(time:dT:time+dT*Hp-dT,Control_input_pumps(tank_select,:)*60,'b','LineWidth',1.75);
hold on;
stairs(time-4*dT:dT:time,Overflow_previous(tank_select,:),'g--','LineWidth',1.75);
hold on;
stairs(time:dT:time+dT*Hp-dT,Overflow(tank_select,:)*60,'g','LineWidth',1.75);
hold on;
stairs(time:dT:time+dT*Hp-dT,tank2_inflow(1,1:end-1),'k','LineWidth',1.75);
hold on;
leg = legend('Past action','Predicted action','Overflow','Location','northwest');
set(leg,'Interpreter','latex');
ylim([0,25]);
xlabel('k','Interpreter','latex');
ylabel('$u_k$','Interpreter','latex')
if time <= 5
    xlim([-25,125]);
else
    xlim([time-25,time-25+150]);
end