%Get average variance over Hp
var_D =  diag([average_dist_variance_Hp(:,time+1)]);
ensambles_T1 = [D_sim_ens(1:10,:)];
variance_disturbance_T1 = var(ensambles_T1);

ensambles_pipe = [D_sim_ens(21:30,:)];
variance_disturbance_pipe = var(ensambles_pipe);

combined_variances = [variance_disturbance_T1;variance_disturbance_pipe];

disturbance_variance = zeros(2,Hp);
for i=0:1:Hp-1
    start_index = time+1+i*dT*simulink_frequency;
    end_index = start_index+dT*simulink_frequency-1;
    disturbance_variance(:,i+1) = mean(combined_variances(:,start_index:end_index),2)/60;
end

% Rename two variables for consistency
var_model = modelCovPipe; 
var_measuremants = measCovPipe;

%Predeclare uncertainties (variances) on the tank levels
sigma_x = zeros(2,Hp);
sigma_u = zeros(2,Hp);

% Declare selection matrices
h1 = [1 zeros(1,9)];
h2 = [zeros(1,9) 1];

%Initialise uncertainty on X at time 0
var_x_prev = diag(ones(10,1)*0.0001); % Pretty small we trust our current messurment
for i = 1:Hp
    var_D = diag(disturbance_variance(:,i));
    var_x = full(F_variance(var_x_prev, var_D, var_model, var_pumps,K,dT));
    var_U = K*var_x*K';
    
    % Selecting tank uncertainties
    sigma_x(1,i) = h1*var_x*h1';
    sigma_x(2,i) = h2*var_x*h2';
    
    
    sigma_u(:,i) = [var_U(1,1);var_U(2,2)];
    
    % Prep for next run
    var_x_prev = var_x;
end


% Plot of constraint tightening - Can be ignored.
analyse = 1;
if analyse
    Font_scale = 14
    tightening_x = sqrt(sigma_x)*norminv(0.95);
    tightening_u = sqrt(sigma_u)*norminv(0.95)*60;
    
    %figure
    %subplot(2,1,1)
    hold on;
    ttl = title('Tightening of the tank upperbounds');
    plot(0:Hp-1,tightening_x(1,:),'b-','DisplayName','Tank1','LineWidth',1.75);
    hold on;
    plot(0:Hp-1,tightening_x(2,:),'r-','DisplayName','Tank2','LineWidth',1.75);
    xlabel('Prediction time step [5 sec]'); 
    ylabel('Level tightening [dm]');
    leg = legend('Tank 1 w. avg. $\Sigma_d$','Tank 2 w. avg. $\Sigma_d$', 'Tank 1 w. actual $\Sigma_d$','Tank 2 w. actual $\Sigma_d$');
    set(leg,'Interpreter','latex');
    leg.Location = 'EastOutside';
    leg.Orientation = 'Vertical';
    grid on;
    leg.FontSize = Font_scale;
    set(gca,'FontSize',Font_scale);
    
%     subplot(2,1,2)
%     title('Tightening of the U to allow for LQR');
%     plot(0:Hp-1,tightening_u(1,:),'DisplayName','Pipe Inflow U1','LineWidth',1.75);
%     hold on
%     plot(0:Hp-1,tightening_u(2,:),'DisplayName','Tank2 outflow U2','LineWidth',1.75);
%     xlabel('prediction time step [5 sec]'); 
%     ylabel('U_{ub} tightening [L/min]');
%     leg = legend('Pump 1','Pump 2')
%     grid on;
%     leg.FontSize = Font_scale;
%     set(gca,'FontSize',Font_scale);
end