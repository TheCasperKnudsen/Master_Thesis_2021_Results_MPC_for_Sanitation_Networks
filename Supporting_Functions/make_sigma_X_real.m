%Get average variance over Hp
var_D =  diag([average_dist_variance_Hp(:,time+1)]);

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
    tightening_x = sqrt(sigma_x)*norminv(0.95);
    tightening_u = sqrt(sigma_u)*norminv(0.95)*60;
    
    figure
    subplot(2,1,1)
    title('Tightening of the tank upperbounds');
    plot(0:Hp-1,tightening_x(1,:),'DisplayName','Tank1');
    hold on;
    plot(0:Hp-1,tightening_x(2,:),'DisplayName','Tank2');
    xlabel('prediction time step [5 sec]'); 
    ylabel('level decrease [dm]');
    grid on;
    
    subplot(2,1,2)
    title('Tightening of the U to allow for LQR');
    plot(0:Hp-1,tightening_u(1,:),'DisplayName','Pipe Inflow U1');
    hold on
    plot(0:Hp-1,tightening_u(2,:),'DisplayName','Tank2 outflow U2');
    xlabel('prediction time step [5 sec]'); 
    ylabel('U_ub decrease [L/min]');
    grid on;
end