    h1 = [1 zeros(1,5)];
    h2 = [zeros(1,5) 1];
    sigma_x = zeros(2,Hp);
    
    var_D =  diag([average_dist_variance_Hp(:,time+1)]);  
    var_model = diag([0.0449 0.0012 0.0018 0.0008 0.0004 0.0449]); 
    var_pumps = diag([0.0142 0.0117]);
    var_measuremants = diag([0.005 0.0036 0.0045 0.0028 0.0030 0.005]);
    var_x_prev = var_measuremants;
    sigma_u = zeros(2,Hp);
    
    for i = 1:Hp
        var_x = full(F_variance(var_x_prev, var_D, var_model, var_pumps, K,10));
        var_U = K*var_x*K';
        
        sigma_x(1,i) = h1*var_x*h1';
        sigma_x(2,i) = h2*var_x*h2';
        sigma_u(:,i) = [var_U(1,1);var_U(2,2)];
        var_x_prev = var_x;
    end
    
    analyse = 0;
    if analyse
        tightening = sqrt(sigma_x)*norminv(0.95);
        figure
        plot(tightening(1,:));
        hold on;
        plot(tightening(2,:));
    end