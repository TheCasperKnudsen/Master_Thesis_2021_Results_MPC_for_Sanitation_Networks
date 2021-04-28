    h1 = [1 zeros(1,9)];
    h2 = [zeros(1,9) 1];
    sigma_x = zeros(2,Hp);
    var_D =  diag([average_dist_variance_Hp(:,time+1)]);  
    var_model = modelCovPipe;
    var_pumps = diag([0.0142 0.0117]);
    var_measuremants = measCovPipe;
    var_x_prev = diag(ones(10,1)*0.0001);
    sigma_u = zeros(2,Hp);
    
    for i = 1:Hp
        var_x = full(F_variance(var_x_prev, var_D, var_model, var_pumps, K,10));
        var_U = K*var_x*K';
        
        sigma_x(1,i) = h1*var_x*h1';
        sigma_x(2,i) = h2*var_x*h2';
        sigma_u(:,i) = [var_U(1,1);var_U(2,2)];
        var_x_prev = var_x;
    end
    
    analyse = 1;
    if analyse
        tightening = sqrt(sigma_x)*norminv(0.95);
        figure
        plot(tightening(1,:));
        hold on;
        plot(tightening(2,:));
    end