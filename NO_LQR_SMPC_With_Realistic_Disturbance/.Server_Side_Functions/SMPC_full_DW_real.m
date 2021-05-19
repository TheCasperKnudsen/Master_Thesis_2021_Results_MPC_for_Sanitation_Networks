function [output]  = SMPC_full_DW_real(X0,time)
    eml.extrinsic('evalin');

    % Define persistent variables and constants.
    % MPC Problem and warmstart information
    persistent OCP;
    persistent Hp;
    persistent warmStartEnabler;
    persistent lam_g;
    persistent x_init;

    % Info from previous MPC run.
    persistent U0;
    persistent X_pre;

    % For Make_LQR_real
    persistent sys;
    persistent K;

    % For make_Sigma_real
    persistent var_pumps;
    persistent measCovPipe;
    persistent modelCovPipe;
    persistent F_variance;
    persistent mean_disturbance;
    persistent average_dist_variance_Hp;

    % Constants
    dT = 5;                 % Sample time in minutes
    simulink_frequency = 2;  % Sampling frequency in seconds
    time = int64(round(time));

    if isempty(lam_g) 
        % Get MPC problem and setup warmstart
        OCP = evalin('base','OCP');
        Hp = evalin('base','Hp');
        K = zeros(2,10);

        warmStartEnabler = evalin('base','warmStartEnabler');
        lam_g = 1;
        x_init = 1;


        % Initialize MPC inputs
        U0 = [3.4;6];
        X_pre = X0/100;

        % Get disturbance mean and variance - Used in make sigma_X_real
        load('System_Identification/Noise_Identification/Results/input_uncertainty.mat');
        var_pumps = input_covariance;
        F_variance = evalin('base','F_variance');
        mean_disturbance = evalin('base','mean_disturbance');
        mean_disturbance = mean_disturbance(1:2:3,:);
        average_dist_variance_Hp = evalin('base', 'average_dist_variance_Hp');

        % Get covariances and run Make_LQR_real
        load('System_Identification/Noise_Identification/Results/model_and_messurement_cov_matrices.mat','measCovPipe','modelCovPipe');
        modelCovPipe = blkdiag(0.001,BuildModelCovPipe4Aug(modelCovPipe),0.001);
        sys = evalin('base','sys');       
    end

    % Precompute sigma_X for chance constraint, Open Loop MPC
    make_sigma_X_real  

    %Create forcast from disturbance reference
    disturbance = zeros(2,Hp);
    reference = zeros(10,Hp);
    reference(1:9:10,:) = 3;
    for i=0:1:Hp-1
        start_index = time+1+i*dT*simulink_frequency;
        end_index = start_index+dT*simulink_frequency-1;
        disturbance(:,i+1) = mean(mean_disturbance(:,start_index:end_index),2)/60;
    end


    % Scale inputs from mm (measurements) to dm (MPC)
    X0 = X0/100;
    % Calculate the upper bound on tank state adjustment
    Ub_adjust = zeros(2,1);
    
    if X0(1) >= sys.X_ub(1)
        Ub_adjust(1) = X0(1) - sys.X_ub(1);
    end
    if X0(10) >= sys.X_ub(10)
        Ub_adjust(2) = X0(10) - sys.X_ub(10);
    end
    
    % run openloop MPC
    if warmStartEnabler == 1
        % Parametrized Open Loop Control problem with WARM START
        [u , S, S_ub, lam_g, x_init,Obj] = (OCP(X0,U0,disturbance, lam_g, x_init, dT,reference,Ub_adjust, sigma_x,sigma_u));
    elseif warmStartEnabler == 0
        % Parametrized Open Loop Control problem without WARM START 
        [u , S, S_ub,Obj] = (OCP(X0 ,U0, disturbance, dT, reference,Ub_adjust,sigma_x,sigma_u));
    end
    % Get numeric values for results
    u_full = full(u);
    S_full = full(S);
    S_ub_full = full(S_ub);


    % Find LQR contribution and saturate control input
    u_2_implement = u_full(:,1);

    % Create output
    client_output = [u_2_implement(:,1); S_full(:,1)]*60;             % Scale outputs form L/s (MPC) to L/m (pump)
    tank_ref = [reference(1,1);reference(end,1)]*100;   % Scale reference form dm (MPC) to mm (Lab)
    client_output = [client_output; tank_ref; S_ub_full(:,1)];
    
    % Augmenting the output for plotting
    output = [client_output; ...
        u_full(1,:)';u_full(2,:)';S_full(1,:)';S_full(2,:)';S_ub_full(1,:)';S_ub_full(2,:)'; full(Obj); Ub_adjust;];

    % Set vairables for next iteration and make sure they don't break the
    % bounds.
    U0 = u_full(:,1);
    X_pre = full(sys.F_system(X0, u_full(:,1), disturbance(:,1), 0, dT));
    for i =1:9:10
        if X_pre(i) > sys.X_ub(i)
            X_pre(i) = sys.X_ub(i);
        elseif X_pre(i) < sys.X_lb(i)
            X_pre(i) = sys.X_lb(i);
        end
    end
end
