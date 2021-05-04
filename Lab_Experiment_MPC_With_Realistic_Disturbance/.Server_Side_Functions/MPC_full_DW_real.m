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

    % Constants
    dT = 5;                 % Sample time in minutes
    simulink_frequency = 2;  % Sampling frequency in seconds
    time = int64(round(time));

    if isempty(lam_g) 
        % Get MPC problem and setup warmstart
        OCP = evalin('base','OCP');
        Hp = evalin('base','Hp');

        warmStartEnabler = evalin('base','warmStartEnabler');
        lam_g = 1;
        x_init = 0.001;


        % Initialize MPC inputs
        U0 = [3;4.5];
        X_pre = X0/100;  
    end
    
    %Create forcast from disturbance reference
    disturbance = zeros(2,Hp);
    reference = zeros(10,Hp);
    reference(1:9:10,:) = 2;
    for i=0:1:Hp-1
        start_index = time+1+i*dT*simulink_frequency;
        end_index = start_index+dT*simulink_frequency-1;
        disturbance(:,i+1) = mean(mean_disturbance(:,start_index:end_index),2)/60;
    end


    % Scale inputs from mm (measurements) to dm (MPC)
    X0 = X0/100;        
    % run openloop MPC
    if warmStartEnabler == 1
        % Parametrized Open Loop Control problem with WARM START
        [u , S, lam_g, x_init] = OCP(X0,U0,disturbance, lam_g, x_init, dT,reference);
    elseif warmStartEnabler == 0
        % Parametrized Open Loop Control problem without WARM START 
        [u , S] = (OCP(X0 ,U0, disturbance, dT, reference));
    end

    % Get numeric values for results
    u_full = full(u);
    S_full = full(S);

    % Create output
    output = [u_full(:,1); S_full(:,1)]*60;             % Scale outputs form L/s (MPC) to L/m (pump)
    tank_ref = [reference(1,1);reference(end,1)]*100;   % Scale reference form dm (MPC) to mm (Lab)
    output = [output; tank_ref; 0; 0];

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
