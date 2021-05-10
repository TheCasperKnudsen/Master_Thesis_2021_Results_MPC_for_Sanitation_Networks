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
    persistent S0;
    persistent sys;
    
    %Persistent Forcast
    persistent mean_disturbance; 

    % Constants
    simulink_frequency = 2;  % Sampling frequency in seconds
    time = int64(round(time));

    if isempty(lam_g) 
        % Get MPC problem and setup warmstart
        OCP = evalin('base','OCP');
        Hp = evalin('base','Hp');
        sys = evalin('base','sys');
        
        warmStartEnabler = evalin('base','warmStartEnabler');
        lam_g = 1;
        x_init = 0.001;


        % Initialize MPC inputs
        U0 = [3;4.5];
        S0 = [0;0];
        X_pre = X0/100;  
        mean_disturbance = evalin('base','mean_disturbance');
        mean_disturbance = mean_disturbance(1:2:3,:);
    
    end
    
    %Create forcast from disturbance reference
    disturbance = zeros(2,Hp);
    reference = zeros(10,Hp);
    reference(1:9:10,:) = 2;
    for i=0:1:Hp-1
        start_index = time+1+i*sys.Ts*simulink_frequency;
        end_index = start_index+sys.Ts*simulink_frequency-1;
        disturbance(:,i+1) = mean(mean_disturbance(:,start_index:end_index),2)/60;
    end


    % Scale inputs from mm (measurements) to dm (MPC)
    X0 = X0/100;        
    % run openloop MPC
    % Parametrized Open Loop Control problem with WARM START    
    if warmStartEnabler == 1
        % Parametrized Open Loop Control problem with WARM START
        [u , S, lam_g, x_init] = OCP(X0, U0, S0, disturbance, lam_g, x_init, reference);
    elseif warmStartEnabler == 0
        % Parametrized Open Loop Control problem without WARM START 
        [u , S] = (OCP(X0 ,U0, S0, disturbance, reference));
    end

    % Get numeric values for results
    u_full = full(u);
    S_full = full(S);

    % Create output
    output = [u_full(:,1); S_full(:,1)]*60;             % Scale outputs form L/s (MPC) to L/m (pump)
    tank_ref = [reference(1,1);reference(end,1)]*100;   % Scale reference form dm (MPC) to mm (Lab)
    output = [output; tank_ref; 0; 0];
    
    % Values for next run
    U0 = u_full(:,1);
    S0 = S_full(:,1);
end
