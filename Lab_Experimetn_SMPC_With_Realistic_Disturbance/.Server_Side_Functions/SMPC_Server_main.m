%% ======================== Initialize the MPC ============================
clear all;
clc;
SMPC_init_DW_real

% Enable/Disable plotting 
plotting = true;

if plotting 
    figure
    X_previous = zeros(10,5);
end

%%
number_of_receiving_data = 11;
number_of_sending_data = 8;

% Holding is writting registers (size is number of variables x2)
% 6 states + current time = 7
DataBaseHolding = uint16(zeros(1,number_of_receiving_data*4));
% Input is readin registers
% 2 pump references + 2 overflow slacks + 2 tank references + 2 slack tightenings = 8
DataBaseInput = uint16(zeros(1,number_of_sending_data*4));
% Ignore Coils
DataBaseCoils = logical(0);

Client_IP = 'localhost';
Port_Number = 502;

display('Server is running!')

while(1)
    ModBusTCP = openConnectionServer(Client_IP, Port_Number)
    %Modbus server
    while ~ModBusTCP.BytesAvailable
        %wait for the response to be in the buffer
    end
    
    %Save old measurements
    oldDataBaseHolding = DataBaseHolding;
    
    %Handle new request
    [DataBaseInput,DataBaseHolding] = handleRequest(ModBusTCP, ...
                              DataBaseInput,DataBaseHolding,DataBaseCoils);
   
    %MPC
    if(any(oldDataBaseHolding ~= DataBaseHolding))
        Updated_Measurements_data = unit16Be2doubleLe(DataBaseHolding);
      
        %Run MPC
        X0 = Updated_Measurements_data(1,1:10)';
        time = Updated_Measurements_data(1,11);
        
        output = SMPC_full_DW_real(X0, time); 
        
        %Prepare calculations for sending to client
        data2Send = flip(output');
        DataBaseInput = flip(typecast(data2Send,'uint16'));
        
        if plotting
            PredictionPlotter
        end
    end
    
    fclose(ModBusTCP);
end 