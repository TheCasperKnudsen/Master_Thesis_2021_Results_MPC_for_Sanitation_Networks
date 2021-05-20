%% ======================== Initialize the MPC ============================
clear all;
clc;
SMPC_init_DW_real

% Enable/Disable plotting 
plotting = false;

if plotting 
    figure
    X_previous = zeros(10,5);
    U_previous = zeros(2,5);
    Overflow_previous = zeros(2,5);
end

% Initialize output variables
Control_input_pumps = zeros(2,Hp);
Overflow = zeros(2,Hp);
Tightening = zeros(2,Hp);

saveFile = matfile(strcat('Saved_predictions',date,'.mat'), 'Writable', true); %Note: writable is true by default IF the file does not exist
saveToIndex = 1;
container_data_points = 7;
saveContainer = zeros(container_data_points,Hp);

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

Client_IP = '192.168.100.246';
Port_Number = 502;
display('Server is running!')
ModBusTCP = openConnectionServer(Client_IP, Port_Number);
display('Connection made')
while(1)
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
        clc
        display('Computing MPC');
        
        tic
        Updated_Measurements_data = unit16Be2doubleLe(DataBaseHolding);
      
        %Run MPC
        X0 = Updated_Measurements_data(1,1:10)';
        time = Updated_Measurements_data(1,11);
        
        output = SMPC_full_DW_real(X0, time); 
        send2Client_output = output(1:8,:);
        
        %Seperate the output into variables
        Control_input_pumps(1,:) = output(9:9+Hp-1,:)';
        Control_input_pumps(2,:) = output(9+Hp:9+2*Hp-1,:)';
        
        Overflow(1,:) = output(9+2*Hp:9+3*Hp-1,:)';
        Overflow(2,:) = output(9+3*Hp:9+4*Hp-1,:)';
        
        Tightening(1,:) = output(9+4*Hp:9+5*Hp-1,:)';
        Tightening(2,:) = output(9+5*Hp:9+6*Hp-1,:)';
        
        Cost = output(end-2,:);
        
        % Save the data:
        saveContainer = [Control_input_pumps;Overflow;Tightening;Cost*ones(1,size(Tightening,2))];
        saveFile.out(saveToIndex:saveToIndex+container_data_points-1, 1:Hp) = saveContainer;
        saveToIndex = saveToIndex+container_data_points;
        
        
        adjustment = output(end-1:end,:);
        X_ref = output(5:6,:);
        
        %Prepare calculations for sending to client
        data2Send = flip(send2Client_output');
        DataBaseInput = flip(typecast(data2Send,'uint16'));
        
%         if plotting
%             PredictionPlotter
%         end
        toc
    end
end 
fclose(ModBusTCP);