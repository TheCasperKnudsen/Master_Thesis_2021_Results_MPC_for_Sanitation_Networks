function [MPCOutput] = requestFromServer()
    persistent ModBusTCP
    
    if isempty(ModBusTCP)
        eml.extrinsic('evalin');
        ModBusTCP = evalin('base','ModBusTCP');
    end
    %ModBusTCP = openConnectionClient('192.168.100.246', 502); %open the connection

    transIDHi= uint8(randi(256));%8 bits transaction identifier
    transIDLow = uint8(randi(256));
    ProtID = uint8(0); % 8bit Protocol ID (0 for ModBus) 
    Lenght = uint8(6); % 8b Remaining bytes (24) 
    UnitID = uint8(1); % Unit ID (1)
    
    
    FunCod =  uint8(4); % Function code to read register (3)
    Address_Start = uint8(1); % 16b Adress of the register
    Number_of_registers = uint8(8*4);
    message = [transIDHi; transIDLow; uint8(0); ProtID; uint8(0);...
        Lenght; UnitID; FunCod; uint8(0); Address_Start; uint8(0); Number_of_registers];

    % Write the message
    fwrite(ModBusTCP, message,'uint8');

    % check if the message is received correctly
    while ~ModBusTCP.BytesAvailable
        % wait for the response to be in the buffer
    end
    
    % Handle return
    transID =  fread16Bit(ModBusTCP);
    protId =  fread16Bit(ModBusTCP);
    len =  fread16Bit(ModBusTCP);
    UnitID = fread16Bit(ModBusTCP);
    
    bytesToFollow = fread8Bit(ModBusTCP);
    response = uint16(zeros(1,bytesToFollow/2));
    for i=1:bytesToFollow/2
        debug = fread16Bit(ModBusTCP);
        response(i) = debug;
    end
    MPCOutput = flip(typecast(flip(response),'double'));
end

