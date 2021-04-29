function send2Server(measurements, time)
    %persistent ModBusTCP
    ModBusTCP = openConnectionClient('localhost' , 502); %open the connection

    
    %16 bits transaction identifier
    transIDHi= uint8(randi(256));%8 bits transaction identifier
    transIDLow = uint8(randi(256));
    %Dosn't matter
    ProtID=uint8(0);%8 bits protocol id
    Lenght =uint8(23); % 8bits Remaining bytes 
    UnitID = uint8(1); % Unit ID (1)

    FunCod = uint8(16); % Function code to write multiple holding registers
    Address_offset = uint8(1); % 16b Adress of the register
    reg_number = uint8( size(measurements,1)*4 + 4); %number of register to written
    byteToFollow = uint8(reg_number*2); %bytes that will follow 
    
    timeData = flip(typecast(double(time), 'uint8')');% 4x8bit data: the order of each 2 bytes has to be reversed
    newMeas = flip(measurements);
    measuremntData = flip(typecast(newMeas,'uint8'));

    message = [transIDHi; transIDLow; int8(0); ProtID; int8(0); Lenght; UnitID; FunCod; int8(0); Address_offset; int8(0); reg_number; byteToFollow; measuremntData; timeData];

    fwrite(ModBusTCP, message,'uint8');
end