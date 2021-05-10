function Bof = BuildBof(NumberOfStates,phi,DeltaT)
%BUILDBd Returns the Bof matrix for the two tank topology
%The first tank is included in the B matrix. dim(phi) = (1,2)

Bof = zeros(NumberOfStates,2);

Bof(1,:) = [-phi(1) * DeltaT, 0];
Bof(end,:) = [0, -phi(2) * DeltaT];
end

