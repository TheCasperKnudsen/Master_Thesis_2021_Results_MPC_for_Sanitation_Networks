function Bof = BuildBofContinues(NumberOfStates,phi)
%BUILDBd Returns the Bof matrix for the two tank topology
%The first tank is included in the B matrix. dim(phi) = (1,2)

Bof = casadi.MX.zeros(NumberOfStates,2);

Bof(1,:) = [-phi(1), 0];
Bof(end,:) = [0, -phi(2)];
end

