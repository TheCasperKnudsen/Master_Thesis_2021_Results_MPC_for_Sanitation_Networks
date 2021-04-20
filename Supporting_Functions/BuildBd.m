function B = BuildBd(NumberOfStates,pipeDistPosition,p,phi,DeltaT)
%BUILDBd Returns the Bd matrix for the two tank topology
%The first tank is included in the B matrix. dim(p) = (1,5) and
%   dim(phi) = (1,2)

B = zeros(NumberOfStates,2);

B(1,:) = [phi(1) * DeltaT, 0];

B(pipeDistPosition+1,:) = [0, p(1) * DeltaT];
end

