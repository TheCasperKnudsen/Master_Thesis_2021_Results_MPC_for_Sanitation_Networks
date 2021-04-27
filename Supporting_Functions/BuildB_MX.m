function B = BuildB_MX(NumberOfStates,p,phi,DeltaT)
%BUILDB Returns the B matrix for the two tank topology
%The first tank is included in the B matrix. dim(p) = (1,5) and
%   dim(phi) = (1,2)

B = casadi.MX.zeros(NumberOfStates,2);

B(1,:) = [-phi(1) * DeltaT, 0];
B(2,:) = [p(1) * DeltaT, 0];

for index = 3:1:NumberOfStates
    B(index,:) = [0, 0];
end
B(NumberOfStates,:) = [0, -phi(2) * DeltaT];
end