function [Buk,Bukm1] = ComputationTimeCompensationB(A,B,Ts,tau)
%COMPUTATIONTIMECOMPENSATIONB Creates B maricies to compenaste for 
% computation time tau with MPC period Ts. See (Maciejowski Predicive
% Control with Constraints page 53).
func = @(t) expm(A * t);
numInt = integral(func,0,Ts-tau,'ArrayValued', 1);
Buk = numInt*B;

numInt = integral(func,0,tau, 'ArrayValued', 1);
Bukm1 = expm(A*(Ts-tau))*numInt*B;
end

