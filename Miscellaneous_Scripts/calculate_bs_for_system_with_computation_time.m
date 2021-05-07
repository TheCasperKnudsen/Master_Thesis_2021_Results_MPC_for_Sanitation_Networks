%Given some computational delay and sampling time for a discrete MPC:
tau= 2;
Ts = 5;
NumberOfStates = 10;
%system paramters:
p = [0.0578290979772850,0.137832091474361,0.000100000000000000,-0.00513392718034500,0.100000000000000];
phi = [1/(1.25^2*pi), 1/(1.25^2*pi)]
%controlling a system with description: 
A = BuildAContinues(NumberOfStates,p,phi);
B = BuildBContinues(NumberOfStates,p,phi);
Bdiscrete = BuildB(NumberOfStates,p,phi,5)

%% Find B1 and B2

%B1 and B2 is dependant on a integral
func = @(t) expm(A * t);
numInt = integral(func,0,tau, 'ArrayValued', 1);
B1 = expm(A*(Ts-tau))*numInt*B
 
numInt = integral(func,0,Ts-tau,'ArrayValued', 1);
B2 = numInt*B
