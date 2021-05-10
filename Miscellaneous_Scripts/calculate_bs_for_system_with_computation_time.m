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
Delta = BuildDeltaContinues(NumberOfStates, p);
Bd = BuildBdContinues(NumberOfStates,2,p,phi)

Bddiscrete = BuildBd(NumberOfStates,2,p,phi,5);
Deltadiscrete = BuildDelta(NumberOfStates, p,5);
Bdiscrete = BuildB(NumberOfStates,p,phi,5)

%% Find B1 and B2

[b1,b2] = ComputationTimeCompensationB(A,B,Ts,tau);
[bd1,bd2] = ComputationTimeCompensationB(A,Bd,Ts,tau);
[newDelta1,newDelta2] = ComputationTimeCompensationB(A,Delta,Ts,tau);
[Deltak,Deltakm1] = ComputationTimeCompensationDelta(A,Delta,Ts,tau);

