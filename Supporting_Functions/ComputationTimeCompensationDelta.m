function [Deltak,Deltakm1] = ComputationTimeCompensationDelta(A,Delta,Ts,tau)
%COMPUTATIONTIMECOMPENSATIONDELTA Summary of this function goes here
%   Detailed explanation goes here
[Deltak,Deltakm1] = ComputationTimeCompensationB(A,Delta,Ts,tau);

DeltaSum = Deltak+Deltakm1;
DeltaTopSum = sum(DeltaSum(1:end-2));
Deltak(end) = 0;
Deltakm1(end) = 0;

Deltak(end-1) = -(Deltak(end-1)/DeltaSum(end-1)) * DeltaTopSum;

Deltakm1(end-1) = -(Deltakm1(end-1)/DeltaSum(end-1)) * DeltaTopSum;
end

