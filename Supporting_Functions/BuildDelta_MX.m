function Delta = BuildDelta_MX(NumberOfStates, p,DeltaT)
%BUILDDELTA  Returns the Delta matrix for the two tank topology with
%NumberOfStates
Delta = casadi.MX.zeros(NumberOfStates, 1);
Delta(2) = [-p(4)*DeltaT];
Delta(NumberOfStates-1) = [p(4)*DeltaT];
end

