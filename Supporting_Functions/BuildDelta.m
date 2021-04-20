function Delta = BuildDelta(NumberOfStates, p,DeltaT)
%BUILDDELTA  Returns the Delta matrix for the two tank topology with
%NumberOfStates
Delta = zeros(NumberOfStates, 1);
Delta(2) = [-p(4)*DeltaT];
Delta(NumberOfStates-1) = [p(4)*DeltaT];
end

