function Delta = BuildDeltaContinues(NumberOfStates, p)
%BUILDDELTA  Returns the Delta matrix for the two tank topology with
%NumberOfStates
Delta = zeros(NumberOfStates, 1);
Delta(2) = [-p(4)];
Delta(NumberOfStates-1) = [p(4)];
end

