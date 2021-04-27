function [C] = BuildCfor4Aug()
%BUILDCFOR4AUG Simply returns the C matrix for a the 10 state Two tank
%topology, with 4 meassurend pipestates and 4 unmeasured

C = zeros(6,10);
C(1,1) = 1;
C(2,3) = 1;
C(3,5) = 1;
C(4,6) = 1;
C(5,8) = 1;
C(6,10) = 1;
end

