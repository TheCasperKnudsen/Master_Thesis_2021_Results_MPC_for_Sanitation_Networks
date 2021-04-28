function vertMatrix = vertcatComplete(Matrix)
%VERTCATComplete Concatinates a 2D matrix into a column vector
% Note this should only be used for casadi variables
numberOfColums = Matrix.size2;
numberOfrows = Matrix.size1;

vertMatrix = Matrix(:,1);

for index = 2:numberOfColums
    vertMatrix = vertcat([vertMatrix; Matrix(:,index)]);
end
end

