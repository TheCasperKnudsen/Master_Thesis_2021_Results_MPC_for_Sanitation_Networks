function [ModelCovPipe4Aug] = BuildModelCovPipe4Aug(modelCovPipe)
%BUILDMODELCOV4AUG Creates the model covariance for the model with 4
%unmeasured states based on the 4 state model.
    ModelCovPipe4Aug = zeros(size(modelCovPipe,2)*2);
    ii = 0;
    for i = 1:2:7
        ii = ii + 1;
        kk = 0;
        for k = 1:2:7
            kk = kk + 1;
            ModelCovPipe4Aug(k:k+1,i:i+1) = modelCovPipe(kk,ii);
        end
    end
end

