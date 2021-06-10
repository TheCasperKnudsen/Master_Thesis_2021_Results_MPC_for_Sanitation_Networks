% Get all interesting .mat files in the current folder
clear;
% Add as many files as you want:
files = dir('Experiment_data_deterministic_MPC_full_DW.mat');
files = [files dir('Experiment_data_deterministic_MPC.mat') dir('Experiment_data_stochastic_MPC_full_DW.mat') dir('SMPC_DW_Realistic.mat') dir('SMPC_NO_LQR_DW_NA.mat')];
%%
% Loop through each file 
for id = 1:length(files)
    % Get the file name 
    [~, f,ext] = fileparts(files(id).name);
    add = '_';
    rename = strcat(f,add,ext) ;
    while isfile(rename)
        add = strcat(add,'_');
        rename = strcat(f,add,ext) ;
    end
    % Add _ to the end
    movefile(files(id).name, rename); 
end