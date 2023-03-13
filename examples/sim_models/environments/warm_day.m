%% User Entry

% Where to save
output_path = fullfile(cd,'warm_day.hdf5');

% Environmental conditions
temperature        = 295;      %K
RH                 = 0.7;      %ratio
pressure           = 101325;   %pa


%% Code

% Define environment
environment = chassis_dynamics.model.environment(temperature,...
                                                 RH,...
                                                 pressure);
                                                  
% Save
environment.save_hdf5(output_path);

