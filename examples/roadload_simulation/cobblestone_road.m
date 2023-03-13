%% User Entry

% Vehicle Path
vehicle_path     = fullfile(cd,'sim_models\vehicles','minivan.hdf5');

% Environment Path
environment_path = fullfile(cd,'sim_models\environments','warm_day.hdf5');

% Road Path
road_path        = fullfile(cd,'sim_models\roads','cobblestone.hdf5');

% Output folder
output_folder    = winqueryreg('HKEY_CURRENT_USER', 'Software\Microsoft\Windows\CurrentVersion\Explorer\Shell Folders', 'Desktop');


%% Code

% Create simulation object
solver = chassis_dynamics.sequence.roadload.solver(vehicle_path,...
                                                   environment_path,...
                                                   road_path);

% Simulate
solver.solve;

% Save
solver.save_hdf5(fullfile(output_folder,'simulation.hdf5'));

% Plots
solver.result.plot_cg_displacement;
solver.result.plot_wheel_displacement;

% Animation
video_path = fullfile(output_folder,'simulation.avi');
solver.animate(0.25,video_path);
