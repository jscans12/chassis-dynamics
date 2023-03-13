%% User Entry

% Where to save
output_path = fullfile(cd,'minivan.hdf5');

% Mass properties
body_mass          = 1927;     %kg
body_MOI           = [ 896
                      4199
                      4422];   %kg*m^2
body_CG            = [1.716
                      0.030
                      0.631];  %m
suspension_mass_f  = 19.9;     %kg
suspension_mass_r  = 17.5;     %kg
outbd_rim_mass_f   = 35.6;     %kg
outbd_rim_mass_r   = 30.0;     %kg
tire_mass          = 12.9;     %kg

% Aero properties
body_CP            = [1.716
                      0.030
                      0.631];  %m
body_SC            = [-1.3
                       0.0];   %m^2

% Dimensions
wheelbase          = 3.091;    %m
track_width_f      = 1.735;    %m
track_width_r      = 1.735;    %m

% Corner Suspension
spring_rate_f      = 55000;    %N/m
spring_rate_r      = 70000;    %N/m
damping_coeff_f    = 3000;     %N*s/m *UNKNOWN*
damping_coeff_r    = 4850;     %N*s/m *UNKNOWN*
motion_ratio_f     = 1.00;     %ratio
motion_ratio_r     = 0.91;     %ratio

% Antiroll
antiroll_rate_f    = 27500;    %N/m
antiroll_rate_r    =     0;    %N/m

% Tire properties
tire_radius        = 0.37;     %m
tire_spring_rate   = 300000;   %N/m
tire_damping_coeff = 50;       %N*s/m *UNKNOWN*


%% Code

% Create tire
tire_obj = chassis_dynamics.model.simple_car.tire(tire_radius,...
                                                  tire_mass,...
                                                  tire_spring_rate,...
                                                  tire_damping_coeff);

% Create front suspension
wheel_obj_f = chassis_dynamics.model.simple_car.wheel(outbd_rim_mass_f,...
                                                      tire_obj);
corner_f = chassis_dynamics.model.simple_car.corner(wheel_obj_f,...
                                                    spring_rate_f,...
                                                    motion_ratio_f,...
                                                    damping_coeff_f,...
                                                    motion_ratio_f,...
                                                    suspension_mass_f);
axle_f   = chassis_dynamics.model.simple_car.axle(corner_f,...
                                                  corner_f,...
                                                  track_width_f,...
                                                  antiroll_rate_f);

% Create rear suspension
wheel_obj_r = chassis_dynamics.model.simple_car.wheel(outbd_rim_mass_r,...
                                                      tire_obj);
corner_r = chassis_dynamics.model.simple_car.corner(wheel_obj_r,...
                                                    spring_rate_r,...
                                                    motion_ratio_r,...
                                                    damping_coeff_r,...
                                                    motion_ratio_r,...
                                                    suspension_mass_r);
axle_r   = chassis_dynamics.model.simple_car.axle(corner_r,...
                                                  corner_r,...
                                                  track_width_r,...
                                                  antiroll_rate_r);

% Create body
body = chassis_dynamics.model.simple_car.body(body_mass,...
                                              body_MOI,...
                                              body_CG,...
                                              body_CP,...
                                              body_SC);

% Create full vehicle assembly
vehicle = chassis_dynamics.model.simple_car.assembly(body,...
                                                     axle_f,...
                                                     axle_r,...
                                                     wheelbase);
                                                      
% Save to HDF5 file
vehicle.save_hdf5(output_path);

