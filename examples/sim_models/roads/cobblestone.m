%% User Entry

% Where to save
output_path = fullfile(cd,'cobblestone.hdf5');

% Basics
block_length_nom  = 0.15; %m
block_length_sig  = 0.01; %m
block_height_sig  = 0.02; %m
total_dist        = 50;   %m
d_dist            = 0.01; %m
n_dist = total_dist / d_dist + 1;
distX = [0:d_dist:total_dist]'; %#ok<NBRAK>

% Left profile
distZl = zeros(n_dist,1);
prof_ind = 1;
curr_height = 0;
while true
    curr_length = block_length_nom + randn * block_length_sig;
    stop_dist = curr_length + distX(prof_ind);
    while true
        distZl(prof_ind) = curr_height;
        prof_ind = prof_ind + 1;
        if (prof_ind > n_dist)
            break
        end
        if (distX(prof_ind) > stop_dist)
            break
        end
    end
    if (prof_ind > n_dist)
        break
    end
    curr_height = randn * block_height_sig;
end

% Right profile
distZr = zeros(n_dist,1);
prof_ind = 1;
curr_height = 0;
while true
    curr_length = block_length_nom + randn * block_length_sig;
    stop_dist = curr_length + distX(prof_ind);
    while true
        distZr(prof_ind) = curr_height;
        prof_ind = prof_ind + 1;
        if (prof_ind > n_dist)
            break
        end
        if (distX(prof_ind) > stop_dist)
            break
        end
    end
    if (prof_ind > n_dist)
        break
    end
    curr_height = randn * block_height_sig;
end

% Vehicle velocity
road_speed = 10;
rateZ = zeros(n_dist,1);
time  = distX / road_speed;


%% Code

% Define road
road = chassis_dynamics.model.simple_road(time,...
                                          distX,...
                                          rateZ,...
                                          distZl,...
                                          distZr);
                                           
road.save_hdf5(output_path);

