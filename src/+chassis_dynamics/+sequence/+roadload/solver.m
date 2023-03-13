classdef solver < chassis_dynamics.sequence.template.solver
%SOLVER Roadload simulation
    properties
        %VEHICLE_PATH Path to vehicle hdf5 file
        vehicle_path char
        %ENVIRONMENT_PATH Path to environment hdf5 file
        environment_path char
        %ROAD_PATH Path to road profile hdf5 or csv file
        road_path char
    end
    methods
        % Constructor
        function this = solver(vehicle_path,environment_path,road_path)
        %SOLVER Class constructor
        
            % Load vehicle
            vehicle = chassis_dynamics.model.simple_car.assembly.load_hdf5(vehicle_path);
            
            % Load environment
            environment = chassis_dynamics.model.environment.load_hdf5(environment_path);
            
            % Load road
            [~,~,ext] = fileparts(road_path);
            switch lower(ext)
                case '.csv'
                    road = chassis_dynamics.model.simple_road.load_csv(road_path);
                case '.hdf5'
                    road = chassis_dynamics.model.simple_road.load_hdf5(road_path);
                otherwise
                    error('Unrecognized file type %s\n',ext);
            end
            
            % Create model
            residual = chassis_dynamics.sequence.roadload.residual(vehicle);
            model = chassis_dynamics.sequence.roadload.model(residual);

            % Create force object
            force_inputs = struct;
            force_inputs.environment  = environment;
            force_inputs.road         = road;
            force_inputs.vehicle      = vehicle;
            force = chassis_dynamics.sequence.roadload.force(force_inputs);
            
            % Populate object
            this = this@chassis_dynamics.sequence.template.solver(model,force);
            this.vehicle_path = vehicle_path;
            this.environment_path = environment_path;
            this.road_path = road_path;
        
        end
        % Solve
        function solve(this)
        %SOLVE Solve for vehicle state over time
        
            % Get force index
            indF = this.get_indF;
            
            % Initial conditions
            this.model.wheel_contact = true(1,4);
            Xic = this.solve_ic(indF);
            
            % Solve
            [t_out,Xd] = ode45(@(t,X) this.EOM(t,X,indF),...
                               [0,this.force.inputs.road.time(end)],...
                               Xic);
            
            % MPC recovery for displacements and velocities
            Xd_recovered = recover_responses(this,Xd);
            
            % Create output structure
            this.result = chassis_dynamics.sequence.roadload.result(t_out,...
                                                                         Xd_recovered,...
                                                                         this.model.map,...
                                                                         this.model.desc);
            
        end
        % Plotting
        function animate(this,playback_speed,video_file)
        %ANIMATE Animate the simulation results
            
            % Default inputs
            if nargin < 2
                playback_speed = 1;
            end
            if nargin < 3
                video_file = '';
            end
            
            % Create figure and axes
            f1 = figure;
            set(f1,'Units','Normalized','OuterPosition',[0.1,0.1,0.8,0.8]);
            set(f1,'color',[0.5 0.5 0.5])
            ax1 = axes(f1,'visible','off','Projection','perspective');
            
            % Set aspect ratio and view angle
            daspect(ax1,[1 1 1]);
            view(ax1,45,20);
            
            % Run the animation loop
            drawnow;
            timer_plot = tic;
            frames = cell(0);
            timeReached = 0;
            try
                while true
                    curr_time = toc(timer_plot) * playback_speed;
                    if curr_time > this.result.time(end)
                        break
                    end
                    cla(ax1);
                    hold(ax1,'on');
                    this.plot_state(ax1,curr_time);
                    hold(ax1,'off');
                    drawnow;
                    if ~isempty(video_file)
                        frame = getframe(f1);
                        frames = [frames {frame}]; %#ok<AGROW>
                    end
                    timeReached = curr_time;
                end
            catch ex
                if ~isvalid(f1)
                    fprintf(1,'User cancelled animation\n');
                else
                    rethrow(ex);
                end
            end
            
            % Write video if requested
            if ~isempty(video_file)
                video_obj = VideoWriter(video_file);
                video_obj.FrameRate = (numel(frames) / timeReached) * playback_speed;
                open(video_obj);
                for i = 1:numel(frames)
                    writeVideo(video_obj,frames{i});
                end
                close(video_obj);
            end
            
        end
        % Serialization
        function save_hdf5(this,filename)
        %SAVE_HDF5 Save to an HDF5 file
            
            % Create file
            file_obj = h5io.file(filename,'w');
            
            % Save attributes
            file_obj.attributes.vehicle_path       = this.vehicle_path;
            file_obj.attributes.environment_path   = this.environment_path;
            file_obj.attributes.road_path          = this.road_path;
            
            % Save sub groups
            if ~isempty(this.result)
                result_group = file_obj.add_group('result');
                this.result.save_hdf5(result_group);
            end
            
        end
    end
    methods (Access = private)
        function indF = get_indF(this)
        %GET_INDF Get force application index
            
            % Get force map
            L_model = this.model.get_L;
            mapF = this.force.map;
            mapM = L_model.map;
            
            % Get the force map
            indF = chassis_dynamics.tools.map_index(mapM(:,1:2),mapF);
            if numel(indF) ~= size(mapF,1)
                error('There are force ID/DOF pairs that don''t exist in the model');
            end
            
        end
        function Xic = solve_ic(this,indF)
        %SOLVE_IC Initial conditions solver
            
            % Get vehicle matrices
            A  = this.model.get_A;
            L  = this.model.get_L;
            
            % Get MPC info
            [~,~,nset] = this.model.get_MPC;
            
            % Find rigid body rows
            ndof = size(A,1)/2;
            Ak = A(ndof+1:2*ndof,1:ndof);
            Ab = A(ndof+1:2*ndof,ndof+1:2*ndof);
            rowA = full(all(Ak==0,1));
            rowB = full(all(Ab==0,1));
            iR = rowA & rowB;
            iR = [iR iR];
            
            % Initial conditions
            Xic = zeros(ndof*2,1);
            F = this.force.get_FofT(0);
            P = L.data(indF,:)'*F;
            Xic(~iR) = A(~iR,~iR)\(-P(~iR'));
            
            % Velocity at t0 determines initial rigid body motion
            mapV = [1 1
                    1 6];
            indV = chassis_dynamics.tools.map_index(mapV,this.model.map(nset,:)) + ndof;
            Vx = this.force.inputs.road.get_input_at_t('velX',0);
            Xic(indV(1)) = Vx;
            
        end
        function Xd = EOM(this,t,X,indF)
        %EOM Equation of motion
            
            % Get MPC info
            [~,~,nset] = this.model.get_MPC;
        
            % Find road displacement indices
            persistent indZr
            if isempty(indZr)
                mapZr = [111 3
                         112 3
                         113 3
                         114 3];
                indZr = chassis_dynamics.tools.map_index(mapZr,this.force.map);
            end
            
            % Find wheel displacement indices
            persistent indZw
            if isempty(indZw)
                mapZw = [11 3
                         12 3
                         13 3
                         14 3];
                indZw = chassis_dynamics.tools.map_index(mapZw,this.model.map(nset,:));
            end
            
            % Persistent variables
            persistent A
            persistent L
            
            % Road motion
            F = this.force.get_FofT(t);
            
            % Check which wheels are in contact
            wheel_contact_curr = X(indZw)' - F(indZr)' <= 0;
            
            % Update stiffness and locator matrices if boundary conditions
            % changed
            if ~isequal(wheel_contact_curr,this.model.wheel_contact) || t == 0
                this.model.wheel_contact = wheel_contact_curr;
                A  = this.model.get_A;
                L  = this.model.get_L;
            end
            
            % Load vector
            P = L.data(indF,:)'*F;
            
            % Get state
            Xd  =  A*X + P;
            
        end
        function Xd_recovered = recover_responses(this,Xd)
        %RECOVER_RESPONSE Solved matrices are reduced, recover responses
        %here
            
            % Math easier if transposed
            Xd = Xd';
            
            % MPC recovery
            ndof_res = size(Xd,1)/2;
            [MPC,~,nset] = this.model.get_MPC;
            Xd_recovered = zeros(size(MPC,1)*2,size(Xd,2));
            ndof_full = size(Xd_recovered,1)/2;
            Xd_recovered(nset,:) = Xd(1:ndof_res,:);
            Xd_recovered(1:ndof_full,:) = MPC*Xd_recovered(1:ndof_full,:);
            Xd_recovered(nset+ndof_full,:) = Xd((1:ndof_res)+ndof_res,:);
            Xd_recovered((1:ndof_full)+ndof_full,:) = MPC*Xd_recovered((1:ndof_full)+ndof_full,:);
            
            % Transpose back
            Xd_recovered = Xd_recovered';
            
        end
        function plot_state(this,ax,time)
        %PLOT_STATE Plot the current state of the vehicle
            
            % Get the current state
            state = this.result.get_state_at_time(time);
            
            % Current distance
            distX = this.force.inputs.road.get_input_at_t('distX',time);
            distY = this.force.inputs.road.get_input_at_t('distY',time);
            
            % Update axis limits
            cg_loc = this.model.inputs.inputs.body.cg_loc;
            xlim(ax,[-max(this.model.inputs.inputs.axle_r.corner_l.wheel.tire.radius,...
                          this.model.inputs.inputs.axle_r.corner_r.wheel.tire.radius)*2 + distX, ...
                      max(this.model.inputs.inputs.axle_f.corner_l.wheel.tire.radius,...
                          this.model.inputs.inputs.axle_f.corner_r.wheel.tire.radius)*2+this.model.inputs.inputs.wheelbase + distX]);
            ylim(ax,[-max(this.model.inputs.inputs.axle_f.track_width,...
                          this.model.inputs.inputs.axle_r.track_width)/2 * 1.1, ...
                      max(this.model.inputs.inputs.axle_f.track_width,...
                          this.model.inputs.inputs.axle_r.track_width)/2 * 1.1]);
            zlim(ax,[min([this.result.get_response_by_name('Wfl_Tz');...
                          this.result.get_response_by_name('Wfr_Tz');...
                          this.result.get_response_by_name('Wrl_Tz');...
                          this.result.get_response_by_name('Wrr_Tz')]) * 1.1, ...
                          cg_loc(3) * 3 + max(this.result.get_response_by_name('CG_Tz')) * 1.1])
            
            % Plot inputs
            dist_start  = distX - this.model.inputs.inputs.axle_r.corner_l.wheel.tire.radius;
            dist_end    = distX + this.model.inputs.inputs.wheelbase + this.model.inputs.inputs.axle_f.corner_l.wheel.tire.radius;
            this.force.plot_state(dist_start,dist_end,ax);
            
            % Plot model
            offset = [0;-distY;0];
            this.model.plot_state(state,ax,offset);
            
        end
    end
    methods (Static)
        function obj = load_hdf5(filename)
        %LOAD_HDF5 Deserialize an HDF5 file
            
            % Open file
            file_obj = h5io.file(filename,'r');
            
            % Get attributes
            vehicle_path     = file_obj.attributes.vehicle_path;
            environment_path = file_obj.attributes.environment_path;
            road_path        = file_obj.attributes.road_path;
            
            % Get datasets
            superelement_paths = [];
            if any(strcmpi(file_obj.dataset_names,'superelement_paths'))
                superelement_paths_dataset = file_obj.get_dataset('superelement_paths');
                superelement_paths = superelement_paths_dataset.get_data;
            end
            
            % Build object
            obj = chassis_dynamics.sequence.roadload.solver(vehicle_path,environment_path,road_path,superelement_paths);
            
            % Load result
            if any(strcmpi(file_obj.group_names,'result'))
                result_group = file_obj.get_group('result');
                obj.result = chassis_dynamics.sequence.roadload.result.load_hdf5(result_group);
            end
            
        end
    end
end

