classdef simple_road < handle
%SIMPLE_ROAD Define a simple road input
    properties (Constant)
        %DESC Column descriptions
        desc = {'posX'
                'posY'
                'distX'
                'velX'
                'accX'
                'distY'
                'velY'
                'accY'
                'angZ'
                'rateZ'
                'alphaZ'
                'distZl'
                'distZr'};
    end
    properties (SetAccess = immutable)
        %TIME Time in test (s)
        time(:,1) double {mustBeReal, mustBeFinite}
    end
    properties (SetAccess = immutable, GetAccess = private)
        %INTERPOLANT_T Save gridded interpolant for fast lookup (time
        %based)
        interpolant_t(1,1) griddedInterpolant
        %INTERPOLANT_D Save gridded interpolant for fast lookup (distance
        %based)
        interpolant_d(1,1) griddedInterpolant
    end
    methods
        % Constructor
        function this = simple_road(time,distX,rateZ,distZl,distZr)
        %SIMPLE_ROAD Construct an instance of this class
            
            % Verify vector lengths match
            if ~isequal(size(time),size(distX),size(rateZ),size(distZl),size(distZr))
                error('All vectors for road profile must be the same length');
            end
            
            % Each road profile vector must start at zero
            if time(1) ~= 0
                error('Road profile time vector must start at zero');
            end
            
            % Ensure Xv is monotonically increasing
            if ~all(diff(time) > 0)
                error('time vector must be monotonically increasing');
            end
            
            % Forward speed and acceleration calcs
            velX = diff(distX) ./ diff(time);
            velX = [velX(1); velX];
            accX = diff(velX)  ./ diff(time);
            accX = [accX(1); accX];
            
            % Lateral
            accY  = velX .* rateZ;
            velY  = cumtrapz(time,accY);
            distY = cumtrapz(time,velY);
            
            % Yaw
            angZ   = cumtrapz(time,rateZ);
            alphaZ = diff(rateZ) ./ diff(time);
            alphaZ = [alphaZ(1); alphaZ];
            
            % Position
            posXd = velX .* cos(angZ);
            posYd = velX .* sin(angZ);
            posX = cumtrapz(time,posXd);
            posY = cumtrapz(time,posYd);
            
            % Set time
            this.time  = time;
            
            % Assemble data
            data = [posX,posY,distX,velX,accX,distY,velY,accY,angZ,rateZ,alphaZ,distZl,distZr];
            
            % Create interpolant_t
            this.interpolant_t = griddedInterpolant({time ,1:size(data,2)},data,'linear','nearest');
            
            % Create interpolant_d
            this.interpolant_d = griddedInterpolant({distX,1:size(data,2)},data,'linear','nearest');
            
        end
        % Interpolation
        function value = get_input_at_t(this,name,time)
        %GET_INPUT_AT_T Get a specified input at a time
        
            % Find index
            name_index = strcmp(name,this.desc);
            if ~any(name_index)
                error('%s was not found\n',name);
            end
            
            % Defaults
            if nargin < 3
                value = this.interpolant_t.Values(:,name_index);
                return
            end
            
            % Get the value
            value = this.interpolant_t({time,find(name_index)});
            
        end
        function value = get_input_at_d(this,name,dist)
        %GET_INPUT_AT_D Get a specified input at a distance
        
            % Find index
            name_index = strcmp(name,this.desc);
            if ~any(name_index)
                error('%s was not found\n',name);
            end
            
            % Defaults
            if nargin < 3
                value = this.interpolant_d.Values(:,name_index);
                return
            end
            
            % Get the value
            value = this.interpolant_d({dist,find(name_index)});
            
        end
        % Serialization
        function save_hdf5(this,filename)
        %SAVE_HDF5 Save to an HDF5 file
            
            % Get data
            distX  = this.get_input_at_t('distX');
            rateZ  = this.get_input_at_t('rateZ');
            distZl = this.get_input_at_t('distZl');
            distZr = this.get_input_at_t('distZr');
        
            % Create file
            file_obj = h5io.file(filename,'w');
            
            % Save datasets
            file_obj.add_dataset('time',this.time);
            file_obj.add_dataset('distX',distX);
            file_obj.add_dataset('rateZ',rateZ);
            file_obj.add_dataset('distZl',distZl);
            file_obj.add_dataset('distZr',distZr);
            
        end
    end
    methods (Static)
        function obj = load_hdf5(filename)
        %LOAD_HDF5 Deserialize an HDF5 file
            
            % Open file
            file_obj = h5io.file(filename,'r');
            
            % Get datasets
            time_dataset   = file_obj.get_dataset('time');
            distX_dataset  = file_obj.get_dataset('distX');
            rateZ_dataset  = file_obj.get_dataset('rateZ');
            distZl_dataset = file_obj.get_dataset('distZl');
            distZr_dataset = file_obj.get_dataset('distZr');
            
            % Get data
            time   = time_dataset.get_data;
            distX  = distX_dataset.get_data;
            rateZ  = rateZ_dataset.get_data;
            distZl = distZl_dataset.get_data;
            distZr = distZr_dataset.get_data;
            
            % Build object
            obj = chassis_dynamics.model.simple_road(time,distX,rateZ,distZl,distZr);
            
        end
        function obj = load_csv(filename)
        %LOAD_CSV Load from a standard format of CSV file
            
            % Import table
            tab_road = readtable(filename);
            
            % Read columns
            time   = tab_road.time;
            distX  = tab_road.distX;
            rateZ  = tab_road.rateZ;
            distZl = tab_road.distZl;
            distZr = tab_road.distZr;
            
            % Build object
            obj = chassis_dynamics.model.simple_road(time,distX,rateZ,distZl,distZr);
            
        end
    end
end

