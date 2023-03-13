classdef axle < handle
%AXLE Defines an axle
    properties (SetAccess = private)
        %CORNER_L Left suspension corner
        corner_l chassis_dynamics.model.simple_car.corner
        %CORNER_R Right suspension corner
        corner_r chassis_dynamics.model.simple_car.corner
        %TRACK_WIDTH Distance between wheel centerlines @ this axle (m)
        track_width(1,1) double {mustBeReal, mustBeFinite}
        %ANTIROLL_RATE Antiroll stiffness, currently expressed as a linear
        %spring rate based on the difference in height of the two wheel
        %centers (N/m)
        antiroll_rate(1,1) double {mustBeReal, mustBeFinite}
    end
    methods
        % Constructor
        function this = axle(corner_l,corner_r,track_width,antiroll_rate)
        %AXLE Class constructor
            
            % Set object properties
            this.corner_l       = corner_l;
            this.corner_r       = corner_r;
            this.track_width    = track_width;
            this.antiroll_rate  = antiroll_rate;
        
        end
        % Serialization
        function save_hdf5(this,group_obj)
            
            % Save attributes
            group_obj.attributes.track_width   = this.track_width;
            group_obj.attributes.antiroll_rate = this.antiroll_rate;
            
            % Save sub groups
            corner_l_group = group_obj.add_group('corner_l');
            this.corner_l.save_hdf5(corner_l_group);
            corner_r_group = group_obj.add_group('corner_r');
            this.corner_r.save_hdf5(corner_r_group);
            
        end
    end
    methods (Static)
        function obj = load_hdf5(group_obj)
        %LOAD_HDF5 Deserialize an HDF5 file
            
            % Load attributes
            track_width   = group_obj.attributes.track_width;
            antiroll_rate = group_obj.attributes.antiroll_rate;
            
            % Load sub groups
            corner_l_group = group_obj.get_group('corner_l');
            corner_l = chassis_dynamics.model.simple_car.corner.load_hdf5(corner_l_group);
            corner_r_group = group_obj.get_group('corner_r');
            corner_r = chassis_dynamics.model.simple_car.corner.load_hdf5(corner_r_group);
            
            % Build object
            obj = chassis_dynamics.model.simple_car.axle(corner_l,corner_r,track_width,antiroll_rate);
            
        end
    end
end

