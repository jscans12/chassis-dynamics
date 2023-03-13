classdef wheel < handle
%WHEEL Defines a wheel
    properties (SetAccess = private)
        %RIM_MASS the mass of the wheel rim, not including the tire (kg)
        rim_mass(1,1) double {mustBeReal, mustBeFinite}
        %TIRE tire object
        tire chassis_dynamics.model.simple_car.tire
    end
    properties (Dependent)
        %MASS Total mass of the wheel+rim
        mass
    end
    methods
        % Getters and setters
        function mass = get.mass(this)
            mass = this.rim_mass + this.tire.mass;
        end
        % Constructor
        function this = wheel(rim_mass,tire)
        %WHEEL Class constructor
        
            % Set object properties
            this.rim_mass   = rim_mass;
            this.tire       = tire;
            
        end
        % Serialization
        function save_hdf5(this,group_obj)
            
            % Save attributes
            group_obj.attributes.rim_mass = this.rim_mass;
            
            % Save sub groups
            tire_group = group_obj.add_group('tire');
            this.tire.save_hdf5(tire_group);
            
        end
    end
    methods (Static)
        function obj = load_hdf5(group_obj)
        %LOAD_HDF5 Deserialize an HDF5 file
            
            % Load attributes
            rim_mass = group_obj.attributes.rim_mass;
            
            % Load sub groups
            tire_group = group_obj.get_group('tire');
            tire = chassis_dynamics.model.simple_car.tire.load_hdf5(tire_group);
            
            % Build object
            obj = chassis_dynamics.model.simple_car.wheel(rim_mass,tire);
            
        end
    end
end

