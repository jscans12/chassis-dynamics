classdef tire < handle
%TIRE Defines a tire
    properties (SetAccess = private)
        %RADIUS Unloaded tire radius (m)
        radius(1,1) double {mustBeReal, mustBeFinite}
        %MASS Mass of the tire (kg)
        mass(1,1) double {mustBeReal, mustBeFinite}
        %SPRING_RATE Tire spring rate (N/m)
        spring_rate(1,1) double {mustBeReal, mustBeFinite}
        %DAMPING_COEFF Tire damping coefficient (Nm/s)
        damping_coeff(1,1) double {mustBeReal, mustBeFinite}
    end
    methods
        % Constructor
        function this = tire(radius,mass,spring_rate,damping_coeff)
        %TIRE Class constructor
            
            % Set object properties
            this.radius         = radius;
            this.mass           = mass;
            this.spring_rate    = spring_rate;
            this.damping_coeff  = damping_coeff;
            
        end
        % Serialization
        function save_hdf5(this,group_obj)
            
            % Save attributes
            group_obj.attributes.radius        = this.radius;
            group_obj.attributes.mass          = this.mass;
            group_obj.attributes.spring_rate   = this.spring_rate;
            group_obj.attributes.damping_coeff = this.damping_coeff;
            
        end
    end
    methods (Static)
        function obj = load_hdf5(group_obj)
        %LOAD_HDF5 Deserialize an HDF5 file
            
            % Load attributes
            radius        = group_obj.attributes.radius;
            mass          = group_obj.attributes.mass;
            spring_rate   = group_obj.attributes.spring_rate;
            damping_coeff = group_obj.attributes.damping_coeff;
            
            % Build object
            obj = chassis_dynamics.model.simple_car.tire(radius,mass,spring_rate,damping_coeff);
            
        end
    end
end

