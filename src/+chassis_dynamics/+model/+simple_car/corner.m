classdef corner < handle
%CORNER Defines a suspension corner
    properties (SetAccess = private)
        %WHEEL Wheel object
        wheel chassis_dynamics.model.simple_car.wheel
        %SPRING_RATE Rate of the installed spring (N/m)
        spring_rate(1,1) double {mustBeReal, mustBeFinite}
        %SPRING_MOTION_RATIO Motion ratio of the spring (ratio)
        spring_motion_ratio(1,1) double {mustBeReal, mustBeFinite}
        %DAMPER_COEFF Damping coefficient of the installed damper (Nm/s)
        damper_coeff(1,1) double {mustBeReal, mustBeFinite}
        %DAMPER_MOTION_RATIO Motion ratio of the damper (ratio)
        damper_motion_ratio(1,1) double {mustBeReal, mustBeFinite}
        %SUSPENSION_MASS Mass of the suspension components, not including
        %tire and wheel (kg)
        suspension_mass(1,1) double {mustBeReal, mustBeFinite}
    end
    properties (Dependent)
        %WHEEL_SPRING_RATE Wheel rate
        wheel_spring_rate
        %WHEEL_DAMPING_COEFF Damping coefficient at the wheel
        wheel_damping_coeff
        %SPRUNG_MASS Sprung mass at the suspension corner
        sprung_mass
        %UNSPRUNG_MASS Unsprung mass at the suspension corner
        unsprung_mass
    end
    methods
        % Getters and setters
        function wheel_spring_rate = get.wheel_spring_rate(this)
            wheel_spring_rate = this.spring_rate * this.spring_motion_ratio^2;
        end
        function wheel_damping_coeff = get.wheel_damping_coeff(this)
            wheel_damping_coeff = this.damper_coeff * this.damper_motion_ratio^2;
        end
        function sprung_mass = get.sprung_mass(this)
            sprung_mass = this.suspension_mass / 2;
        end
        function unsprung_mass = get.unsprung_mass(this)
            unsprung_mass = this.suspension_mass / 2 + this.wheel.mass;
        end
        % Constructor
        function this = corner(wheel,spring_rate,spring_motion_ratio,...
                               damper_coeff,damper_motion_ratio,...
                               suspension_mass)
        %CORNER Class constructor
        
            % Set object properties
            this.wheel                  = wheel;
            this.spring_rate            = spring_rate;
            this.spring_motion_ratio    = spring_motion_ratio;
            this.damper_coeff           = damper_coeff;
            this.damper_motion_ratio    = damper_motion_ratio;
            this.suspension_mass        = suspension_mass;
        
        end
        % Serialization
        function save_hdf5(this,group_obj)
            
            % Save attributes
            group_obj.attributes.spring_rate         = this.spring_rate;
            group_obj.attributes.spring_motion_ratio = this.spring_motion_ratio;
            group_obj.attributes.damper_coeff        = this.damper_coeff;
            group_obj.attributes.damper_motion_ratio = this.damper_motion_ratio;
            group_obj.attributes.suspension_mass     = this.suspension_mass;
            
            % Save sub groups
            wheel_group = group_obj.add_group('wheel');
            this.wheel.save_hdf5(wheel_group);
            
        end
    end
    methods (Static)
        function obj = load_hdf5(group_obj)
        %LOAD_HDF5 Deserialize an HDF5 file
            
            % Load attributes
            spring_rate         = group_obj.attributes.spring_rate;
            spring_motion_ratio = group_obj.attributes.spring_motion_ratio;
            damper_coeff        = group_obj.attributes.damper_coeff;
            damper_motion_ratio = group_obj.attributes.damper_motion_ratio;
            suspension_mass     = group_obj.attributes.suspension_mass;
            
            % Load sub groups
            wheel_group = group_obj.get_group('wheel');
            wheel = chassis_dynamics.model.simple_car.wheel.load_hdf5(wheel_group);
            
            % Build object
            obj = chassis_dynamics.model.simple_car.corner(wheel,spring_rate,spring_motion_ratio,...
                                                           damper_coeff,damper_motion_ratio,...
                                                           suspension_mass);
            
        end
    end
end

