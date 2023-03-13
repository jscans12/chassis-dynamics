classdef assembly < handle
%ASSEMBLY Full vehicle assembly
    properties (SetAccess = immutable)
        %BODY Body object
        body chassis_dynamics.model.simple_car.body
        %AXLE_F Front axle object
        axle_f chassis_dynamics.model.simple_car.axle
        %AXLE_R Rear axle object
        axle_r chassis_dynamics.model.simple_car.axle
        %WHEELBASE Distance between front and rear axle centerlines (m)
        wheelbase(1,1) double {mustBeReal, mustBeFinite}
    end
    properties (Dependent)
        %SPRUNG_MASS Total sprung mass of the car (kg)
        sprung_mass
        %MASS Total mass of the car (kg)
        mass
    end
    methods
        % Getters and setters
        function sprung_mass = get.sprung_mass(this)
            sprung_mass = this.body.mass ...
                        + this.axle_f.corner_l.sprung_mass ...
                        + this.axle_f.corner_r.sprung_mass ...
                        + this.axle_r.corner_l.sprung_mass ...
                        + this.axle_r.corner_r.sprung_mass;
        end
        function mass = get.mass(this)
            mass = this.body.mass ...
                 + this.axle_f.corner_l.sprung_mass ...
                 + this.axle_f.corner_r.sprung_mass ...
                 + this.axle_r.corner_l.sprung_mass ...
                 + this.axle_r.corner_r.sprung_mass ...
                 + this.axle_f.corner_l.unsprung_mass ...
                 + this.axle_f.corner_r.unsprung_mass ...
                 + this.axle_r.corner_l.unsprung_mass ...
                 + this.axle_r.corner_r.unsprung_mass;
        end
        % Constructor
        function this = assembly(body,axle_f,axle_r,wheelbase)
        %ASSEMBLY Class constructor
        
            % Set object properties
            this.body               = body;
            this.axle_f             = axle_f;
            this.axle_r             = axle_r;
            this.wheelbase          = wheelbase;
        
        end
        % Serialization
        function save_hdf5(this,filename)
        %SAVE_HDF5 Save to an HDF5 file
            
            % Create file
            file_obj = h5io.file(filename,'w');
            
            % Save attributes
            file_obj.attributes.wheelbase = this.wheelbase;
            
            % Save sub groups
            body_group = file_obj.add_group('body');
            this.body.save_hdf5(body_group);
            axle_f_group = file_obj.add_group('axle_f');
            this.axle_f.save_hdf5(axle_f_group);
            axle_r_group = file_obj.add_group('axle_r');
            this.axle_r.save_hdf5(axle_r_group);
            
        end
    end
    methods (Static)
        function obj = load_hdf5(filename)
        %LOAD_HDF5 Deserialize an HDF5 file
            
            % Open file
            file_obj = h5io.file(filename,'r');
            
            % Load attributes
            wheelbase = file_obj.attributes.wheelbase;
            
            % Load sub groups
            body_group = file_obj.get_group('body');
            body = chassis_dynamics.model.simple_car.body.load_hdf5(body_group);
            axle_f_group = file_obj.get_group('axle_f');
            axle_f = chassis_dynamics.model.simple_car.axle.load_hdf5(axle_f_group);
            axle_r_group = file_obj.get_group('axle_r');
            axle_r = chassis_dynamics.model.simple_car.axle.load_hdf5(axle_r_group);
            
            % Build object
            obj = chassis_dynamics.model.simple_car.assembly(body,axle_f,axle_r,wheelbase);
            
        end
    end
end

