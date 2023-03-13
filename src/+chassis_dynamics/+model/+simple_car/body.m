classdef body < handle
%BODY Defines a body, this is all unsprung mass upstream of the suspension
%(includes the frame and/or subframes)
    properties (SetAccess = immutable)
        %MASS Total mass of the body, not including wheels/suspension (kg)
        mass(1,1) double {mustBeReal, mustBeFinite}
        %MOI Mass moment of inertia of the body about the center of gravity
        %[Ixx,Iyy,Izz] (kg*m^2)
        moi(1,3) double {mustBeReal, mustBeFinite}
        %CG_LOC Center of gravity location in [X,Y,Z] at static ride (m)
        %
        %Origin defined as follows:
        %X - rear axle centerline
        %Y - vehicle centerline
        %Z - height above ground
        cg_loc(1,3) double {mustBeReal, mustBeFinite}
        %CP_LOC Center of pressure location in [X,Y,Z] at static ride (m)
        %Origin is same as CG definition
        cp_loc(1,3) double {mustBeReal, mustBeFinite}
        %SC Scaled aero coefficients, ind1 is drag, ind2 is lift. Scaled
        %coefficients are just aero coefficients (nondimensional)
        %multiplied by reference area. Example, in the case of drag it will
        %be Cd * frontal area. Units are technically (m^2)
        sc(1,2) double {mustBeReal, mustBeFinite}
    end
    methods
        % Constructor
        function this = body(mass,moi,cg_loc,cp_loc,sc)
        %BODY Class constructor
        
            % Set object properties
            this.mass   = mass;
            this.moi    = moi;
            this.cg_loc = cg_loc;
            this.cp_loc = cp_loc;
            this.sc     = sc;
            
        end
        % Serialization
        function save_hdf5(this,group_obj)
            
            % Save attributes
            group_obj.attributes.mass   = this.mass;
            group_obj.attributes.moi    = this.moi;
            group_obj.attributes.cg_loc = this.cg_loc;
            group_obj.attributes.cp_loc = this.cp_loc;
            group_obj.attributes.sc     = this.sc;
            
        end
    end
    methods (Static)
        function obj = load_hdf5(group_obj)
        %LOAD_HDF5 Deserialize an HDF5 file
            
            % Load attributes
            mass   = group_obj.attributes.mass;
            moi    = group_obj.attributes.moi;
            cg_loc = group_obj.attributes.cg_loc;
            cp_loc = group_obj.attributes.cp_loc;
            sc     = group_obj.attributes.sc;
            
            % Build object
            obj = chassis_dynamics.model.simple_car.body(mass,moi,cg_loc,cp_loc,sc);
            
        end
    end
end

