classdef force < handle
%FORCE Defines a force
    properties (Abstract)
        %MAP Force ID/DOF map
        map(:,2) int32
        %DESC Load set descriptions
        desc(:,1) cell
    end
    properties (Dependent)
        %NDOF Number of degrees of freedom
        ndof
    end
    properties (SetAccess = immutable)
        %INPUTS Inputs to the force object
        inputs;
    end
    methods
        %Getters and setters
        function ndof = get.ndof(this)
            ndof = size(this.map,1);
        end
        %Constructor
        function this = force(inputs)
        %FORCE Class constructor
            this.inputs = inputs;
        end
    end
    methods (Abstract)
        %GET_FofT Get force at a certain time step
        F = get_FofT(this,t)
    end
end

