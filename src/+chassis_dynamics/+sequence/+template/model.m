classdef model < handle
%MODEL Defines a model
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
        function this = model(inputs)
        %MODEL Class constructor
        
            % Set object properties
            this.inputs = inputs;
        
        end
    end
end

