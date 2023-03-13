classdef solver < handle
%SOLVER Defines a solver
    properties (SetAccess = immutable)
        %MODEL A vehicle model object
        model
        %FORCE A road profile object
        force
    end
    properties (SetAccess = protected)
        %RESULT Simulation result
        result
    end
    methods
        function this = solver(model,force)
        %SOLVER Class constructor
        
            % Populate object
            this.model = model;
            this.force = force;
        
        end
    end
    methods (Abstract)
        %SOLVE Solve this thing
        solve(this)
    end
end

