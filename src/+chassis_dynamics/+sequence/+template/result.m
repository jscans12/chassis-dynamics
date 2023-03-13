classdef result < handle
%RESULT Result base class
    properties (SetAccess = immutable)
        %TIME Simulation time (s)
        time(:,1) double {mustBeReal, mustBeFinite}
    end
    properties (SetAccess = immutable, GetAccess = protected)
        %MAP_ Map of the response matrix
        map_(:,2) int32
        %DESC_ Description of response columns
        desc_(:,1) cell
        %INTERPOLANT Save gridded interpolant for fast lookup
        interpolant(1,1) griddedInterpolant
        %NDOF Number of degrees of freedom in output
        ndof int32
    end
    properties (Dependent)
        %MAP Map of the response matrix
        map
        %DESC Description of response columns
        desc
    end
    methods
        % Getters and setters
        function map = get.map(this)
            map = [this.map_ zeros(size(this.map_,1),1)
                   this.map_  ones(size(this.map_,1),1)];
        end
        function desc = get.desc(this)
            desc = [this.desc_
                    cellfun(@(x) sprintf('%s_d',x),this.desc_,'UniformOutput',false)];
        end
        % Constructor
        function this = result(time,data,map,desc)
        %RESULT Class constructor
        
            % Input checks
            if size(time,1) ~= size(data,1)
                error('Time and response must have the same number of rows');
            end
            if ~isequal(size(data,2)/2,size(map,1),size(desc,1))
                error('Response, map, and desc should all have same ndof');
            end
            
            % Set to class
            this.ndof        = size(data,2);
            this.time        = time;
            this.map_        = map;
            this.desc_       = desc;
            this.interpolant = griddedInterpolant({time,1:size(data,2)},data);
            
        end
        % Indexing
        function state = get_state_at_time(this,time)
        %GET_STATE_AT_TIME Get the state at a certain time or times
            
            % Simple linear interpolation
            state = this.interpolant({time,1:double(this.ndof)});
            
        end
        function response = get_response_by_name(this,name,time)
        %GET_RESPONSE_BY_NAME Get the response for a given name(s)
            
            % Find index
            name_index = strcmp(name,this.desc);
            if ~any(name_index)
                error('%s was not found\n',name);
            end
            
            % Defaults
            if nargin < 3
                response = this.interpolant.Values(:,name_index);
                return
            end
            
            % Get response
            response = this.interpolant({time,find(name_index)});
            
        end
    end
end

