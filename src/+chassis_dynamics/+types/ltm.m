classdef ltm < handle
%LTM Defines a load transformation matrix
    properties
        %NAME The LTM name
        name char
        %MAP ID and DOF map for the LTM
        map(:,3) int32
        %DESC Description for each field
        desc(:,1) cell
        %DATA Data stored for the LTM
        data double
    end
    methods
        function this = ltm(name,map,desc,data)
        %LTM Class constructor
            
            % Data checks
            if size(data,1) ~= size(map,1)
                error('Data must have the same amount of columns as map rows');
            end
            if size(desc,1) ~= size(map,1)
                error('Desc must have the same amount of rows as map rows');
            end
        
            % Set data
            this.name = name;
            this.map  = map;
            this.desc = desc;
            this.data = data;
            
        end
        function obj = vertcat(this,varargin)
        %VERTCAT Vertical concatenation
            
            % Copy this object
            obj = this.COPY;
        
            % Concatenate each input
            for i = 1:nargin-1
                
                if ~isa(varargin{i},'chassis_dynamics.types.ltm')
                    error('Error in vertcat, must be LTMs');
                end
                
                obj.name = sprintf('%s:%s',obj.name,varargin{i}.name);
                obj.map  = [obj.map; varargin{i}.map];
                obj.desc = [obj.desc;varargin{i}.desc];
                obj.data = [obj.data;varargin{i}.data];
                
            end
            
        end
    end
    methods (Access = private)
        function obj = COPY(this)
            obj = chassis_dynamics.types.ltm(this.name,this.map,this.desc,this.data);
        end
    end
end

