classdef environment < handle
%ENVIRONMENT Defines environmental conditions
    properties (SetAccess = immutable)
        %TEMPERATURE Ambient temperature (K)
        temperature(1,1) double {mustBeReal, mustBeFinite}
        %HUMIDITY Relative humidity (ratio)
        humidity(1,1) double {mustBeReal, mustBeFinite}
        %PRESSURE Ambient pressure (Pa)
        pressure(1,1) double {mustBeReal, mustBeFinite}
    end
    properties (Dependent)
        %DENSITY Air density (kg/m^3)
        density
    end
    methods
        % Getters and setters
        function rho = get.density(this)
        % Calculates density of air
        %  Output:  rho = air density [kg/m3]
        %
        %  Refs:
        % 1)'Equation for the Determination of the Density of Moist Air' P. Giacomo  Metrologia 18, 33-40 (1982)
        % 2)'Equation for the Determination of the Density of Moist Air' R. S. Davis Metrologia 29, 67-70 (1992)
        %
        % ver 1.0   06/10/2006   Jose Luis Prego Borges (Sensor & System Group, Universitat Politecnica de Catalunya)
        % ver 1.1   05/02/2007   Richard Signell (rsignell@usgs.gov)  Vectorized
        % ver 1.2   06/04/2020   John Scanlon - modifications for this class
        
            % Convert class properties
            T  = this.temperature;
            hr = this.humidity * 100;
            p  = this.pressure;
        
            % Conversions
            T0 = 273.16;        % Triple point of water (aprox. 0ºC)
            t  = T - T0;        % Ambient temperature in ºC
            
            % 1) Coefficients values
            R  =  8.314510;      % Molar ideal gas constant   [J/(mol.ºK)]
            Mv = 18.015e-3;      % Molar mass of water vapour [kg/mol]
            Ma = 28.9635e-3;     % Molar mass of dry air      [kg/mol]
            A  =  1.2378847e-5;  % [ºK^-2]
            B  = -1.9121316e-2;  % [ºK^-1]
            C  = 33.93711047;    %
            D  = -6.3431645e3;   % [ºK]
            a0 =  1.58123e-6;    % [ºK/Pa]
            a1 = -2.9331e-8;     % [1/Pa]
            a2 =  1.1043e-10;    % [1/(ºK.Pa)]
            b0 =  5.707e-6;      % [ºK/Pa]
            b1 = -2.051e-8;      % [1/Pa]
            c0 =  1.9898e-4;     % [ºK/Pa]
            c1 = -2.376e-6;      % [1/Pa]
            d  =  1.83e-11;      % [ºK^2/Pa^2]
            e  = -0.765e-8;      % [ºK^2/Pa^2]
            
            % 2) Calculation of the saturation vapour pressure at ambient temperature, in [Pa]
            psv = exp(A.*(T.^2) + B.*T + C + D./T);   % [Pa]
            
            % 3) Calculation of the enhancement factor at ambient temperature and pressure
            fpt = 1.00062 + (3.14e-8)*p + (5.6e-7)*(t.^2);
            
            % 4) Calculation of the mole fraction of water vapour
            xv = hr.*fpt.*psv.*(1./p)*(10^-2);
            
            % 5) Calculation of the compressibility factor of air
            Z = 1 - ((p./T).*(a0 + a1*t + a2*(t.^2) + (b0+b1*t).*xv + (c0+c1*t).*(xv.^2))) + ((p.^2/T.^2).*(d + e.*(xv.^2)));
            
            % 6) Final calculation of the air density in [kg/m^3]
            rho = (p.*Ma./(Z.*R.*T)).*(1 - xv.*(1-Mv./Ma));
            
        end
        % Constructor
        function this = environment(temperature,humidity,pressure)
        %ENVIRONMENT Class constuctor
        
            this.temperature = temperature;
            this.humidity    = humidity;
            this.pressure    = pressure;
            
        end
        % Serialization
        function save_hdf5(this,filename)
        %SAVE_HDF5 Save to an HDF5 file
            
            % Create file
            file_obj = h5io.file(filename,'w');
            
            % Save attributes
            file_obj.attributes.temperature = this.temperature;
            file_obj.attributes.humidity    = this.humidity;
            file_obj.attributes.pressure    = this.pressure;
            
        end
    end
    methods (Static)
        function obj = load_hdf5(filename)
        %LOAD_HDF5 Deserialize an HDF5 file
            
            % Open file
            file_obj = h5io.file(filename,'r');
            
            % Load attributes
            temperature = file_obj.attributes.temperature;
            humidity    = file_obj.attributes.humidity;
            pressure    = file_obj.attributes.pressure;
            
            % Build object
            obj = chassis_dynamics.model.environment(temperature,humidity,pressure);
            
        end
    end
end

