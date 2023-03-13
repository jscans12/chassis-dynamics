classdef result < chassis_dynamics.sequence.template.result
%RESULT Result of a chassis simulation run
    methods
        % Constructor
        function this = result(time,data,map,desc)
        %RESULT Class constructor
            
            % Base class constructor
            this = this@chassis_dynamics.sequence.template.result(time,data,map,desc);
            
        end
        % Plotting
        function plot_cg_displacement(this)
        %PLOT_CG_DISPLACEMENT Create a time history plot of cg
        %motion
            
            % Plot
            figure;
            ax1 = subplot(3,1,1);
            plot(this.time,this.get_response_by_name('CG_Tz'),'b');
            grid on;
            ylabel('Heave (m)');
            title('CG Time History');
            ax2 = subplot(3,1,2);
            plot(this.time,this.get_response_by_name('CG_Rx'),'m');
            grid on;
            ylabel('Roll (rad)');
            ax3 = subplot(3,1,3);
            plot(this.time,this.get_response_by_name('CG_Ry'),'r');
            grid on;
            ylabel('Pitch (rad)');
            xlabel('Time (s)');
            linkaxes([ax1,ax2,ax3],'x');
            linkaxes([ax2,ax3],'y');
            
        end
        function plot_wheel_displacement(this)
        %PLOT_WHEEL_DISPLACEMENT Create a time history plot of wheel
        %motion
            
            % Plot
            figure;
            ax1 = subplot(4,1,1);
            plot(this.time,this.get_response_by_name('Wfl_Tz'),'b');
            grid on;
            ylabel('FL (m)');
            title('Wheel Bump Time History');
            ax2 = subplot(4,1,2);
            plot(this.time,this.get_response_by_name('Wfr_Tz'),'r');
            grid on;
            ylabel('FR (m)');
            ax3 = subplot(4,1,3);
            plot(this.time,this.get_response_by_name('Wrl_Tz'),'c');
            grid on;
            ylabel('RL (m)');
            ax4 = subplot(4,1,4);
            plot(this.time,this.get_response_by_name('Wrr_Tz'),'m');
            grid on;
            ylabel('RR (m)');
            xlabel('Time (s)');
            linkaxes([ax1,ax2,ax3,ax4],'xy');

        end
        % Serialization
        function save_hdf5(this,group_obj)
        %SAVE_HDF5 Save to an HDF5 group
            
            % Save attributes
            group_obj.add_dataset('time',this.time);
            group_obj.add_dataset('data',this.interpolant.Values);
            group_obj.add_dataset('map',this.map_);
            group_obj.add_dataset('desc',this.desc_);
            
        end
    end
    methods (Static)
        function obj = load_hdf5(group_obj)
        %LOAD_HDF5 Deserialize an HDF5 file
            
            % Get datasets
            time_dataset = group_obj.get_dataset('time');
            data_dataset = group_obj.get_dataset('data');
            map_dataset  = group_obj.get_dataset('map');
            desc_dataset = group_obj.get_dataset('desc');
            
            % Get data
            time = time_dataset.get_data;
            data = data_dataset.get_data;
            map  = map_dataset.get_data;
            desc = desc_dataset.get_data;
            
            % Build object
            obj = chassis_dynamics.sequence.roadload.result(time,data,map,desc);
            
        end
    end
end

