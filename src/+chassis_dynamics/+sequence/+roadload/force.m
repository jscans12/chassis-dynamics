classdef force < chassis_dynamics.sequence.template.force
%FORCE Forcing function for the simulation
    properties (Dependent)
        %MAP Force ID/DOF map
        map
        %DESC Load set descriptions
        desc
    end
    methods
        % Getters and setters
        function map = get.map(~)
            map = int32([111 3
                         112 3
                         113 3
                         114 3
                         211 3
                         212 3
                         213 3
                         214 3
                         301 1
                         301 2
                         301 3
                         301 4
                         301 5
                         301 6
                         311 3
                         312 3
                         313 3
                         314 3
                         401 1
                         401 2
                         401 3
                         401 4
                         401 5
                         401 6
                         411 3
                         412 3
                         413 3
                         414 3]);
        end
        function desc = get.desc(~)
            desc = {'Rfl_Tz'
                    'Rfr_Tz'
                    'Rrl_Tz'
                    'Rrr_Tz'
                    'Rfl_Vz'
                    'Rfr_Vz'
                    'Rrl_Vz'
                    'Rrr_Vz'
                    'CG_Ax'
                    'CG_Ay'
                    'CG_Az'
                    'CG_Alphax'
                    'CG_Alphay'
                    'CG_Alphaz'
                    'Wfl_Ax'
                    'Wfl_Ay'
                    'Wfl_Az'
                    'Wfr_Ax'
                    'Wfr_Ay'
                    'Wfr_Az'
                    'Wrl_Ax'
                    'Wrl_Ay'
                    'Wrl_Az'
                    'Wrr_Ax'
                    'Wrr_Ay'
                    'Wrr_Az'
                    'CG_Fx'
                    'CG_Fy'
                    'CG_Fz'
                    'CG_Mx'
                    'CG_My'
                    'CG_Mz'
                    'Wfl_Fx'
                    'Wfl_Fy'
                    'Wfl_Fz'
                    'Wfr_Fx'
                    'Wfr_Fy'
                    'Wfr_Fz'
                    'Wrl_Fx'
                    'Wrl_Fy'
                    'Wrl_Fz'
                    'Wrr_Fx'
                    'Wrr_Fy'
                    'Wrr_Fz'};
        end
        % Constructor
        function this = force(inputs)
        %FORCE Class constructor
            
            % Populate variables
            this = this@chassis_dynamics.sequence.template.force(inputs);
            
        end
        % FofT
        function F = get_FofT(this,t)
        %GET_FofT Get force vector at a given time step
        
            % Persistent variables for rate calc
            persistent t_prev
            if isempty(t_prev) || t <= 0
                t_prev = t;
            end
            persistent F_prev
            if isempty(F_prev) || t <= 0
                F_prev = zeros(size(this.map,1),1);
            end
            
            % Front and rear axle distance travelled
            dist   = this.inputs.road.get_input_at_t('distX',t);
            dist_f = dist + this.inputs.vehicle.wheelbase;
            dist_r = dist;
            
            % Interpolate to get road input
            Zr_fl = 0;
            Zr_fr = 0;
            Zr_rl = 0;
            Zr_rr = 0;
            if t >= 0
                Zr_fl = this.interp_Zr(dist_f,...
                                       'distZl',...
                                       this.inputs.vehicle.axle_f.corner_l.wheel.tire.radius);
                Zr_fr = this.interp_Zr(dist_f,...
                                       'distZr',...
                                       this.inputs.vehicle.axle_f.corner_r.wheel.tire.radius);
                Zr_rl = this.interp_Zr(dist_r,...
                                       'distZl',...
                                       this.inputs.vehicle.axle_r.corner_l.wheel.tire.radius);
                Zr_rr = this.interp_Zr(dist_r,...
                                       'distZr',...
                                       this.inputs.vehicle.axle_r.corner_r.wheel.tire.radius);
            end
            
            % Road input rate of change
            Zdr_fl = 0;
            Zdr_fr = 0;
            Zdr_rl = 0;
            Zdr_rr = 0;
            if t > t_prev
                dt = t - t_prev;
                Zdr_fl = (Zr_fl-F_prev(1))/dt;
                Zdr_fr = (Zr_fr-F_prev(2))/dt;
                Zdr_rl = (Zr_rl-F_prev(3))/dt;
                Zdr_rr = (Zr_rr-F_prev(4))/dt;
            end
            
            % Angles
            phi_f = atan((Zr_fl - Zr_fr)/this.inputs.vehicle.axle_f.track_width);
            phi_r = atan((Zr_rl - Zr_rr)/this.inputs.vehicle.axle_r.track_width);
            phi   = mean([phi_f,phi_r]);
            theta = atan((mean([Zr_rl,Zr_rr])-mean([Zr_fl,Zr_fr]))/this.inputs.vehicle.wheelbase);
            
            % Aero forces
            fX_aero  = 0;
            fY_aero  = 0;
            fZ_aero  = 0;
            mY_aero  = 0;
            mX_aero  = 0;
            mZ_aero  = 0;
            if t >= 0
                vel_curr = this.inputs.road.get_input_at_t('velX',t);
                fX_aero  = this.inputs.vehicle.body.sc(1) * this.inputs.environment.density * vel_curr^2 / 2;
                fY_aero  = 0;
                fZ_aero  = this.inputs.vehicle.body.sc(2) * this.inputs.environment.density * vel_curr^2 / 2;
                mY_aero  =  (this.inputs.vehicle.body.cp_loc(3) - this.inputs.vehicle.body.cg_loc(3)) * fX_aero ...
                           -(this.inputs.vehicle.body.cp_loc(1) - this.inputs.vehicle.body.cg_loc(1)) * fZ_aero;
                mX_aero  =  (this.inputs.vehicle.body.cp_loc(2) - this.inputs.vehicle.body.cg_loc(2)) * fZ_aero;
                mZ_aero  = -(this.inputs.vehicle.body.cp_loc(2) - this.inputs.vehicle.body.cg_loc(2)) * fX_aero;
            end
            
            % Driver requested acceleration
            aX_request     = 0;
            aY_request     = 0;
            if t >= 0
                aX_request     = this.inputs.road.get_input_at_t('accX',t);
                aY_request     = this.inputs.road.get_input_at_t('accY',t);
            end
            
            % Gravity vector
            aX_grav =  9.807 .* sin(theta);
            aY_grav = -9.807 .* sin(phi);
            aZ_grav = -9.807 .* cos(phi)   .* cos(theta);
            
            % Reaction force at tires needed to achieve request
            fX_reac = (aX_request - aX_grav)*this.inputs.vehicle.mass - fX_aero;
            fY_reac = (aY_request - aY_grav)*this.inputs.vehicle.mass - fY_aero;
            
            % Body forces
            CGz = this.inputs.vehicle.body.cp_loc(3);
            fX = 0;
            fY = 0;
            fZ = fZ_aero;
            mX = mX_aero + fY_reac*CGz;
            mY = mY_aero - fX_reac*CGz;
            mZ = mZ_aero;
            
            % Body accelerations
            aX      = aX_request;
            aY      = aY_request;
            aZ      = aZ_grav;
            
            % Output vector
            F = zeros(this.ndof,1);
            
            % Displacement vector
            F(1)  = Zr_fl;
            F(2)  = Zr_fr;
            F(3)  = Zr_rl;
            F(4)  = Zr_rr;
            
            % Velocity vector
            F(5) = Zdr_fl;
            F(6) = Zdr_fr;
            F(7) = Zdr_rl;
            F(8) = Zdr_rr;
            
            % Gravity vector
            F(9)  = aX;
            F(10) = aY;
            F([11,15:18]) = aZ;
            
            % Body force resultants
            F(19) = fX;
            F(20) = fY;
            F(21) = fZ;
            F(22) = mX;
            F(23) = mY;
            F(24) = mZ;
            
            % Maintain persistent variables
            F_prev = F;
            t_prev = t;
            
        end
        % Other
        function plot_state(this,dist_start,dist_end,ax)
        % Plot road in a range
            
            % Defaults
            if nargin < 4
                f = figure;
                ax = axes(f);
            end
            
            % Plot
            distX = this.inputs.road.get_input_at_t('distX');
            distZl   = this.inputs.road.get_input_at_t('distZl');
            distZr   = this.inputs.road.get_input_at_t('distZr');
            dist_indx = distX < dist_end ...
                      & distX > dist_start;
            n_dist = sum(dist_indx);
            if n_dist > 0
                line_fl = [distX(dist_indx) ,...
                           repmat(this.inputs.vehicle.axle_f.track_width/2,sum(dist_indx),1),...
                           distZl(dist_indx)];
                plot3(ax,line_fl(:,1),line_fl(:,2),line_fl(:,3),'k','linewidth',2);
                
                line_fr = [distX(dist_indx) ,...
                           repmat(-this.inputs.vehicle.axle_f.track_width/2,sum(dist_indx),1),...
                           distZr(dist_indx)];
                plot3(ax,line_fr(:,1),line_fr(:,2),line_fr(:,3),'k','linewidth',2);
                markers = unique(round(distX(dist_indx)));
                scatter3(ax,markers,zeros(size(markers)),zeros(size(markers)),...
                         50,'s','MarkerFaceColor','y','MarkerEdgeColor','y')
            end
            
        end
    end
    methods (Access = private)
        function Zr = interp_Zr(this,dist,output,Rw)
        %INTERP_ZR Get Zr for a wheel
        %
        %Assume a rigid circular body rolling along the surface. Also
        %assumes 11 discrete nodes along the tire/ground to find
        %intersection.
            
            % Use 11 nodes to represent envelope of wheel
            wheel_envelope = linspace(-Rw,Rw,11);
            
            % Project those points on to the ground
            ground_proj = this.inputs.road.get_input_at_d(output,dist+wheel_envelope)';
            
            % Project points on to wheel
            wheel_proj = Rw - sqrt(Rw^2 - wheel_envelope.^2);
            
            % Difference gives distance from wheel to ground
            wheel_dist = wheel_proj - ground_proj;
            
            % Get point at minimum
            [~,I] = min(wheel_dist);
            Zr = -wheel_dist(I);
            
        end
    end
end

