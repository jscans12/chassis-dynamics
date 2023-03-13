classdef residual < chassis_dynamics.sequence.template.model
%RESIDUAL A simple full-vehicle chassis model
%
%This is a MATLAB implementation of the matrix formulation for a passive
%full-vehicle model, based loosely on the paper:
%
%Simulation and Analysis of Full Car Model for Various Road Profile on a
%Analytically Validated MATLAB/SIMULINK Model
%
%A. Mitra, N. Benerjee, H.A. Khalane, M.A. Sonawane, D.R. Joshi, G.R. Bagul
%
%https://pdfs.semanticscholar.org/3276/000fb6203692f697cd326148a0e52450fcc3.pdf
%
%Warning: the paper is frought with minor errors, and also doesn't account
%for tire damping or antiroll bars. Those are implemented here.
    properties
        %WHEEL_CONTACT Wheel contact bits, true for in-contact and false
        %for out of contact. Order is FL,FR,RL,RR.
        wheel_contact(1,4) logical = true(1,4);
    end
    properties (Dependent)
        %MAP Map of each DOF
        map
        %DESC Description for each DOF
        desc
        %GRID Node definitions
        GRID
        %KGG Global stiffness matrix
        KGG
        %MGG Global mass matrix
        MGG
        %BGG Global damping matrix
        BGG
        %KAA Modal stiffness matrix
        KAA
        %KAA Modal mass matrix
        MAA
        %KAA Modal damping matrix
        BAA
        %PHI Mode shapes
        phi
    end
    methods
        % Getters and setters
        function map = get.map(~)
            map = int32([1   1
                         1   2
                         1   3
                         1   4
                         1   5
                         1   6
                         11  1
                         11  2
                         11  3
                         12  1
                         12  2
                         12  3
                         13  1
                         13  2
                         13  3
                         14  1
                         14  2
                         14  3]);
        end
        function desc = get.desc(~)
            desc = {'CG_Tx'
                    'CG_Ty'
                    'CG_Tz'
                    'CG_Rx'
                    'CG_Ry'
                    'CG_Rz'
                    'Wfl_Tx'
                    'Wfl_Ty'
                    'Wfl_Tz'
                    'Wfr_Tx'
                    'Wfr_Ty'
                    'Wfr_Tz'
                    'Wrl_Tx'
                    'Wrl_Ty'
                    'Wrl_Tz'
                    'Wrr_Tx'
                    'Wrr_Ty'
                    'Wrr_Tz'};
        end
        function GRID = get.GRID(this)
            
            GRID = cell2table(cell(0,6),'VariableNames',{'ID','CP','X1','X2','X3','CD'});
            GRID = [GRID;{ 1,0,this.inputs.body.cg_loc(1),this.inputs.body.cg_loc(2),this.inputs.body.cg_loc(3),0}];
            GRID = [GRID;{11,0,this.inputs.wheelbase, this.inputs.axle_f.track_width/2,this.inputs.axle_f.corner_l.wheel.tire.radius,0}];
            GRID = [GRID;{12,0,this.inputs.wheelbase,-this.inputs.axle_f.track_width/2,this.inputs.axle_f.corner_r.wheel.tire.radius,0}];
            GRID = [GRID;{13,0,                    0, this.inputs.axle_r.track_width/2,this.inputs.axle_r.corner_l.wheel.tire.radius,0}];
            GRID = [GRID;{14,0,                    0,-this.inputs.axle_r.track_width/2,this.inputs.axle_r.corner_r.wheel.tire.radius,0}];
            
        end
        function KGG = get.KGG(this)
        %Construct the stiffness matrix
            
            % Mass properties
            [a,b,cf,df,cr,dr] = get_abcd(this);
            
            % Prototype
            KGG = sparse(this.ndof,this.ndof);
            
            % Get indices from map
            persistent iMap
            if isempty(iMap)
                map_ = [1  3
                        1  4
                        1  5
                        11 3
                        12 3
                        13 3
                        14 3
                        11 1
                        11 2
                        12 1
                        12 2
                        13 1
                        13 2
                        14 1
                        14 2];
                iMap = chassis_dynamics.tools.map_index(map_,this.map);
            end
            
            % Row 3
            KGG(iMap(1),iMap(1)) = ( this.inputs.axle_f.corner_l.wheel_spring_rate     +this.inputs.axle_f.corner_r.wheel_spring_rate     +this.inputs.axle_r.corner_l.wheel_spring_rate     +this.inputs.axle_r.corner_r.wheel_spring_rate     );
            KGG(iMap(1),iMap(2)) = ( this.inputs.axle_f.corner_l.wheel_spring_rate  *cf-this.inputs.axle_f.corner_r.wheel_spring_rate  *df+this.inputs.axle_r.corner_l.wheel_spring_rate  *cr-this.inputs.axle_r.corner_r.wheel_spring_rate  *dr);
            KGG(iMap(1),iMap(3)) = (-this.inputs.axle_f.corner_l.wheel_spring_rate  *a -this.inputs.axle_f.corner_r.wheel_spring_rate  *a +this.inputs.axle_r.corner_l.wheel_spring_rate  *b +this.inputs.axle_r.corner_r.wheel_spring_rate  *b );
            KGG(iMap(1),iMap(4)) = (-this.inputs.axle_f.corner_l.wheel_spring_rate);
            KGG(iMap(1),iMap(5)) = (-this.inputs.axle_f.corner_r.wheel_spring_rate);
            KGG(iMap(1),iMap(6)) = (-this.inputs.axle_r.corner_l.wheel_spring_rate);
            KGG(iMap(1),iMap(7)) = (-this.inputs.axle_r.corner_r.wheel_spring_rate);
            
            % Row 4
            KGG(iMap(2),iMap(1)) = ( this.inputs.axle_f.corner_l.wheel_spring_rate  *cf   -this.inputs.axle_f.corner_r.wheel_spring_rate  *df   +this.inputs.axle_r.corner_l.wheel_spring_rate  *cr   -this.inputs.axle_r.corner_r.wheel_spring_rate  *dr   +this.inputs.axle_f.antiroll_rate*  (cf   -df   )+this.inputs.axle_r.antiroll_rate*  (cr   -dr   ));
            KGG(iMap(2),iMap(2)) = ( this.inputs.axle_f.corner_l.wheel_spring_rate  *cf*cf+this.inputs.axle_f.corner_r.wheel_spring_rate  *df*df+this.inputs.axle_r.corner_l.wheel_spring_rate  *cr*cr+this.inputs.axle_r.corner_r.wheel_spring_rate  *dr*dr+this.inputs.axle_f.antiroll_rate*  (cf*cf+df*df)+this.inputs.axle_r.antiroll_rate*  (cr*cr+dr*dr));
            KGG(iMap(2),iMap(3)) = (-this.inputs.axle_f.corner_l.wheel_spring_rate  *a*cf +this.inputs.axle_f.corner_r.wheel_spring_rate  *a*df +this.inputs.axle_r.corner_l.wheel_spring_rate  *b*cr -this.inputs.axle_r.corner_r.wheel_spring_rate  *b *dr-this.inputs.axle_f.antiroll_rate*a*(cf   -df   )+this.inputs.axle_r.antiroll_rate*b*(cr   -dr   ));
            KGG(iMap(2),iMap(4)) = (-this.inputs.axle_f.corner_l.wheel_spring_rate  *cf   -this.inputs.axle_f.antiroll_rate*(cf+df));
            KGG(iMap(2),iMap(5)) = ( this.inputs.axle_f.corner_r.wheel_spring_rate  *df   +this.inputs.axle_f.antiroll_rate*(cf+df));
            KGG(iMap(2),iMap(6)) = (-this.inputs.axle_r.corner_l.wheel_spring_rate  *cr   -this.inputs.axle_r.antiroll_rate*(cr+dr));
            KGG(iMap(2),iMap(7)) = ( this.inputs.axle_r.corner_r.wheel_spring_rate  *dr   +this.inputs.axle_r.antiroll_rate*(cr+dr));
            
            % Row 5
            KGG(iMap(3),iMap(1)) = (-this.inputs.axle_f.corner_l.wheel_spring_rate  *a   -this.inputs.axle_f.corner_r.wheel_spring_rate  *a   +this.inputs.axle_r.corner_l.wheel_spring_rate  *b   +this.inputs.axle_r.corner_r.wheel_spring_rate  *b   );
            KGG(iMap(3),iMap(2)) = (-this.inputs.axle_f.corner_l.wheel_spring_rate  *a*cf+this.inputs.axle_f.corner_r.wheel_spring_rate  *a*df+this.inputs.axle_r.corner_l.wheel_spring_rate  *b*cr-this.inputs.axle_r.corner_r.wheel_spring_rate  *b*dr);
            KGG(iMap(3),iMap(3)) = ( this.inputs.axle_f.corner_l.wheel_spring_rate  *a*a +this.inputs.axle_f.corner_r.wheel_spring_rate  *a*a +this.inputs.axle_r.corner_l.wheel_spring_rate  *b*b +this.inputs.axle_r.corner_r.wheel_spring_rate  *b*b );
            KGG(iMap(3),iMap(4)) = ( this.inputs.axle_f.corner_l.wheel_spring_rate  *a);
            KGG(iMap(3),iMap(5)) = ( this.inputs.axle_f.corner_r.wheel_spring_rate  *a);
            KGG(iMap(3),iMap(6)) = (-this.inputs.axle_r.corner_l.wheel_spring_rate  *b);
            KGG(iMap(3),iMap(7)) = (-this.inputs.axle_r.corner_r.wheel_spring_rate  *b);
            
            % Row 9
            if this.wheel_contact(1)
                Kt_fl_ = this.inputs.axle_f.corner_l.wheel.tire.spring_rate;
            else
                Kt_fl_ = 0;
            end
            KGG(iMap(4),iMap(1)) = (-this.inputs.axle_f.corner_l.wheel_spring_rate);
            KGG(iMap(4),iMap(2)) = (-this.inputs.axle_f.corner_l.wheel_spring_rate  *cf-this.inputs.axle_f.antiroll_rate*(cf+df));
            KGG(iMap(4),iMap(3)) = ( this.inputs.axle_f.corner_l.wheel_spring_rate  *a);
            KGG(iMap(4),iMap(4)) = ( this.inputs.axle_f.corner_l.wheel_spring_rate  +Kt_fl_+this.inputs.axle_f.antiroll_rate);
            KGG(iMap(4),iMap(5)) = (-this.inputs.axle_f.antiroll_rate);
            KGG(iMap(4),iMap(6)) = 0;
            KGG(iMap(4),iMap(7)) = 0;
            
            % Row 12
            if this.wheel_contact(2)
                Kt_fr_ = this.inputs.axle_f.corner_r.wheel.tire.spring_rate;
            else
                Kt_fr_ = 0;
            end
            KGG(iMap(5),iMap(1)) = (-this.inputs.axle_f.corner_r.wheel_spring_rate);
            KGG(iMap(5),iMap(2)) = ( this.inputs.axle_f.corner_r.wheel_spring_rate  *df+this.inputs.axle_f.antiroll_rate*(cf+df));
            KGG(iMap(5),iMap(3)) = ( this.inputs.axle_f.corner_r.wheel_spring_rate  *a);
            KGG(iMap(5),iMap(4)) = (-this.inputs.axle_f.antiroll_rate);
            KGG(iMap(5),iMap(5)) = ( this.inputs.axle_f.corner_r.wheel_spring_rate  +Kt_fr_+this.inputs.axle_f.antiroll_rate);
            KGG(iMap(5),iMap(6)) = 0;
            KGG(iMap(5),iMap(7)) = 0;
            
            % Row 15
            if this.wheel_contact(3)
                Kt_rl_ = this.inputs.axle_r.corner_l.wheel.tire.spring_rate;
            else
                Kt_rl_ = 0;
            end
            KGG(iMap(6),iMap(1)) = (-this.inputs.axle_r.corner_l.wheel_spring_rate);
            KGG(iMap(6),iMap(2)) = (-this.inputs.axle_r.corner_l.wheel_spring_rate  *cr-this.inputs.axle_r.antiroll_rate*(cr+dr));
            KGG(iMap(6),iMap(3)) = (-this.inputs.axle_r.corner_l.wheel_spring_rate  *b);
            KGG(iMap(6),iMap(4)) = 0;
            KGG(iMap(6),iMap(5)) = 0;
            KGG(iMap(6),iMap(6)) = ( this.inputs.axle_r.corner_l.wheel_spring_rate  +Kt_rl_+this.inputs.axle_r.antiroll_rate);
            KGG(iMap(6),iMap(7)) = (-this.inputs.axle_r.antiroll_rate);
            
            % Row 18
            if this.wheel_contact(4)
                Kt_rr_ = this.inputs.axle_r.corner_r.wheel.tire.spring_rate;
            else
                Kt_rr_ = 0;
            end
            KGG(iMap(7),iMap(1)) = (-this.inputs.axle_r.corner_r.wheel_spring_rate);
            KGG(iMap(7),iMap(2)) = ( this.inputs.axle_r.corner_r.wheel_spring_rate  *dr+this.inputs.axle_r.antiroll_rate*(cr+dr));
            KGG(iMap(7),iMap(3)) = (-this.inputs.axle_r.corner_r.wheel_spring_rate  *b);
            KGG(iMap(7),iMap(4)) = 0;
            KGG(iMap(7),iMap(5)) = 0;
            KGG(iMap(7),iMap(6)) = (-this.inputs.axle_r.antiroll_rate);
            KGG(iMap(7),iMap(7)) = ( this.inputs.axle_r.corner_r.wheel_spring_rate  +Kt_rr_+this.inputs.axle_r.antiroll_rate);
            
        end
        function BGG = get.BGG(this)
        %Construct the damping matrix
        
            % Mass properties
            [a,b,cf,df,cr,dr] = get_abcd(this);
            
            % Prototype
            BGG = sparse(this.ndof,this.ndof);
            
            % Get indices from map
            persistent iMap
            if isempty(iMap)
                map_ = [1  3
                        1  4
                        1  5
                        11 3
                        12 3
                        13 3
                        14 3];
                iMap = chassis_dynamics.tools.map_index(map_,this.map);
            end
        
            % Row 3
            BGG(iMap(1),iMap(1)) = ( this.inputs.axle_f.corner_l.wheel_damping_coeff   +this.inputs.axle_f.corner_r.wheel_damping_coeff   +this.inputs.axle_r.corner_l.wheel_damping_coeff   +this.inputs.axle_r.corner_r.wheel_damping_coeff   );
            BGG(iMap(1),iMap(2)) = ( this.inputs.axle_f.corner_l.wheel_damping_coeff*cf-this.inputs.axle_f.corner_r.wheel_damping_coeff*df+this.inputs.axle_r.corner_l.wheel_damping_coeff*cr-this.inputs.axle_r.corner_r.wheel_damping_coeff*dr);
            BGG(iMap(1),iMap(3)) = (-this.inputs.axle_f.corner_l.wheel_damping_coeff*a -this.inputs.axle_f.corner_r.wheel_damping_coeff*a +this.inputs.axle_r.corner_l.wheel_damping_coeff*b +this.inputs.axle_r.corner_r.wheel_damping_coeff*b );
            BGG(iMap(1),iMap(4)) =  -this.inputs.axle_f.corner_l.wheel_damping_coeff;
            BGG(iMap(1),iMap(5)) =  -this.inputs.axle_f.corner_r.wheel_damping_coeff;
            BGG(iMap(1),iMap(6)) =  -this.inputs.axle_r.corner_l.wheel_damping_coeff;
            BGG(iMap(1),iMap(7)) =  -this.inputs.axle_r.corner_r.wheel_damping_coeff;
            
            % Row 4
            BGG(iMap(2),iMap(1)) = ( this.inputs.axle_f.corner_l.wheel_damping_coeff*cf   -this.inputs.axle_f.corner_r.wheel_damping_coeff*df   +this.inputs.axle_r.corner_l.wheel_damping_coeff*cr   -this.inputs.axle_r.corner_r.wheel_damping_coeff*dr   );
            BGG(iMap(2),iMap(2)) = ( this.inputs.axle_f.corner_l.wheel_damping_coeff*cf*cf+this.inputs.axle_f.corner_r.wheel_damping_coeff*df*df+this.inputs.axle_r.corner_l.wheel_damping_coeff*cr*cr+this.inputs.axle_r.corner_r.wheel_damping_coeff*dr*dr);
            BGG(iMap(2),iMap(3)) = (-this.inputs.axle_f.corner_l.wheel_damping_coeff*a*cf +this.inputs.axle_f.corner_r.wheel_damping_coeff*a*df +this.inputs.axle_r.corner_l.wheel_damping_coeff*b*cr -this.inputs.axle_r.corner_r.wheel_damping_coeff*b *dr);
            BGG(iMap(2),iMap(4)) = (-this.inputs.axle_f.corner_l.wheel_damping_coeff*cf);
            BGG(iMap(2),iMap(5)) = ( this.inputs.axle_f.corner_r.wheel_damping_coeff*df);
            BGG(iMap(2),iMap(6)) = (-this.inputs.axle_r.corner_l.wheel_damping_coeff*cr);
            BGG(iMap(2),iMap(7)) = ( this.inputs.axle_r.corner_r.wheel_damping_coeff*dr);
            
            % Row 5
            BGG(iMap(3),iMap(1)) = (-this.inputs.axle_f.corner_l.wheel_damping_coeff*a   -this.inputs.axle_f.corner_r.wheel_damping_coeff*a   +this.inputs.axle_r.corner_l.wheel_damping_coeff*b   +this.inputs.axle_r.corner_r.wheel_damping_coeff*b   );
            BGG(iMap(3),iMap(2)) = (-this.inputs.axle_f.corner_l.wheel_damping_coeff*a*cf+this.inputs.axle_f.corner_r.wheel_damping_coeff*a*df+this.inputs.axle_r.corner_l.wheel_damping_coeff*b*cr-this.inputs.axle_r.corner_r.wheel_damping_coeff*b*dr);
            BGG(iMap(3),iMap(3)) = ( this.inputs.axle_f.corner_l.wheel_damping_coeff*a*a +this.inputs.axle_f.corner_r.wheel_damping_coeff*a*a +this.inputs.axle_r.corner_l.wheel_damping_coeff*b*b +this.inputs.axle_r.corner_r.wheel_damping_coeff*b*b );
            BGG(iMap(3),iMap(4)) = ( this.inputs.axle_f.corner_l.wheel_damping_coeff*a);
            BGG(iMap(3),iMap(5)) = ( this.inputs.axle_f.corner_r.wheel_damping_coeff*a);
            BGG(iMap(3),iMap(6)) = (-this.inputs.axle_r.corner_l.wheel_damping_coeff*b);
            BGG(iMap(3),iMap(7)) = (-this.inputs.axle_r.corner_r.wheel_damping_coeff*b);
            
            % Row 9
            if this.wheel_contact(1)
                Bt_fl_ = this.inputs.axle_f.corner_l.wheel.tire.damping_coeff;
            else
                Bt_fl_ = 0;
            end
            BGG(iMap(4),iMap(1)) =  -this.inputs.axle_f.corner_l.wheel_damping_coeff;
            BGG(iMap(4),iMap(2)) = (-this.inputs.axle_f.corner_l.wheel_damping_coeff*cf);
            BGG(iMap(4),iMap(3)) = ( this.inputs.axle_f.corner_l.wheel_damping_coeff*a);
            BGG(iMap(4),iMap(4)) =   this.inputs.axle_f.corner_l.wheel_damping_coeff+Bt_fl_;
            BGG(iMap(4),iMap(5)) = 0;
            BGG(iMap(4),iMap(6)) = 0;
            BGG(iMap(4),iMap(7)) = 0;
            
            % Row 12
            if this.wheel_contact(2)
                Bt_fr_ = this.inputs.axle_f.corner_r.wheel.tire.damping_coeff;
            else
                Bt_fr_ = 0;
            end
            BGG(iMap(5),iMap(1)) =  -this.inputs.axle_f.corner_r.wheel_damping_coeff;
            BGG(iMap(5),iMap(2)) = ( this.inputs.axle_f.corner_r.wheel_damping_coeff*df);
            BGG(iMap(5),iMap(3)) = ( this.inputs.axle_f.corner_r.wheel_damping_coeff*a);
            BGG(iMap(5),iMap(4)) = 0;
            BGG(iMap(5),iMap(5)) =   this.inputs.axle_f.corner_r.wheel_damping_coeff+Bt_fr_;
            BGG(iMap(5),iMap(6)) = 0;
            BGG(iMap(5),iMap(7)) = 0;
            
            % Row 15
            if this.wheel_contact(3)
                Bt_rl_ = this.inputs.axle_r.corner_l.wheel.tire.damping_coeff;
            else
                Bt_rl_ = 0;
            end
            BGG(iMap(6),iMap(1)) =  -this.inputs.axle_r.corner_l.wheel_damping_coeff;
            BGG(iMap(6),iMap(2)) = (-this.inputs.axle_r.corner_l.wheel_damping_coeff*cr);
            BGG(iMap(6),iMap(3)) = (-this.inputs.axle_r.corner_l.wheel_damping_coeff*b);
            BGG(iMap(6),iMap(4)) = 0;
            BGG(iMap(6),iMap(5)) = 0;
            BGG(iMap(6),iMap(6)) =   this.inputs.axle_r.corner_l.wheel_damping_coeff+Bt_rl_;
            BGG(iMap(6),iMap(7)) = 0;
            
            % Row 18
            if this.wheel_contact(4)
                Bt_rr_ = this.inputs.axle_r.corner_r.wheel.tire.damping_coeff;
            else
                Bt_rr_ = 0;
            end
            BGG(iMap(7),iMap(1)) =  -this.inputs.axle_r.corner_r.wheel_damping_coeff;
            BGG(iMap(7),iMap(2)) = ( this.inputs.axle_r.corner_r.wheel_damping_coeff*dr);
            BGG(iMap(7),iMap(3)) = (-this.inputs.axle_r.corner_r.wheel_damping_coeff*b);
            BGG(iMap(7),iMap(4)) = 0;
            BGG(iMap(7),iMap(5)) = 0;
            BGG(iMap(7),iMap(6)) = 0;
            BGG(iMap(7),iMap(7)) =   this.inputs.axle_r.corner_r.wheel_damping_coeff+Bt_rr_;
        
        end
        function MGG = get.MGG(this)
        %Construct the mass matrix
            
            % Memoize
            persistent M_res_p
            if ~isempty(M_res_p)
                MGG = M_res_p;
                return;
            end
            
            % Prototype
            MGG = sparse(this.ndof,this.ndof);
            
            % Sprung
            map1 = [1 1
                    1 2
                    1 3
                    1 4
                    1 5
                    1 6];
            i1   = chassis_dynamics.tools.map_index(map1,this.map);
            MGG(i1(1),i1(1)) = this.inputs.sprung_mass;
            MGG(i1(2),i1(2)) = this.inputs.sprung_mass;
            MGG(i1(3),i1(3)) = this.inputs.sprung_mass;
            MGG(i1(4),i1(4)) = this.inputs.body.moi(1);
            MGG(i1(5),i1(5)) = this.inputs.body.moi(2);
            MGG(i1(6),i1(6)) = this.inputs.body.moi(3);
            
            % FL unsprung
            map11 = [11 1
                     11 2
                     11 3];
            i11   = chassis_dynamics.tools.map_index(map11,this.map);
            MGG(i11(1),i11(1)) = this.inputs.axle_f.corner_l.unsprung_mass;
            MGG(i11(2),i11(2)) = this.inputs.axle_f.corner_l.unsprung_mass;
            MGG(i11(3),i11(3)) = this.inputs.axle_f.corner_l.unsprung_mass;
            
            % FR unsprung
            map12 = [12 1
                     12 2
                     12 3];
            i12   = chassis_dynamics.tools.map_index(map12,this.map);
            MGG(i12(1),i12(1)) = this.inputs.axle_f.corner_r.unsprung_mass;
            MGG(i12(2),i12(2)) = this.inputs.axle_f.corner_r.unsprung_mass;
            MGG(i12(3),i12(3)) = this.inputs.axle_f.corner_r.unsprung_mass;
            
            % RL unsprung
            map13 = [13 1
                     13 2
                     13 3];
            i13   = chassis_dynamics.tools.map_index(map13,this.map);
            MGG(i13(1),i13(1)) = this.inputs.axle_r.corner_l.unsprung_mass;
            MGG(i13(2),i13(2)) = this.inputs.axle_r.corner_l.unsprung_mass;
            MGG(i13(3),i13(3)) = this.inputs.axle_r.corner_l.unsprung_mass;
            
            % RR unsprung
            map14 = [14 1
                     14 2
                     14 3];
            i14   = chassis_dynamics.tools.map_index(map14,this.map);
            MGG(i14(1),i14(1)) = this.inputs.axle_r.corner_r.unsprung_mass;
            MGG(i14(2),i14(2)) = this.inputs.axle_r.corner_r.unsprung_mass;
            MGG(i14(3),i14(3)) = this.inputs.axle_r.corner_r.unsprung_mass;
            
            % Set persistent variable
            M_res_p = MGG;
            
        end
        function KAA = get.KAA(this)
            [~,KAA] = eigs(this.KGG,...
                           this.MGG,...
                           this.ndof);
        end
        function MAA = get.MAA(this) 
            MAA = this.phi'*...
                  this.MGG*...
                  this.phi;
        end
        function BAA = get.BAA(this)
            BAA = this.phi'*...
                  this.BGG*...
                  this.phi;
        end
        function phi = get.phi(this)
            [phi,~] = eigs(this.KGG,...
                           this.MGG,...
                           this.ndof);
        end
        % Constructor
        function this = residual(inputs)
        %RESIDUAL Class constructor
        
            % Input checks
            if ~isa(inputs,'chassis_dynamics.model.simple_car.assembly')
                error('Input must be of type chassis_dynamics.model.simple_car.assembly');
            end
        
            % Set object properties
            this = this@chassis_dynamics.sequence.template.model(inputs);
        
        end
        % Modal information
        function fn = get_wn(this)
        %GET_WN Get circular natural frequencies of the system
            
            %Calculate from modal stiffenss matrix
            fn = real(sqrt(diag(this.KAA)));
            
        end
        function fn = get_fn(this)
        %GET_FN Get natural frequencies of the system
            
            %Calculate from modal stiffenss matrix
            fn = this.get_wn/(2*pi);
            
        end
        function zeta = get_zeta(this)
        %GET_ZETA Get critical damping ratios for modal damping, assumes
        %diagonal damping, which is not completely accurate for this system
            
            %Calculate from modal damping and stiffenss matrices
            zeta = real(diag(this.BAA) ./ (2.*sqrt(diag(this.KAA))));
            
        end
        % Plotting
        function animate_mode(this,mode,scale,delay)
        %ANIMATE_MODE Animate a mode shape
           
            % Default inputs
            if nargin < 3
                scale = 1;
            end
            if nargin < 4
                delay = 0;
            end
            
            % Create figure and axes
            f1 = figure;
            set(f1,'Units','Normalized','OuterPosition',[0.1,0.1,0.8,0.8]);
            ax1 = axes(f1,'Projection','perspective');
            grid on;
            fn = this.get_fn;
            zeta = this.get_zeta;
            title(sprintf('Mode Number: %d\nFreq: %0.2f\nDamping %0.1f%%\nScale Factor: %0.1f\n',mode,fn(mode),zeta(mode)*100,scale));
            
            % Set aspect ratio and view angle
            daspect(ax1,[1 1 1]);
            view(ax1,45,20);
            
            % Axis limits
            cg_loc = this.inputs.body.cg_loc;
            xlim(ax1,[-max(this.inputs.axle_r.corner_l.wheel.tire.radius,this.inputs.axle_r.corner_r.wheel.tire.radius)*8, ...
                       max(this.inputs.axle_f.corner_l.wheel.tire.radius,this.inputs.axle_f.corner_r.wheel.tire.radius)*8+this.inputs.wheelbase]);
            ylim(ax1,[-max(this.inputs.axle_f.track_width,this.inputs.axle_r.track_width)/2 * 4, ...
                       max(this.inputs.axle_f.track_width,this.inputs.axle_r.track_width)/2 * 4]);
            zlim(ax1,[-cg_loc(3)*2,cg_loc(3)*6])
            
            % Get the mode shape
            
            mode_shape = this.phi(:,mode);
            mode_scales = sin(linspace(-pi/2,pi/2,21)).*scale;
                  
            % Run the animation loop
            try
                while true
                    for i = 1:numel(mode_scales)
                        cla(ax1);
                        hold(ax1,'on');
                        this.plot_state(mode_shape.*mode_scales(i),ax1);
                        hold(ax1,'off');
                        drawnow;
                        pause(delay);
                    end
                    for i = 1:numel(mode_scales)-1
                        cla(ax1);
                        hold(ax1,'on');
                        this.plot_state(mode_shape.*mode_scales(end-i),ax1);
                        hold(ax1,'off');
                        drawnow;
                        pause(delay);
                    end
                end
            catch ex
                if ~isvalid(f1)
                    fprintf(1,'User cancelled animation\n');
                else
                    rethrow(ex);
                end
            end
            
        end
        function plot_state(this,state,ax,map,offset)
        %PLOT_STATE Plot the current state of the vehicle
            
            % Defaults
            if nargin < 3
                f = figure;
                ax = axes(f);
            end
            if nargin < 4
                map = this.map;
            end
            if nargin < 5
                offset = [0;0;0];
            end
            
            % Plot wheels
            persistent iMap11
            if isempty(iMap11)
                map11 = [11 1
                         11 2
                         11 3];
                iMap11 = chassis_dynamics.tools.map_index(map11,map);
            end
            Xw_fl_curr    = state(iMap11(1));
            Yw_fl_curr    = state(iMap11(2));
            Zw_fl_curr    = state(iMap11(3));
            wheel_ctr_fl = [ this.inputs.wheelbase + Xw_fl_curr
                             this.inputs.axle_f.track_width/2 + Yw_fl_curr
                             this.inputs.axle_f.corner_l.wheel.tire.radius + Zw_fl_curr];
            wheel_ctr_fl = wheel_ctr_fl + offset;
            this.plot_wheel(ax,wheel_ctr_fl,this.inputs.axle_f.corner_l.wheel.tire.radius,'k');
            
            persistent iMap12
            if isempty(iMap12)
                map12 = [12 1
                         12 2
                         12 3];
                iMap12 = chassis_dynamics.tools.map_index(map12,map);
            end
            Xw_fr_curr    = state(iMap12(1));
            Yw_fr_curr    = state(iMap12(2));
            Zw_fr_curr    = state(iMap12(3));
            wheel_ctr_fr = [ this.inputs.wheelbase + Xw_fr_curr
                            -this.inputs.axle_f.track_width/2 + Yw_fr_curr
                             this.inputs.axle_f.corner_r.wheel.tire.radius + Zw_fr_curr];
            wheel_ctr_fr = wheel_ctr_fr + offset;
            this.plot_wheel(ax,wheel_ctr_fr,this.inputs.axle_f.corner_r.wheel.tire.radius,'k');
            
            persistent iMap13
            if isempty(iMap13)
                map13 = [13 1
                         13 2
                         13 3];
                iMap13 = chassis_dynamics.tools.map_index(map13,map);
            end
            Xw_rl_curr    = state(iMap13(1));
            Yw_rl_curr    = state(iMap13(2));
            Zw_rl_curr    = state(iMap13(3));
            wheel_ctr_rl = [ Xw_rl_curr
                             this.inputs.axle_f.track_width/2 + Yw_rl_curr
                             this.inputs.axle_r.corner_l.wheel.tire.radius + Zw_rl_curr];
            wheel_ctr_rl = wheel_ctr_rl + offset;
            this.plot_wheel(ax,wheel_ctr_rl,this.inputs.axle_r.corner_l.wheel.tire.radius,'k');
            
            persistent iMap14
            if isempty(iMap14)
                map14 = [14 1
                         14 2
                         14 3];
                iMap14 = chassis_dynamics.tools.map_index(map14,map);
            end
            Xw_rr_curr    = state(iMap14(1));
            Yw_rr_curr    = state(iMap14(2));
            Zw_rr_curr    = state(iMap14(3));
            wheel_ctr_rr = [Xw_rr_curr
                            -this.inputs.axle_f.track_width/2 + Yw_rr_curr
                             this.inputs.axle_r.corner_r.wheel.tire.radius + Zw_rr_curr];
            wheel_ctr_rr = wheel_ctr_rr + offset;
            this.plot_wheel(ax,wheel_ctr_rr,this.inputs.axle_r.corner_r.wheel.tire.radius,'k');
            
            % Baseline CG location
            cg_loc_base = this.inputs.body.cg_loc;
            
            % Get body envelope
            persistent body_corners_base
            if isempty(body_corners_base)
                body_corners_base = [ this.inputs.wheelbase+this.inputs.axle_f.corner_l.wheel.tire.radius,  -this.inputs.axle_f.track_width/2*0.9, cg_loc_base(3) / 0.38
                                      this.inputs.wheelbase+this.inputs.axle_f.corner_l.wheel.tire.radius,   this.inputs.axle_f.track_width/2*0.9, cg_loc_base(3) / 0.38
                                      this.inputs.wheelbase+this.inputs.axle_f.corner_l.wheel.tire.radius,   this.inputs.axle_f.track_width/2*0.9, cg_loc_base(3) / 2.00
                                      this.inputs.wheelbase+this.inputs.axle_f.corner_l.wheel.tire.radius,  -this.inputs.axle_f.track_width/2*0.9, cg_loc_base(3) / 2.00
                                     -this.inputs.axle_r.corner_l.wheel.tire.radius,                        -this.inputs.axle_f.track_width/2*0.9, cg_loc_base(3) / 0.38
                                     -this.inputs.axle_r.corner_l.wheel.tire.radius,                         this.inputs.axle_f.track_width/2*0.9, cg_loc_base(3) / 0.38
                                     -this.inputs.axle_r.corner_l.wheel.tire.radius,                         this.inputs.axle_f.track_width/2*0.9, cg_loc_base(3) / 2.00
                                     -this.inputs.axle_r.corner_l.wheel.tire.radius,                        -this.inputs.axle_f.track_width/2*0.9, cg_loc_base(3) / 2.00];
            end
            body_corners = this.get_disp_at_loc(body_corners_base,state,map);
            body_corners = body_corners + offset';
            
            % Scale factor for object edges
            edge_scale_factor = this.inputs.wheelbase / 8;
            
            % Plot body
            this.plot_body(ax,body_corners);
            cg_loc_curr = this.get_disp_at_loc(cg_loc_base,state,map);
            cg_loc_curr = cg_loc_curr + offset';
            
            % Plot CG
            persistent x_sphere
            persistent y_sphere
            persistent z_sphere
            if isempty(x_sphere)
                [x_sphere,y_sphere,z_sphere] = sphere(10);
                x_sphere = x_sphere.*(edge_scale_factor/2);
                y_sphere = y_sphere.*(edge_scale_factor/2);
                z_sphere = z_sphere.*(edge_scale_factor/2);
            end
            surf(ax,x_sphere + cg_loc_curr(1),...
                    y_sphere + cg_loc_curr(2),...
                    z_sphere + cg_loc_curr(3),...
                    'edgecolor','none','facecolor',[0.5,0,0]);
            
            % Plot coordinate system
            plot3(ax,[0; edge_scale_factor],...
                     [0; 0],...
                     [0; 0]                ,'r','linewidth',2);
            plot3(ax,[0; 0],...
                     [0; edge_scale_factor],...
                     [0; 0]                ,'g','linewidth',2);
            plot3(ax,[0; 0],...
                     [0; 0],...
                     [0; edge_scale_factor],'b','linewidth',2);
            
        end
        % Coordinate transformation
        function loc = get_disp_at_loc(this,orig_loc,state,map)
        %GET_DISP_AT_LOC Get displacement results at a given location
            
            % Defaults
            if nargin < 4
                map = this.map;
            end
        
            % Get indices from map
            persistent iMap
            if isempty(iMap)
                map_ = [1 1
                        1 2
                        1 3
                        1 4
                        1 5
                        1 6];
                iMap = chassis_dynamics.tools.map_index(map_,map);
            end
            
            % State info
            X_cg     = state(iMap(1));
            Y_cg     = state(iMap(2));
            Z_cg     = state(iMap(3));
            phi_cg   = state(iMap(4));
            theta_cg = state(iMap(5));
            beta_cg  = state(iMap(6));
            
            % Baseline CG location
            cg_loc_base = this.inputs.body.cg_loc;
            
            % Vector to location from CG
            loc_vec = orig_loc - cg_loc_base;
            
            % Rotations about CG
            loc_vec_rot1 = chassis_dynamics.tools.rodrigues_rot(loc_vec,     [1,0,0],phi_cg);
            loc_vec_rot2 = chassis_dynamics.tools.rodrigues_rot(loc_vec_rot1,[0,1,0],theta_cg);
            loc_vec_rot3 = chassis_dynamics.tools.rodrigues_rot(loc_vec_rot2,[0,0,1],beta_cg);
            
            % Translation about CG
            loc = loc_vec_rot3 + cg_loc_base + [X_cg,Y_cg,Z_cg];
            
        end
    end
    methods (Access = private)
        function [a,b,cf,df,cr,dr] = get_abcd(this)
            
            cg_loc = this.inputs.body.cg_loc;
            a  = this.inputs.wheelbase - cg_loc(1);
            b  = this.inputs.body.cg_loc(1);
            cf = this.inputs.axle_f.track_width - cg_loc(2);
            df = this.inputs.axle_f.track_width + cg_loc(2);
            cr = this.inputs.axle_r.track_width - cg_loc(2);
            dr = this.inputs.axle_r.track_width + cg_loc(2);
            
        end
    end
    methods (Static,Access = private)
        function plot_wheel(ax,wheel_ctr,wheel_r,colorspec)
        %PLOT_WHEEL Plot the shape of a wheel at its current position
            
            % Create vector of tangential coordinates
            envelope_space = linspace(0,2*pi,20);
            
            % Envelope of wheel
            wheel_envelope = [sin(envelope_space).*wheel_r+wheel_ctr(1)
                              repmat(wheel_ctr(2),1,numel(envelope_space))
                              cos(envelope_space).*wheel_r+wheel_ctr(3)];
                          
            % Plot shape
            fill3(ax,wheel_envelope(1,:),wheel_envelope(2,:),wheel_envelope(3,:),colorspec);
            
        end
        function plot_body(ax,body_corners)
        %PLOT_BODY Plot the body as a prism
            
            % Plot a rectangular prism for body
            fill3(ax,body_corners([1,2,3,4],1),body_corners([1,2,3,4],2),body_corners([1,2,3,4],3),'r','facealpha',.2);
            fill3(ax,body_corners([5,6,7,8],1),body_corners([5,6,7,8],2),body_corners([5,6,7,8],3),'r','facealpha',.2);
            fill3(ax,body_corners([1,4,8,5],1),body_corners([1,4,8,5],2),body_corners([1,4,8,5],3),'r','facealpha',.2);
            fill3(ax,body_corners([2,3,7,6],1),body_corners([2,3,7,6],2),body_corners([2,3,7,6],3),'r','facealpha',.2);
            fill3(ax,body_corners([3,4,8,7],1),body_corners([3,4,8,7],2),body_corners([3,4,8,7],3),'r','facealpha',.2);
            fill3(ax,body_corners([1,2,6,5],1),body_corners([1,2,6,5],2),body_corners([1,2,6,5],3),'r','facealpha',.2);
            
        end
    end
end

