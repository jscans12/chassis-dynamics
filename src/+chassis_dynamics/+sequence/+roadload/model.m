classdef model < chassis_dynamics.sequence.template.model
%MODEL A simple full-vehicle chassis model and coupled structures
    properties (Access = private)
        %RESET_MPC The MPC function only calculates once, so I need a way
        %to reset it
        reset_mpc(1,1) logical = true
    end
    properties (Dependent)
        %MAP Map of each DOF
        map
        %DESC Description for each DOF
        desc
        %KGG Residual and subsystem stiffness matrices coupled
        KGG
        %MGG Residual and subsystem mass matrices coupled
        MGG
        %BGG Residual and subsystem damping matrices coupled
        BGG
        %KLL Stiffness matrix after MPC reduction
        KLL
        %MLL Mass matrix after MPC reduction
        MLL
        %BLL Damping matrix after MPC reduction
        BLL
        %WHEEL_CONTACT Wheel contact bits, true for in-contact and false
        %for out of contact. Order is FL,FR,RL,RR.
        wheel_contact
    end
    methods
        % Getters and setters
        function map = get.map(this)
            map = this.inputs.map;
        end
        function desc = get.desc(this)
            desc = this.inputs.desc;
        end
        function KGG = get.KGG(this)
            
            % Preallocate
            base_size = size(this.inputs.KGG,1);
            nnz_all = nnz(this.inputs.KGG);
            KGG = spalloc(base_size,base_size,nnz_all);
            
            % Insert residual structure
            curr_index = 1;
            curr_size = size(this.inputs.KGG,1);
            end_index = curr_index + curr_size - 1;
            KGG(curr_index:end_index,curr_index:end_index) = this.inputs.KGG;
            
            KGG = full(KGG);
            
        end
        function BGG = get.BGG(this)
            
            % Preallocate
            base_size = size(this.inputs.BGG,1);
            nnz_all = nnz(this.inputs.BGG);
            BGG = spalloc(base_size,base_size,nnz_all);
            
            % Insert residual structure
            curr_index = 1;
            curr_size = size(this.inputs.BGG,1);
            end_index = curr_index + curr_size - 1;
            BGG(curr_index:end_index,curr_index:end_index) = this.inputs.BGG;
            
            BGG = full(BGG);
            
        end
        function MGG = get.MGG(this)
            
            % Preallocate
            base_size = size(this.inputs.MGG,1);
            nnz_all = nnz(this.inputs.MGG);
            MGG = spalloc(base_size,base_size,nnz_all);
            
            % Insert residual structure
            curr_index = 1;
            curr_size = size(this.inputs.MGG,1);
            end_index = curr_index + curr_size - 1;
            MGG(curr_index:end_index,curr_index:end_index) = this.inputs.MGG;
  
            MGG = full(MGG);
            
        end
        function KLL = get.KLL(this)
            
            % MPC reduction (from NASTRAN theoretical manual)
            [MPC,~,nset] = this.get_MPC;
            KLL = MPC'*this.KGG*MPC;
            KLL = KLL(nset,nset);

            
        end
        function BLL = get.BLL(this)
            
            % MPC reduction (from NASTRAN theoretical manual)
            [MPC,~,nset] = this.get_MPC;
            BLL = MPC'*this.BGG*MPC;
            BLL = BLL(nset,nset);
            
        end
        function MLL = get.MLL(this)
            
            % MPC reduction (from NASTRAN theoretical manual)
            [MPC,~,nset] = this.get_MPC;
            MLL = MPC'*this.MGG*MPC;
            MLL = MLL(nset,nset);
            
        end
        function wheel_contact = get.wheel_contact(this)
            wheel_contact = this.inputs.wheel_contact;
        end
        function set.wheel_contact(this,wheel_contact)
            this.inputs.wheel_contact = wheel_contact;
        end
        % Constructor
        function this = model(inputs)
        %MODEL Class constructor
        
            % Add residual structure
            this = this@chassis_dynamics.sequence.template.model(inputs);
            
        end
        % Solution matrices
        function A = get_A(this)
        %GET_A Get the A "dynamic" matrix used in the state-space EOM
        %formulation
            
            % Get partitions
            [~,~,nset] = this.get_MPC;
            ndof_ = numel(nset);
            A11 = zeros(ndof_);
            A12 = eye(ndof_);
            A21 = -this.MLL \ this.KLL;
            A22 = -this.MLL \ this.BLL;
            
            % Assemble matrix
            A = [A11 A12
                 A21 A22];
            
        end
        function L = get_L(this)
        %GET_LD Get the L locator matrix used in the state-space EOM
        %formulation
            
            % Get constituent load matrices
            Ld = this.get_Ld;
            Lv = this.get_Lv;
            Lg = this.get_Lg;
            Lf = this.get_Lf;
            
            % Concatenate
            L = [Ld;Lv;Lg;Lf];
            
        end
        % Transformation matrices
        function [MPC,mset,nset] = get_MPC(this)
        %GET_MPC Get the MPC equation for this model
            
            % Memoize
            persistent MPC_pers
            persistent mset_pers
            persistent nset_pers
            if ~isempty(MPC_pers) && ~this.reset_mpc
                MPC = MPC_pers;
                mset = mset_pers;
                nset = nset_pers;
                return
            end
        
            % Nodal connectivity
            %           ind dep dof
            conn_def = [1   11  1
                        1   11  2
                        1   12  1
                        1   12  2
                        1   13  1
                        1   13  2
                        1   14  1
                        1   14  2];
            
            % MSET indices
            mset = chassis_dynamics.tools.map_index(conn_def(:,2:3),this.map);
            nset = setdiff(1:this.ndof,mset);
            
            % Default
            MPC = speye(this.ndof);
            
            % GRIDs
            GRID = this.inputs.GRID;
            
            % Loop through the connection def
            for id_indx = 1:size(conn_def,1)
                
                % Get IDs
                id_indep = conn_def(id_indx,1);
                id_depen = conn_def(id_indx,2);
                
                % Curr DOF
                curr_dof = conn_def(id_indx,3);
                
                % Map indices
                map_ind_indep = find(chassis_dynamics.tools.map_index(this.map,[id_indep,curr_dof]));
                map_ind_depen = find(chassis_dynamics.tools.map_index(this.map,[id_depen,curr_dof]));
                
                % Get GRIDs
                grid_indep = GRID(GRID.ID == id_indep,:);
                grid_depen = GRID(GRID.ID == id_depen,:);

                % Matching DOF is always 1
                MPC(map_ind_depen,map_ind_depen) = 0;
                if ~isempty(map_ind_indep)
                    MPC(map_ind_depen,map_ind_indep) = 1;
                else
                    continue
                end
                
                % Translational DOF get extra treatment
                switch curr_dof
                    case 1
                        map_ind_indep5 = find(chassis_dynamics.tools.map_index(this.map,[id_indep,5]));
                        if ~isempty(map_ind_indep5)
                            MPC(map_ind_depen,map_ind_indep5) =  (grid_depen.X3 - grid_indep.X3);
                        end
                        map_ind_indep6 = find(chassis_dynamics.tools.map_index(this.map,[id_indep,6]));
                        if ~isempty(map_ind_indep6)
                            MPC(map_ind_depen,map_ind_indep6) = -(grid_depen.X2 - grid_indep.X2);
                        end
                    case 2
                        map_ind_indep4 = find(chassis_dynamics.tools.map_index(this.map,[id_indep,4]));
                        if ~isempty(map_ind_indep4)
                            MPC(map_ind_depen,map_ind_indep4) = -(grid_depen.X3 - grid_indep.X3);
                        end
                        map_ind_indep6 = find(chassis_dynamics.tools.map_index(this.map,[id_indep,6]));
                        if ~isempty(map_ind_indep6)
                            MPC(map_ind_depen,map_ind_indep6) =  (grid_depen.X1 - grid_indep.X1);
                        end
                    case 3
                        map_ind_indep4 = find(chassis_dynamics.tools.map_index(this.map,[id_indep,4]));
                        if ~isempty(map_ind_indep4)
                            MPC(map_ind_depen,map_ind_indep4) =  (grid_depen.X2 - grid_indep.X2);
                        end
                        map_ind_indep5 = find(chassis_dynamics.tools.map_index(this.map,[id_indep,5]));
                        if ~isempty(map_ind_indep5)
                            MPC(map_ind_depen,map_ind_indep5) = -(grid_depen.X1 - grid_indep.X1);
                        end
                end
            end
            
            % Memoize
            this.reset_mpc = false;
            MPC_pers = MPC;
            mset_pers = mset;
            nset_pers = nset;
            
        end
        % Plotting
        function plot_state(this,state,ax,offset)
        %PLOT_STATE Plot state of residual structure
            
            % Defaults
            if nargin < 3
                f = figure;
                ax = axes(f);
            end
            if nargin < 4
                offset = [0;0;0];
            end
            
            % Plot residual
            map_ = this.map;
            this.inputs.plot_state(state,ax,map_,offset);
            
        end
    end
    methods (Access = private)
        function Ld = get_Ld(this)
        %GET_LD Get the Ld displacement locator matrix used in the 
        %state-space EOM formulation
        
            % Get nset
            [~,~,nset] = this.get_MPC;
        
            % Get indices from map
            persistent iMap
            if isempty(iMap)
                map_ = [11 3
                        12 3
                        13 3
                        14 3];
                iMap = chassis_dynamics.tools.map_index(map_,this.map(nset,:));
            end
            
            % Get Ed, populate sparsely
            ndof_ = numel(nset);
            Ed = zeros(ndof_*2,numel(iMap));
            if this.inputs.wheel_contact(1)
                Ed(iMap(1)+ndof_,1) = this.inputs.inputs.axle_f.corner_l.wheel.tire.spring_rate;
            end
            if this.inputs.wheel_contact(2)
                Ed(iMap(2)+ndof_,2) = this.inputs.inputs.axle_f.corner_r.wheel.tire.spring_rate;
            end
            if this.inputs.wheel_contact(3)
                Ed(iMap(3)+ndof_,3) = this.inputs.inputs.axle_r.corner_l.wheel.tire.spring_rate;
            end
            if this.inputs.wheel_contact(4)
                Ed(iMap(4)+ndof_,4) = this.inputs.inputs.axle_r.corner_r.wheel.tire.spring_rate;
            end
            
            % Mass partitions
            M_ = [this.MLL    zeros(ndof_)
                  zeros(ndof_)  this.MLL];
            
            % Get Bf
            data = M_\Ed;
            
            % Load set map
            map_ = [111 3 0
                    112 3 0
                    113 3 0
                    114 3 0];
               
            % Load set description
            desc_ = {'Rfl_Tz'
                     'Rfr_Tz'
                     'Rrl_Tz'
                     'Rrr_Tz'};
                
            % Output load set
            Ld = chassis_dynamics.types.ltm('Ld',map_,desc_,data');
            
        end
        function Lv = get_Lv(this)
        %GET_LV Get the Lv velocity locator matrix used in the 
        %state-space EOM formulation
            
            % Get nset
            [~,~,nset] = this.get_MPC;
        
            % Get indices from map
            persistent iMap
            if isempty(iMap)
                map_ = [11 3
                        12 3
                        13 3
                        14 3];
                iMap = chassis_dynamics.tools.map_index(map_,this.map(nset,:));
            end
            
            % Get E, populate sparsely
            ndof_ = numel(nset);
            Ev = zeros(ndof_*2,numel(iMap));
            if this.inputs.wheel_contact(1)
                Ev(iMap(1)+ndof_,1) = this.inputs.inputs.axle_f.corner_l.wheel.tire.damping_coeff;
            end
            if this.inputs.wheel_contact(2)
                Ev(iMap(2)+ndof_,2) = this.inputs.inputs.axle_f.corner_r.wheel.tire.damping_coeff;
            end
            if this.inputs.wheel_contact(3)
                Ev(iMap(3)+ndof_,3) = this.inputs.inputs.axle_r.corner_l.wheel.tire.damping_coeff;
            end
            if this.inputs.wheel_contact(4)
                Ev(iMap(4)+ndof_,4) = this.inputs.inputs.axle_r.corner_r.wheel.tire.damping_coeff;
            end
            
            % Mass partitions
            M_ = [this.MLL      zeros(ndof_)
                  zeros(ndof_)  this.MLL];
            
            % Get Bf
            data = M_\Ev;
            
            % Load set map
            map_ = [211 3 0
                    212 3 0
                    213 3 0
                    214 3 0];
               
            % Load set description
            desc_ = {'Rfl_Vz'
                     'Rfr_Vz'
                     'Rrl_Vz'
                     'Rrr_Vz'};
                
            % Output load set
            Lv = chassis_dynamics.types.ltm('Lv',map_,desc_,data');
            
        end
        function Lg = get_Lg(this)
        %GET_LG Get the Lg acceleration locator matrix used in the
        %state-space EOM formulation
            
            % Get nset
            [~,~,nset] = this.get_MPC;
            
            % Get indices from map
            persistent iMap
            if isempty(iMap)
                map_ = [1  1
                        1  2
                        1  3
                        1  4
                        1  5
                        1  6
                        11 3
                        12 3
                        13 3
                        14 3];
                iMap = chassis_dynamics.tools.map_index(map_,this.map(nset,:));
            end
            
            % Get E, populate sparsely
            ndof_ = numel(nset);
            Eg = zeros(ndof_*2,numel(iMap));
            for i = 1:numel(iMap)
                Eg(iMap(i)+ndof_,iMap(i)) = 1;
            end
            
            % Direct copy
            data = Eg;
            
            % Load set map
            map_ = [301 1 0
                    301 2 0
                    301 3 0
                    301 4 0
                    301 5 0
                    301 6 0
                    311 3 0
                    312 3 0
                    313 3 0
                    314 3 0];
            
            % Load set description
            desc_ = {'CG_Ax'
                     'CG_Ay'
                     'CG_Az'
                     'CG_Alphax'
                     'CG_Alphay'
                     'CG_Alphaz'
                     'Wfl_Az'
                     'Wfr_Az'
                     'Wrl_Az'
                     'Wrr_Az'};
            
            % Output load set
            Lg = chassis_dynamics.types.ltm('Lg',map_,desc_,data');
            
        end
        function Lf = get_Lf(this)
        %GET_LF Get the Lf force locator matrix used in the
        %state-space EOM formulation
            
            % Get nset
            [~,~,nset] = this.get_MPC;
            
            % Get indices from map
            persistent iMap
            if isempty(iMap)
                map_ = [1  1
                        1  2
                        1  3
                        1  4
                        1  5
                        1  6
                        11 3
                        12 3
                        13 3
                        14 3];
                iMap = chassis_dynamics.tools.map_index(map_,this.map(nset,:));
            end
            
            % Get E, populate sparsely
            ndof_ = numel(nset);
            Ef = zeros(ndof_*2,numel(iMap));
            for i = 1:numel(iMap)
                Ef(iMap(i)+ndof_,iMap(i)) = 1;
            end
            
            % Mass partitions
            M_ = [this.MLL      zeros(ndof_)
                  zeros(ndof_)  this.MLL];
            
            % Get Bf
            data = M_\Ef;
            
            % Load set map
            map_ = [401 1 0
                    401 2 0
                    401 3 0
                    401 4 0
                    401 5 0
                    401 6 0
                    411 3 0
                    412 3 0
                    413 3 0
                    414 3 0];
            
            % Load set description
            desc_ = {'CG_Fx'
                     'CG_Fy'
                     'CG_Fz'
                     'CG_Mx'
                     'CG_My'
                     'CG_Mz'
                     'Wfl_Fz'
                     'Wfr_Fz'
                     'Wrl_Fz'
                     'Wrr_Fz'};
            
            % Output load set
            Lf = chassis_dynamics.types.ltm('Lf',map_,desc_,data');
            
        end
    end
end
%#ok<*SPRIX>

