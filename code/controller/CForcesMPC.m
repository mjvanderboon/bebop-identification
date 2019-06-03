classdef CForcesMPC < handle
    % CForcesMPC Generate and store solver as class object
    % A new CForcesMPC object is required per Drone. The generation of
    % the solver should only occur once if all Drones use same MPC 
    
    
    %% Properties
    properties
        
        % Shared variables local copy
        model                   % MPC model variables
        index                   % Problem index
        cfg                     % Configuration
        
        % MPC variables
        Z_k                     % Current stage variables
        
        % MPC data
        problem                 % Input to MPC solver - ForcesPro
        Z_plan                  % MPC planned stage variables over N stages
        
    end
    
    
    %% Methods
    methods
        
        %% Constructor
        function obj = CForcesMPC(share)
            % Forces Controller constructor
                       
            % Copy shared model and index variable for local object use
            obj.model = share.model;
            obj.index = share.index;
        end
        
        %% Initialzation
        function initialise(obj,x_start)
            % Initialise the Z_plan and current stage variable with
            % startpoint and equillibrium conditions
            
            u_start = zeros(obj.model.nin + obj.model.nSlack,1);
            
            obj.Z_k =  [u_start; x_start];
            
            % Store initial MPC plan (used for first MPC optimisation guess of stages problem.x0)
            % Only useful if the Quad is brought to designed startPos and is in equillibirium
            obj.Z_plan = kron(ones(1,obj.model.N),obj.Z_k);
        end
        
        %% Generate solver, ForcesPro options setup
        function generateFORCES(obj)
            % Generate new FORCES Pro controller and obtain files from web
            fprintf('[%s] Generating new FORCES solver...\n',datestr(now,'HH:MM:SS'));
            
            codeoptions = getOptions('FORCESNLPsolver');
            
            codeoptions.maxit       = 100;      % Maximum number of iterations
            codeoptions.printlevel  = 0;        % Use printlevel = 2 to print progress (but not for timings)
            codeoptions.optlevel    = 3;        % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
            codeoptions.overwrite   = 1;
            codeoptions.cleanup     = 1;
            codeoptions.timing      = 1;
            
            codeoptions.BuildSimulinkBlock = 0;
            
            codeoptions.parallel    = 1;        % Run prediction on multiple cores (better for longer horizons)
            codeoptions.nlp.linear_solver = 'symm_indefinite_fast'; % linear system solver, better for long horizons
            codeoptions.noVariableElimination = 1;
            
            codeoptions.nlp.TolStat = 5E-4;     % infinity norm tolerance on stationarity
            codeoptions.nlp.TolEq   = 5E-4;     % infinity norm of residual for equalities
            codeoptions.nlp.TolIneq = 5E-4;     % infinity norm of residual for inequalities
            codeoptions.nlp.TolComp = 5E-4;     % tolerance on complementarity conditions
            
            % Generate forces solver and create necessary files in
            % directory
            FORCES_NLP(obj.model, codeoptions);
            
            fprintf('[%s] FORCES solver generated OK \n',datestr(now,'HH:MM:SS'));
        end
        
        %% Set real-time parameter
        function setRTParameter(obj,Z_plan_i,setpointsDrone,predictionObstacle,yaw,dt,quadID)
            % Set the real-time parameter vector used by MPC to allow
            % changes in the problem definition. The problem.all_parameters
            % requires a real-time vector of size model.npar * model.N
            
            % Repeating part of N stage vector
            % Repeating part with start/end points measured dt and zeros 
            p_rep = zeros(obj.model.npar,1);
            
            % set setpointsDrone to start and endpoint of waypoints
            % setpointsDrone = 6 x 1 (3 x 1 start point 3 x 1 end point)
            p_rep([obj.index.startpointProblem obj.index.endpointProblem]) = setpointsDrone;
            
            % Set real time yaw
            p_rep(obj.index.realyaw) = yaw;
            
            % Set measured dt (Real dt)
            p_rep(obj.index.realdt) = dt;
                                  
            % N Stage vector
            % Initialise real-time N stages vector
            pN = repmat(p_rep,[obj.model.N,1]);
                                                
            % set predictionObstacle as prediction of moving obstacles
            % predictionObstacle = (3 x model.N) x model.nObs
            for iObs = 1:obj.model.nObs
                pN(obj.index.obsParamsN + 3*(iObs - 1)) = reshape(predictionObstacle(:,:,iObs),obj.model.nParamsPerObs*obj.model.N,1);
            end
            
            % set planned MPC of all other quadrotors in p vector
            % no need to set for the current Quad quadID
            % Z_plan_i = model.nvar * model.N * (model.nQuad)
            idx = 1;
            
            for iQuad = 1:obj.model.nQuad
                % Skip self
                if (iQuad == quadID) 
                    continue; 
                end 
                % Set planned positions in p vector
                pN(obj.index.quadParamsN + obj.model.nParamsPerQuad*(idx - 1)) = Z_plan_i(obj.index.posStates, :, iQuad);
                
                idx = idx + 1;
            end
            
            % set problem real-time vector for all stages
            obj.problem.all_parameters = pN;  
        end
        
        %% Set real-time parameter
        function setRTParameterCov(obj,Z_plan_i,predictionCovariance,setpointsDrone,predictionObstacle,yaw,dt,auxProbaObs,auxProbaQuad,...
                weightCollObs, weightCollQuad, weightInput, weightWaypoint, quadID)
            % Set the real-time parameter vector used by MPC to allow
            % changes in the problem definition. The problem.all_parameters
            % requires a real-time vector of size model.npar * model.N
            
            % Repeating part of N stage vector
            % Repeating part with start/end points measured dt and zeros 
            p_rep = zeros(obj.model.npar,1);
            
            % set setpointsDrone to start and endpoint of waypoints
            % setpointsDrone = 6 x 1 (3 x 1 start point 3 x 1 end point)
            p_rep([obj.index.startpointProblem obj.index.endpointProblem]) = setpointsDrone;
            
            % Set real time yaw
            p_rep(obj.index.realyaw) = yaw;
            
            % Set measured dt (Real dt)
            p_rep(obj.index.realdt) = dt;
            
            % Set chance constraint threshold
            p_rep(obj.index.thresholdProbaObs)  = auxProbaObs;
            p_rep(obj.index.thresholdProbaQuad) = auxProbaQuad;
            
            % Set cost terms weights
            p_rep(obj.index.weightCollObs)  = weightCollObs;
            p_rep(obj.index.weightCollQuad) = weightCollQuad;
            p_rep(obj.index.weightInput)    = weightInput;
            p_rep(obj.index.weightWaypoint) = weightWaypoint;

            % Set quadID
            p_rep(obj.index.quadID) = quadID;
                                  
            % N Stage vector
            % Initialise real-time N stages vector
            pN = repmat(p_rep,[obj.model.N,1]);
                                                
            % set predictionObstacle as prediction of moving obstacles
            % predictionObstacle = (20 x model.N) x model.nObs
            for iObs = 1:obj.model.nObs
                pN(obj.index.obsParamsMaintainProbaN + obj.model.nParamsPerObs*(iObs - 1)) = predictionObstacle(obj.index.obsMaintainProba, :, iObs);
                pN(obj.index.obsParamsMaintainPosN + obj.model.nParamsPerObs*(iObs - 1))   = predictionObstacle(obj.index.obsMaintainPos, :, iObs);
                pN(obj.index.obsParamsMaintainPosCovN + obj.model.nParamsPerObs*(iObs - 1)) = predictionObstacle(obj.index.obsMaintainPosCov, :, iObs);
                
                pN(obj.index.obsParamsStopProbaN + obj.model.nParamsPerObs*(iObs - 1)) = predictionObstacle(obj.index.obsStopProba, :, iObs);
                pN(obj.index.obsParamsStopPosN + obj.model.nParamsPerObs*(iObs - 1))   = predictionObstacle(obj.index.obsStopPos, :, iObs);
                pN(obj.index.obsParamsStopPosCovN + obj.model.nParamsPerObs*(iObs - 1)) = predictionObstacle(obj.index.obsStopPosCov, :, iObs);
            end
            
            % set planned MPC of all other quadrotors in p vector
            % no need to set for the current Quad quadID
            % Z_plan_i = model.nvar * model.N * (model.nQuad)
            idx = 1;
            for iQuad = 1:obj.model.nQuad
                % Here skipped
                if (iQuad == quadID) 
                    continue; 
                end 
                % Set planned positions in p vector
                pN(obj.index.quadParamsN + obj.model.nParamsPerQuad*(idx - 1)) = Z_plan_i(obj.index.posStates, :, iQuad);
                % Set planned covariance in p vector 
                pN(obj.index.quadParamsCovN + obj.model.nParamsPerQuad*(idx - 1)) =  predictionCovariance(:, :, iQuad);
                
                idx = idx + 1;
            end
            
            % set self predicted position uncertainty covariance
            pN(obj.index.quadPosCovParamsN) = predictionCovariance(:, :, quadID);
 
            % set problem real-time vector for all stages
            obj.problem.all_parameters = pN;   
        end
        
        
         %% Set real-time parameter
        function setRTParameterFull(obj,Z_plan_i,setpointsDrone,predictionObstacle,yaw,dt,auxProbaObs,auxProbaQuad,...
                weightCollObs, weightCollQuad, weightInput, weightWaypoint, quadID)
            % Set the real-time parameter vector used by MPC to allow
            % changes in the problem definition. The problem.all_parameters
            % requires a real-time vector of size model.npar * model.N
            
            % Repeating part of N stage vector
            % Repeating part with start/end points measured dt and zeros 
            p_rep = zeros(obj.model.npar,1);
            
            % set setpointsDrone to start and endpoint of waypoints
            % setpointsDrone = 6 x 1 (3 x 1 start point 3 x 1 end point)
            p_rep([obj.index.startpointProblem obj.index.endpointProblem]) = setpointsDrone;
            
            % Set real time yaw
            p_rep(obj.index.realyaw) = yaw;
            
            % Set measured dt (Real dt)
            p_rep(obj.index.realdt) = dt;
            
            % Set chance constraint threshold
            p_rep(obj.index.thresholdProbaObs)  = auxProbaObs;
            p_rep(obj.index.thresholdProbaQuad) = auxProbaQuad;
            
            % Set cost terms weights
            p_rep(obj.index.weightCollObs)  = weightCollObs;
            p_rep(obj.index.weightCollQuad) = weightCollQuad;
            p_rep(obj.index.weightInput)    = weightInput;
            p_rep(obj.index.weightWaypoint) = weightWaypoint;
 
            % Set quadID
            p_rep(obj.index.quadID) = quadID;
                                  
            % N Stage vector
            % Initialise real-time N stages vector
            pN = repmat(p_rep,[obj.model.N,1]);
                                                
            % set predictionObstacle as prediction of moving obstacles
            % predictionObstacle = (20 x model.N) x model.nObs
            for iObs = 1:obj.model.nObs
                pN(obj.index.obsParamsMaintainProbaN + obj.model.nParamsPerObs*(iObs - 1)) = predictionObstacle(obj.index.obsMaintainProba, :, iObs);
                pN(obj.index.obsParamsMaintainPosN + obj.model.nParamsPerObs*(iObs - 1))   = predictionObstacle(obj.index.obsMaintainPos, :, iObs);
                pN(obj.index.obsParamsMaintainPosCovN + obj.model.nParamsPerObs*(iObs - 1)) = predictionObstacle(obj.index.obsMaintainPosCov, :, iObs);
                
                pN(obj.index.obsParamsStopProbaN + obj.model.nParamsPerObs*(iObs - 1)) = predictionObstacle(obj.index.obsStopProba, :, iObs);
                pN(obj.index.obsParamsStopPosN + obj.model.nParamsPerObs*(iObs - 1))   = predictionObstacle(obj.index.obsStopPos, :, iObs);
                pN(obj.index.obsParamsStopPosCovN + obj.model.nParamsPerObs*(iObs - 1)) = predictionObstacle(obj.index.obsStopPosCov, :, iObs);
            end
            
            
            % set planned MPC of all other quadrotors in p vector
            % no need to set for the current Quad quadID
            % Z_plan_i = model.nvar * model.N * (model.nQuad)
            idx = 1;
            for iQuad = 1:obj.model.nQuad
                % Here skipped
                if (iQuad == quadID) 
                    continue; 
                end 
                % Set planned positions in p vector
                pN(obj.index.quadParamsN + obj.model.nParamsPerQuad*(idx - 1)) = Z_plan_i(obj.index.posStates, :, iQuad);
                % Set planned covariance in p vector 
                pN(obj.index.quadParamsCovN + obj.model.nParamsPerQuad*(idx - 1)) =  Z_plan_i(obj.index.posCov, :, iQuad);
                
                idx = idx + 1;
            end
            
            % set problem real-time vector for all stages
            obj.problem.all_parameters = pN;  
        end
        
        
        %% Get MPC plan
        function MPCplan = getZplan(obj)
            % Retrieve the MPC plan
            MPCplan = obj.Z_plan;
        end
        
        %% Get the first and second optimized stages
        function [Z_k,Z_k2,exitflag_out,info] = step(obj,X)
            % Call FORCES solver provided the initial conditions X and do
            % one step of solver
            
            % Set initial conditions on variables indexed by model.xinitidx
            % This includes external recorded position and velocity data
            obj.problem.xinit = X;
            
            % Set initial guess for solver for all stages with shifted trajectory
%             x0tmp = reshape([obj.Z_plan(:,2:obj.model.N), obj.Z_plan(:,obj.model.N)], obj.model.N*obj.model.nvar, 1);
% %             obj.problem.x0 = x0tmp + 0.05*rand(size(x0tmp)); % Add some random variations
%             obj.problem.x0 = x0tmp; % Add some random variations

            Z_plan_ini = obj.Z_plan;        % last planned MPC as initial guess
            % random nosie can be added to the plan (to position and velocity)
            % this is very important to avoid deadlock
            % in case that positions are zero
            for i = 1:obj.model.N
%                 Z_plan_ini(obj.index.posStates,i) = Z_plan_ini(obj.index.posStates,i)...
%                     + 1.2*Z_plan_ini(obj.index.posStates,i).*randi([-1,1])...
%                     .*rand(size(Z_plan_ini(obj.index.posStates,i)));
%                 Z_plan_ini(obj.index.posStates,i) = Z_plan_ini(obj.index.posStates,i)...
%                     + 0.8*Z_plan_ini(obj.index.posStates,i)...
%                     .*rand(size(Z_plan_ini(obj.index.posStates,i)));
%                 if abs(Z_plan_ini(obj.index.posStates(1),i)) < 1e-2
%                     Z_plan_ini(obj.index.posStates(1),i) = 1.2*rand;
%                 end
%                 if abs(Z_plan_ini(obj.index.posStates(2),i)) < 1e-2
%                     Z_plan_ini(obj.index.posStates(2),i) = 1.2*rand;
%                 end
%                 Z_plan_ini(obj.index.fullCov,i) = Z_plan_ini(obj.index.fullCov,i)...
%                     - 0.8*Z_plan_ini(obj.index.fullCov,i)...
%                     .*rand(size(Z_plan_ini(obj.index.fullCov,i)));
            end
            x0tmp = reshape([Z_plan_ini(:,2:obj.model.N), Z_plan_ini(:,obj.model.N)], obj.model.N*obj.model.nvar, 1);
            obj.problem.x0 = x0tmp;
            
            % Call the NLP FORCES solver
            [solveroutput, exitflag, info] = FORCESNLPsolver(obj.problem);
            exitflag_out = exitflag;
            
            if exitflag == 0
                % FORCES reached maximum iterations for problem
                exitflag_out = 1;
                disp('MPC: max it');
            end
            if exitflag == 1
                % FORCES optimial solution found
                exitflag_out = 1;
            end
            
            % Copy the MPC plan into Z_plan
            for iStage=1:obj.model.N
                obj.Z_plan(:,iStage) = solveroutput.(['x',sprintf('%02d',iStage)]);
            end
            
            % Return the computed current and next stage (with inputs)
            Z_k = obj.Z_plan(:,1);
            Z_k2 = obj.Z_plan(:,2);
        end
        
        
    %% end of methods    
    end
    
%% end of class
end