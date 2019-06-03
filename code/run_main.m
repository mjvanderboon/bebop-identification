% main run for multiple uav applications uing MPC
% TODO: try to not need to be changed for different application

%% clean workspace
clear all; close all; clear mex; clearvars; clearvars -global
clear java
clear classes
close all force
clc

fprintf('[%s] Starting UAV run_main.m \n',datestr(now,'HH:MM:SS'));

%% initilization
initialize;

%% intialize CGraphic communicator
if cfg.GraphicCom == 1
    GraphicCom = CGraphic(true, cfg.Graphic.nQuadStates, cfg.Graphic.nQuadSetpoints, ...
        cfg.Graphic.nQuadMPCPlan, cfg.Graphic.nObsStates, share);
end

%% Generate Quadrotors with Controllers and Initialise
% Generate the model.nQuad number of quadrotors and intialize the MPC controllers for them
for i = 1:1 %For now 1 drone
    % Create quadrotor class object
    Quadrotor(i) = CDrone(share.cfg.quadType, i, 0, share);            
    
    % Set intial condition for quadrotor
    %x_start = zeros(model.neq,1);
    %x_start(index.x.posStates) = quadStartPos(:,i);     % Set starting position, filted, not real
    %x_start(index.x.velStates) = quadStartVel(:,i);    % starting velocity are set to zero defaultly
    Quadrotor(i).initialise();        
    
    % Set initial yaw
    Quadrotor(i).m_euler_raw(3) = pr.realyawQuad(i);
    Quadrotor(i).m_euler_real(3) = pr.realyawQuad(i);
end

%% Initialises ROS publishers and subscribers for Quadrotor
% TODO: make it not only used in experiment
for i = 1:1
    if ~cfg.ModeSim && (cfg.UseOpti || cfg.UseVicon) 
        Quadrotor(i).initialiseROS(mapping);
    end
end

%% Initialize Joy, state to run
if (cfg.UseOpti || cfg.UseVicon)
    Joy = CJoy();
end

%% Complete initialization
fprintf('[%s] Initialization Complete \n',datestr(now,'HH:MM:SS'));
if cfg.ModeSim
    fprintf('[%s] MODE: Running as Simulation \n',datestr(now,'HH:MM:SS'));
else
    fprintf('[%s] MODE: Running as Experiments \n',datestr(now,'HH:MM:SS'));
end

%% Data logging
if cfg.DoLog    
%     log_TotTime = nan(1, cfg.logsize);                  % Total computation time in one loop
%        
%     log_QuadZk = zeros(model.nvar, cfg.logsize, model.nQuad);   % Quadrotor z data
%     log_QuadEuler = zeros(3, cfg.logsize, model.nQuad); % Quadrotor Euler angles
%     
%     log_QuadPos = zeros(3, cfg.logsize, model.nQuad);   % raw quad position
%     
%     log_QuadStatesReal = zeros(9, cfg.logsize, model.nQuad);    % quadrotor real states
%     log_QuadStateCov = zeros(8, 8, cfg.logsize, model.nQuad);   % quadrotro state uncertainty
%     
%     log_QuadSet = zeros(3, cfg.logsize, model.nQuad);   % Set end-point for quadrotor
% 
%     log_ObsState = zeros(6, cfg.logsize, model.nObs);   % raw obstacle position
%     log_ObsStateCov = zeros(6, 6, cfg.logsize, model.nObs);       
end

%%  =======================================================================
%%% Inter-loop storage
% % Store predicted MPC stage vector z output
% QuadMPCplan = zeros(model.nvar, model.N, model.nQuad);
% 
% % Store predicted position uncertainty covariance information
% QuadPosCovPlan = zeros(6, model.N, model.nQuad);
% 
% % Store Quadrotor State information for Graphic Communicator
% G_QuadrotorState = zeros(cfg.Graphic.nQuadStates, model.nQuad);
% 
% % Store Obstacle State information for Graphic Communicator
% G_ObstacleState = zeros(cfg.Graphic.nObsStates, model.nObs);
% 
% % Set default quadrotor setpoints
% setpointsQuadrotor = zeros(3,2,nQuad);
% G_QuadrotorSetpoint = quadEndPos;
% for k = 1 : nQuad
%     setpointsQuadrotor(:,:,k)  = [quadStartPos(:,1),quadEndPos(:,k)];
% end
% G_flagnewQuadrotorSetpoint = 1;
% G_flagTime = 0;

% if cfg.SetPointGui == 0
%     GraphicCom.setQuadrotorSetpoint(setpointsQuadrotor(:,2,:));
% end

% Timer
fprintf('[%s] Looping... \n',datestr(now,'HH:MM:SS'));

cnt = 0;   % Number of controller loops performed
t_start = seconds(rostime('now'));      % start time before the entire loop
t_prev = t_start;                       % previous time, used to record the start time before one entire loop
                                        % here is intilization

%% Desired frequency                                        
F_desired = 20; %(Hz)             

%% MAIN LOOP
tic;
while(true)        
    % Flags    
    exitflag   = 0;
    infeasible = 0;  
          
    % Loop forces control to run at F_desired   
    while(1/(toc) > F_desired)                     
    end    
    tic;
    
    %% Timer
    t_now = seconds(rostime('now'));
    
    t_dtmeasured = t_now - t_prev;      % computation time of the following one entire loop 
    t_dtreal = t_dtmeasured; 
    
    % For graphic communicator elapsed time     ??? 
    G_Time = t_now - t_start;

    if ~cfg.ModeSim             % in experiment
        % Pause is necessary, otherwise Joy commands are not sent
        pause(0.001); 
        RTF = t_dtmeasured/t_dtreal;    % ???
        
    elseif cfg.UseRealdtSim     % in simulation, and use real dtSim
        RTF = t_dtmeasured/t_dtreal;
        
    else                        % in simulation, but use predefined dt
        % If simulated, set fixed step size. The t_dtreal is saved to determine real time factor. 
        t_dtmeasured = cfg.dtSim;
        RTF = t_dtmeasured/t_dtreal; 
        
        % If RTF is > 1 then add pause to bring RTF = 1
        % ???
        if RTF > 1.0
            pause(t_dtmeasured - t_dtreal);
            RTF = 1.0;
        end
    end
    
    if(t_dtmeasured > cfg.mindt)        % ??? 
        
        % Timer and loop count
        t_prev = t_now;
        cnt = cnt + 1;
        
        % print in screen every ten loops
        if (mod(cnt,10) == 0 && cfg.verbose)        % print every 10 run
            fprintf('Frequency: %.3f Hz, RTF: %.2f\n',1/t_dtmeasured, RTF)
%             fprintf('Time for individual control: %.3f ms\n',t_single*1000)
%             fprintf('Time for team control: %.3f ms\n',t_team*1000)
        end
           
        %% Communicate with CGraphic / Get Visual ROS data (subscribers)
        if cfg.GraphicCom == 1
            % Get quadrotor setpoints (only the end points)
            % setpoint from GUI
            if cfg.SetPointGui == 1
                [G_QuadrotorSetpoint, ~, G_flagnewQuadrotorSetpoint] = GraphicCom.getQuadrotorSetpoint();
            else
            % Automated setpoint
                %% read both from data
%                 changeStep = 80;
%                 if mod(cnt, changeStep) == 0 && cnt ~= 0
%                     for kQuad = 1:model.nQuad
%                         G_QuadrotorSetpoint(:, kQuad) = wayPoints(:, cnt/changeStep,kQuad);
%                     end
%                     G_flagnewQuadrotorSetpoint = 1;
%                 end
                
                %% the first drone follows a shape, second read data
                % first
                if G_flagTime
                    % Ellipse
%                   ta = 1.6;
%                   tb = 0.4;
%                   tperiod = 8; %s
%                   set_x = ta*cos(2*pi*G_Time/tperiod);
%                   set_y = tb*sin(2*pi*G_Time/tperiod);
            
                    % Eight figure
                    ta = 1.2;
                    tb = 0.8;
                    tperiod = 18; %s
                    set_x = ta*cos(2*pi*G_Time/tperiod);
                    set_y = tb*sin(4*pi*G_Time/tperiod);

                    G_QuadrotorSetpoint(:,1) = [set_x; set_y; 1.1];
                    G_flagnewQuadrotorSetpoint = 1;
                end
                % second
                changeStep = 60;
                if mod(cnt, changeStep) == 0 && cnt ~= 0
                    G_QuadrotorSetpoint(:, 2) = wayPoints(:, cnt/changeStep,2);
                    G_flagnewQuadrotorSetpoint = 1;
                end
            end
            
            if G_flagnewQuadrotorSetpoint
                % Copy the old end points to start points of setpoints and add new setpoints
                setpointsQuadrotor = [setpointsQuadrotor(:,2,:), reshape(G_QuadrotorSetpoint,cfg.Graphic.nQuadSetpoints,1,model.nQuad)];
                if cfg.SetPointGui == 0
                    GraphicCom.setQuadrotorSetpoint(setpointsQuadrotor(:,2,:));
                end
            end
                                   
            % Get reset trigger if using simulated obstacles
            if cfg.obsSim
                [~, ~, G_flagnewObstacleSimReset] = GraphicCom.getObstacleSimReset();
                if G_flagnewObstacleSimReset
                    % Reinitialise simulated objects
                    for i = 1:model.nObs
                        Obstacle(i).sim_init(obsStartPos(:,i), obsEndPos(:,i), obsTimer(:,i));
                    end
                    % Reset the trigger
                    GraphicCom.setObstacleSimReset(false);
                end
            end
        end
        
        %% dt_MPC
%         dt_MPC = min(t_dtmeasured,cfg.dtSim);            % ??? TODO: Check if this is good! Why???
                                                             % this is important for simulation
        
%         %% Get  predicted positions of obstacles 
%         predictionObstacle = zeros(20,model.N, model.nObs); % using predicted position with uncertainty covariance now for MPC
% 
%         for jObs = 1:model.nObs
%             % Get obstacle's current position and velocity, using Kalman Filter 
%             Obstacle(jObs).getSystemState(t_dtmeasured); 
%             
%             % Obtain predicted motion
%             [obsPosN, obsVelN, covPosP_N_Vec] = Obstacle(jObs).getPredictionN(model.N, dt_MPC); 
%             predictionObstacle(index.obsMaintainProba,:,jObs)   = Obstacle(jObs).m_maintain_proba; 
%             predictionObstacle(index.obsMaintainPos,:,jObs)     = obsPosN(:,:,1); 
%             predictionObstacle(index.obsMaintainPosCov,:,jObs)  = covPosP_N_Vec(:,:,1); 
% %             predictionObstacle(index.obsStopProba,:,jObs)       = Obstacle(jObs).m_stop_proba; 
% %             predictionObstacle(index.obsStopPos,:,jObs)         = obsPosN(:,:,2); 
% %             predictionObstacle(index.obsStopPosCov,:,jObs)      = covPosP_N_Vec(:,:,2); 
% 
%             % LOG current obstacle position and velocity
%             log_ObsState(:,cnt,jObs) = [obsPosN(:,1,1); obsVelN(:,1,1)]; 
% 
%             % COM: Store Obstacle State for communication
%             G_ObstacleState(:,jObs) = [obsPosN(:,1,1); obsVelN(:,1,1)]; 
% 
%              % If simulated obstacles, simulate step (for next loop)
%             if cfg.obsSim
%                 Obstacle(jObs).sim_step(t_dtmeasured);
%             end       
%         end

        %% Control Loop
%         t_team_start = tic;
        for iQuad = 1:1
            % Set real-time parameter p for all stages N
            % set real time yaw
            if cfg.UseRealYaw == 1
                pr.realyaw = pr.realyawQuad(iQuad);
            end
            
            % Obtain quadrotor filtered position, velocity and euler with uncertainty information
            % yaw is updated
            Quadrotor(iQuad).getSystemState(t_dtmeasured);
            pr.realyawQuad(iQuad) = Quadrotor(iQuad).m_euler_filt(3);
            
            % Obtain real time predicted uncertainty information
%             if cfg.Application == 'chance'  || cfg.Application == 'bound'
% %                 QuadPosCovPlan(:,:,iQuad) = Quadrotor(iQuad).getPredictionCovN(model.N, dt_MPC);
%                   QuadPosCovPlan(:,:,iQuad) = Quadrotor(iQuad).getPredictionFullCovN(model.N, QuadMPCplan(index.inputs,:,iQuad), dt_MPC);
%             end            
%             
%             % set real time parameter
%             if cfg.Application == 'basic'
%                 Quadrotor(iQuad).m_controllerMPC.setRTParameter(QuadMPCplan,setpointsQuadrotor(:,:,iQuad),predictionObstacle(index.obsMaintainPos,:,:),pr.realyaw,dt_MPC,iQuad);
%             elseif cfg.Application == 'chance'
%                 Quadrotor(iQuad).m_controllerMPC.setRTParameterCov(...
%                     QuadMPCplan, QuadPosCovPlan, setpointsQuadrotor(:,:,iQuad),...
%                     predictionObstacle, pr.realyaw, dt_MPC, pr.auxProbaObs, pr.auxProbaQuad,...
%                     pr.weightCollObs, pr.weightCollQuad, pr.weightInput, pr.weightWaypoint,...
%                     iQuad);
%             elseif cfg.Application == 'bound'
%                 Quadrotor(iQuad).m_controllerMPC.setRTParameterCov(...
%                     QuadMPCplan, QuadPosCovPlan, setpointsQuadrotor(:,:,iQuad),...
%                     predictionObstacle, pr.realyaw, dt_MPC, pr.Mahalanobis, pr.Mahalanobis,...
%                     pr.weightCollObs, pr.weightCollQuad, pr.weightInput, pr.weightWaypoint,...
%                     iQuad);
%             elseif cfg.Application == 'full'
%                 Quadrotor(iQuad).m_controllerMPC.setRTParameterFull(...
%                     QuadMPCplan,setpointsQuadrotor(:,:,iQuad),...
%                     predictionObstacle, pr.realyaw, dt_MPC, pr.auxProbaObs, pr.auxProbaQuad,...
%                     pr.weightCollObs, pr.weightCollQuad, pr.weightInput, pr.weightWaypoint,...
%                     iQuad);
%             end
                
            %% Step controller Quadrotor (causes movement)
%             t_single_start = tic;
            if ~cfg.ModeSim && (cfg.UseOpti || cfg.UseOpti)             % in real experiment
                % Get and set control mode of quadrotor
                flagNewMPC = Joy.setDroneState(Quadrotor(iQuad)); 
                
                % If MPC mode is just initiated, then reinitialise MPC with current x_start (current position)
                % This overwrites the Z_plan with current position. This is necessary otherwise the Z_plan from
                % the previous MPC mode session is used for optimisation which destabilises the controller.             
%                 if flagNewMPC
%                     x_start = zeros(model.neq,1); 
%                     x_start(index.x.posStates) = Quadrotor(iQuad).m_Zk(index.posStates);  % Set starting position to real current position
%                     Quadrotor(iQuad).m_controllerMPC.initialise(x_start);
%                 end
                % Compute and execute control action for one step in experiments
                [exitflag, info] = Quadrotor(iQuad).step(Joy);
            elseif cfg.ModeSim                                          % in simulation
                [exitflag, info] = Quadrotor(iQuad).stepSim(Joy,t_dtmeasured);
            end
                      
            % Check if quadrotor is being tracked and solver has solved
            if ~Quadrotor(iQuad).m_visible
                infeasible = 1;
                fprintf('[%s] ALERT: Quadrotor %i is not tracked \n',datestr(now,'HH:MM:SS'),iQuad);
            end

            if exitflag ~=1
                infeasible = 1;
                fprintf('[%s] ALERT: Quadrotor %i exitflag NOT 1 \n',datestr(now,'HH:MM:SS'),iQuad);

                if exitflag == -100
                    fprintf('[%s] LICENSE: License error MPC, exiting... \n',datestr(now,'HH:MM:SS'),iQuad);
                    exit
                end
            end

            % Store Quadrotor State for communication
%             if cfg.Application == "basic"
%                 % not communicate position uncertainty
%                 G_QuadrotorState(:,iQuad) = [Quadrotor(iQuad).m_Zk(index.inputs(1):index.slackEnv,:); ...
%                     Quadrotor(iQuad).m_pos_filt;  Quadrotor(iQuad).m_vel_filt; Quadrotor(iQuad).m_euler_filt];
%             elseif cfg.Application == "chance" || cfg.Application == 'bound' || cfg.Application == 'full'
%                 % communicate uncertainty
%                 G_QuadrotorState(:,iQuad) = [Quadrotor(iQuad).m_Zk(index.inputs(1):index.slackEnv,:); ...
%                     Quadrotor(iQuad).m_pos_filt;  Quadrotor(iQuad).m_vel_filt; Quadrotor(iQuad).m_euler_filt; ...
%                     Quadrotor(iQuad).m_pos_filt_P]; 
%             end
            
            % Log MPC plan if in MPC mode
%             if Quadrotor(iQuad).m_mpc == 1
%                 QuadMPCplan(:,:,iQuad) = Quadrotor(iQuad).m_controllerMPC.getZplan(); % Store MPC plan of quadrotor for inter-loop
%             end
            
%             if cfg.Application == 'full'
%                 QuadPosCovPlan(1:3,:,iQuad) = QuadMPCplan(index.posCov,:,iQuad);
%             end

            %% Infeasibility trigger
            % If any quadrotor has an infeasible solution, of not track, land the quadrotors 

            %%% --- DEBUG: BELOW LINE UNCOMMENT FOR DEBUG ---
            %infeasible = 0; % FOR DEBUG UNCOMMENT
            %%% ---- ABOVE LINE UNCOMMENT FOR DEBUG ---

%             if ~cfg.ModeSim && infeasible
%                 fprintf('[%s] ALERT: Infeasible, landing quadrotors... \n',datestr(now,'HH:MM:SS'));
%                 % TODO: Uncomment and return to original
% %                 while(true)
% %                     % Keep sending land command until program is terminated
% %                     for i = 1:model.nQuad
% %                         Quadrotor(i).land();
% %                     end
% %                     pause(0.5);
% %                 end
% 
%                 % If MPC infeasible, then put quadrotor in manual mode!
%                 Quadrotor(iQuad).m_mpc = 0; Quadrotor(iQuad).m_auto = 0; 
%                 Quadrotor(iQuad).m_manual = 1;
%             end

            %% Logging
            % LOG Quadrotor and payload data
             if cfg.DoLog
                 log_Time(:,cnt) = [t_dtreal; t_dtmeasured];
%                log_TotTime(cnt) = G_Time;
%                log_QuadZk(:,cnt,iQuad) = Quadrotor(iQuad).m_Zk; % Full z_k vector
%                log_QuadEuler(:,cnt,iQuad) = [Quadrotor(iQuad).m_euler_filt]; % Euler angles
%                log_QuadPos(:,cnt,iQuad) = Quadrotor(iQuad).m_pos_raw;      % filted position data
                 log_QuadInputs(:, cnt, iQuad) = [Quadrotor(iQuad).m_ppi_input];
                 log_QuadStatesReal(:, cnt, iQuad) = [Quadrotor(iQuad).m_pos_real; Quadrotor(iQuad).m_euler_real];
                 log_QuadStatesEstimated(:, cnt, iQuad) = [Quadrotor(iQuad).m_pos_estimated; Quadrotor(iQuad).m_vel_estimated; Quadrotor(iQuad).m_euler_estimated; Quadrotor(iQuad).m_ang_estimated]; %pos, vel, euler, euler vel
                 log_QuadStatesEstimatedCov(:, cnt, iQuad) = [Quadrotor(iQuad).m_pos_estimated_cov; Quadrotor(iQuad).m_vel_estimated_cov; Quadrotor(iQuad).m_euler_estimated_cov]; %pos cov vel cov euler cov
%                log_QuadStateCov(:, :, cnt, iQuad) = Quadrotor(iQuad).m_state_full_filt_P;
%                
%                log_QuadSet(:,cnt,iQuad) = setpointsQuadrotor(:,2,iQuad);   % Set end-point for quadrotor
%                 
%                log_QuadMPCplan(:,:,cnt,iQuad) = QuadMPCplan(:,:,iQuad);    % Logs the MPC plan
% 
%                 % Logging only possible in MPC mode
%                if Quadrotor(iQuad).m_mpc == 1           
%                    log_MPC(1,cnt,iQuad) = info.solvetime;
%                    log_MPC(2,cnt,iQuad) = info.it;
%                    log_MPC(3,cnt,iQuad) = info.pobj;
%                end
             end
        end
%         t_team = toc(t_team_start);

        %% Send data through CGraphic to visualisation
        if cfg.GraphicCom == 1
            % Send mode vector, function evalution will convert it to string to send over ROS
            % Identify mode as string
            G_mode = 'Not set'; 
            if Quadrotor(1).m_mpc == 1
                G_mode = 'MPC'; 
            elseif Quadrotor(1).m_auto == 1
                G_mode = 'Auto'; 
            elseif Quadrotor(1).m_manual == 1
                G_mode = 'Manual'; 
            end     
            if cfg.ModeSim == 1
                G_mode = 'MPC - Simulated'; 
            end
            if cfg.Application == 'basic'
                GraphicCom.ControllerSend(G_mode, QuadMPCplan(index.posStates,:,:), G_QuadrotorState, G_ObstacleState);
                G_flagnewQuadrotorSetpoint = 0;
                G_flagTime = 1;
            elseif cfg.Application == 'chance' || cfg.Application == 'bound' || cfg.Application == 'full'
                GraphicCom.ControllerSendCov(G_mode, QuadMPCplan(index.posStates,:,:), QuadPosCovPlan, G_QuadrotorState, G_ObstacleState);
                G_flagnewQuadrotorSetpoint = 0;
                G_flagTime = 1;
            end
        end
      
    end
    
end

if cfg.DoLog
    save_log
end
