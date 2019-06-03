classdef CDrone < handle
    % Common base class for drones
    
    properties
        
        %% type and identifier
        m_type          =   1;              % drone type, 1-Bebop2; 2-Mambo
        m_id            =   1;              % ID
        
        %% shared variables local copy
        m_share         =   [];
        
        %% physical parameters (constants with default values)
        m_radius_real                       % real radius, m
        m_radius_coll                      % collision radius, m
        m_maxAngleRad                       % maximum tilt (pitch/roll) angle, rad
        m_maxVelZ                           % maximum vertical velocity, m/s
        m_maxRotSpeed                       % maximum rotation speed, rad/s
        m_sensingRange                      % maximum sensing range, m
        m_commRange                         % maximum communication range, m           
        
        %% dynamics model
        m_dyn           =   [];
        
        %% modes
        m_flying        =   0;              % is drone flying 
        m_manual        =   0;              % is it manumally contrlled
        m_ppi           =   0;              % is it flying ppi
        m_mpc           =   0;              % if in MPC mode
        m_auto          =   0;              % if in auto LQR mode
        
        %% states and attitudes
        % real position and attitude (from mocap)
        m_pos_real      =   zeros(3,1);     % real position, m
        m_vel_real      =   zeros(3,1);     % real velocity (only used in simulation)
        m_q_real        =   zeros(4,1);     % real quaternion
        m_euler_real    =   zeros(3,1);     % real euler angle, rad
        % raw observation
        m_pos_raw       =   zeros(3,1);     % observed position raw data, m
        m_q_raw         =   zeros(4,1);     % observed quaternion
        m_euler_raw     =   zeros(3,1);     % observed attitude (pitch, roll, yaw) raw data, rad
        % filted state with uncertainty
        m_pos_filt      =   zeros(3,1);     % filted position, m
        m_vel_filt      =   zeros(3,1);     % filted velocity, m
        m_euler_filt    =   zeros(3,1);     % filted euler angels (pitch, roll, yaw), rad
        m_state_filt_P  =   zeros(6,6);     % state error covariance matrix from estimation
        m_pos_filt_P    =   zeros(6,1);     % position uncertainty covariance matrix, in vector form 
        m_state_full_filt_P = zeros(8,8);   % full state error covariance matrix from estimation
        m_state_full_filt_P_Vec = zeros(15,1);
        
        %% estimated state with bebop2_toolbox. Geeft geen hoeksnelheden
        m_pos_estimated     = zeros(3,1);
        m_vel_estimated     = zeros(3,1);        
        m_euler_estimated   = zeros(3,1);
        m_ang_estimated     = zeros(3,1);
        m_pos_estimated_cov = zeros(9,1); %Row-major representation of the 6x6 covariance matrix
        m_vel_estimated_cov = zeros(9,1); %zie documentation http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TwistWithCovariance.htm       
        m_euler_estimated_cov=zeros(9,1);
        
        %% PPI
        m_ppi_index_phi     = 1;            %used to count through ppi vectors
        m_ppi_index_theta   = 1;
        m_ppi_index_z       = 1;
        
        m_ppi_theta         = zeros(1,1);   %holds ppi vector for theta
        m_ppi_phi           = zeros(1,1);   %holds ppi vector for phi
        m_ppi_z             = zeros(1,1);   %holds ppi vector for z
        
        m_ppi_length_phi    = 100;          %holds length of ppi vectorss
        m_ppi_length_theta  = 100;
        m_ppi_length_z      = 100;
        
        m_ppi_input         = zeros(3,1);
        
        m_ppi_controlYaw    = 0;            %If yaw should be controlled for
        m_ppi_yc_kp         = 1;            %kp PD yaw control
        m_ppi_yc_kd         = 1;            %kd PD yaw control
        
        %% predicted state and uncertainty information
        m_pos_N         =   [];             % planned position
        m_vel_N         =   [];             % planned velocity
        m_covP_N        =   [];             % predicted state uncertainty covariance
        m_covPosP_N     =   [];             % predicted position uncertainty covariance
        m_covPosP_N_Vec =   [];             % predicted position uncertainty covariance, in vector form
                
        %% vertical acceleration
        m_accz          =   0;              % saved vertical acceleration
        
        %% estimator
        m_estimatorState=   [];             % estimating drone state (position, velocity)
        m_estimatorStateFull= [];           % estimating drone state (position, velocity, att)
        m_estimatorAtt  =   [];             % estimating drone attitude
        m_estimatorInput=   [];             % estimating drone vertical acceleration
        
        %% controller and control inputs
        m_controllerMPC =   [];             % MPC controller
        m_controlInputs =   zeros(3,1);     % control inputs (commanded pitch, roll, vz)
                                            % in standard unit, computed from the controller
        m_useBodyControl=   0;              % if body control is used
        m_useYawControl =   0;              % if yaw motion is controlled
        
        %% MPC Control Specific variables
        m_Xk            =   [];             % store initial conditions for MPC
        m_Zk            =   [];             % store current stage from MPC
        m_Zk2           =   [];             % store next stage from MPC
        m_MPCPlan       =   [];             % store planned MPC plan
        
        %% motion capture system
        m_useOpti       =   1;              % motion capture system in use
        m_useVicon      =   0;              % use Vicon?
        m_visible       =   1;              % if the drone is visible by Mocap
        
        %% ROS subscriber/publisher
        % Publishers
        m_takeoff_pub
        m_takeoff_msg                       % take off
        m_land_pub
        m_land_msg                          % land
        m_setp_vel_pub
        m_setp_vel_msg                      % send commands to the drone
        
        % Subscribers
        m_pose_sub                          % Quadrotor position, subscribe from Mocap
        m_estimated_sub                     % Quadrotor state estimation, subscribe from toolbox
        
        %% for simulation 
        m_ModeSim       =   1;              % in simulation mode?
        m_simAccNoise   =   0;              % adding noise to acceleration
        m_simObserveNoise = 0;              % adding noise to observation
        m_simControlNoise = 0;              % adding noise to control inputs
        
    end

    
    methods
        
        %%  =======================================================================
        function obj = CDrone(quadType, quadID, ~, share)
            % constructor            
            obj.m_type  =   quadType;               % specify the type of the drone
            obj.m_id    =   quadID;                 % specify the ID of the drone
%             obj.m_dyn   =   quadDyn;                % specify the dynamics model
            obj.m_share =   share;                  % local copy of configuration
            
            % if simulated, then quadrotor in MPC by default
            if share.cfg.ModeSim
                obj.m_mpc    = 1;
                obj.m_manual = 0;
            end
            
            % set flag information
            obj.m_useOpti  = share.cfg.UseOpti;
            obj.m_useVicon = share.cfg.UseVicon;
            obj.m_useBodyControl = share.cfg.UseBodyControl;
            obj.m_ModeSim  = share.cfg.ModeSim;
            
            % noise simulation flag
            obj.m_simAccNoise     = share.cfg.simQuadAccNoise;
            obj.m_simObserveNoise = share.cfg.simQuadObserveNoise;
            obj.m_simControlNoise = share.cfg.simQuadControlNoise;
                 
            % set input limits
            obj.m_maxAngleRad = share.pr.input.maxAngleRad;
            obj.m_maxVelZ     = share.pr.input.maxVelZ;
            obj.m_maxRotSpeed = share.pr.input.maxRotSpeed;
            
            %% Set up PPI vectors
            [obj.m_ppi_phi, obj.m_ppi_length_phi] = preplannedinput(62);     %pitch
            [obj.m_ppi_theta, obj.m_ppi_length_theta] = preplannedinput(62);  %roll            
            [obj.m_ppi_z, obj.m_ppi_length_z] = preplannedinput(62);           %z
            
            %% UKF for full state
%             xDim = 8;
%             uDim = 3;
%             zDim = 5;
%             ffunc = @uavdynamics_simple;
%             hfunc = @(x,p) [x(1);x(2);x(3);x(7);x(8)];
%             Q = share.pr.state.Q_Full_Filt;
%             R = share.pr.state.R_Full;
%             P0 = share.pr.state.P_Full;
%             x0 = zeros(xDim, 1);
%             obj.m_estimatorStateFull = CUncKalmanFilter_Hybrid(xDim, uDim, zDim, ffunc, hfunc, Q*share.cfg.dtSim, R, x0, P0);
%             
        end
        
        %%  =======================================================================
        function initialise(obj)
            % Initialise the intial conditions for the MPC solver with
            % x_start and predicted stage variables with zeros
%             obj.m_Xk = x_start; % In the step function these are overwritten even as initial conditions using MCS data
%             
%             obj.m_Zk = zeros(MPCnvar,1);
            
            % position initialization, used in simulation
%             obj.m_pos_real = x_start(obj.m_share.index.x.posStates);
%             obj.m_vel_real = x_start(obj.m_share.index.x.velStates); 
%             obj.m_euler_real(3) = obj.m_share.pr.realyawQuad(obj.m_id);
        end
       
        %%  =======================================================================
        function initialiseROS(obj,mapping)
            % Initialise ROS publishers and subscribers for Drone
            quadID = mapping(obj.m_id);
            
            %% ROS publishers to command Quad
            obj.m_takeoff_pub = rospublisher(['/q',num2str(quadID),'/real/takeoff'], 'std_msgs/Empty','IsLatching', false);
            obj.m_takeoff_msg = rosmessage(obj.m_takeoff_pub);
            
            obj.m_land_pub = rospublisher(['/q',num2str(quadID),'/real/land'], 'std_msgs/Empty','IsLatching', false);
            obj.m_land_msg = rosmessage(obj.m_land_pub);
            
            obj.m_setp_vel_pub = rospublisher(['/q',num2str(quadID),'/real/cmd_vel'], 'geometry_msgs/Twist','IsLatching', false);
            obj.m_setp_vel_msg = rosmessage(rostype.geometry_msgs_Twist);
                        
            %% ROS subscribers to obtain Motion Capture System data
            if obj.m_useOpti
                
                % Use the optitrack pose information
                name = ['/Bebop',num2str(quadID),'/pose'];
                obj.m_pose_sub = rossubscriber(name,'geometry_msgs/PoseStamped');
               
            elseif obj.m_useVicon
                
                % Use the vicon transform information
                name = ['vicon/Bebop',num2str(quadID),'/Bebop',num2str(quadID)];
                obj.m_pose_sub = rossubscriber(name,'geometry_msgs/TransformStamped');
                               
            end
            
            %% ROS subscribers om estimated velocity data uit lezen uit bebop2_toolbox package
            name = ['/Bebop',num2str(quadID),'/bebopFullStateEstimation'];
            obj.m_estimated_sub = rossubscriber(name, 'bebop2_msgs/FullStateWithCovarianceStamped');
            
        end
        
        %%  =======================================================================
        function getObservedSystemState(obj)
            % Get observed real-time position and attitude of the drone in experiment, get only raw data
            %% Check if drone is visible
            msg = obj.m_pose_sub.LatestMessage;
%             tmpseq = msg.Header.Seq;
%             if tmpseq > obj.PosFrameSequence
%                 obj.PosFrameSequence = msg.Header.Seq;
%                 obj.f_notVisible = 0;
%             else
%                 obj.f_notVisible = 1;
%             end
                     
            %% Obtain data
            if obj.m_useOpti
                % Conversion Opti -> MATLAB
                % x = x
                % y = -z
                % z = y
                
                obj.m_pos_real = [ msg.Pose.Position.X;
                    msg.Pose.Position.Y;
                    msg.Pose.Position.Z];
                
                obj.m_q_real = [ msg.Pose.Orientation.W;
                    msg.Pose.Orientation.X;
                    msg.Pose.Orientation.Y;
                    msg.Pose.Orientation.Z];
                
                % --- Debug lines ---
                %fprintf('X %f, Y %f, Z %f \n', msg.Pose.Position.X, msg.Pose.Position.Y, msg.Pose.Position.Z);
                %fprintf('X %f, Y %f, Z %f \n', msg.Pose.Orientation.X, msg.Pose.Orientation.Y, msg.Pose.Orientation.Z);
                               
            else
                
                obj.m_pos_real = [ msg.Transform.Translation.X;
                    msg.Transform.Translation.Y;
                    msg.Transform.Translation.Z];
                
                obj.m_q_real = [ msg.Transform.Rotation.W;
                    msg.Transform.Rotation.X;
                    msg.Transform.Rotation.Y;
                    msg.Transform.Rotation.Z];
            end            
            
            %% Post Process - compute Quadrotor euler angles
            
            %%% Quadrotor
            % Get rotation matrix to identify quadrotor attitude
            % Rotation matrix to Euler angle conversion LaValle http://planning.cs.uiuc.edu/node103.html
            R_cw_real = quat2rotm(obj.m_q_real');

            % real pitch, roll and yaw angle from Mocap   
            obj.m_euler_real(1)   = atan2(-R_cw_real(3,1),sqrt(R_cw_real(3,2)^2+R_cw_real(3,3)^2));
            obj.m_euler_real(2)   = atan2(R_cw_real(3,2),R_cw_real(3,3));
            obj.m_euler_real(3)   = atan2(R_cw_real(2,1),R_cw_real(1,1));
            
            dpos   = zeros(3,1);
            deuler = zeros(3,1);
            
            % add noise if necessary
            if obj.m_simObserveNoise
                for i = 1 : 3
                   dpos(i) = random('Normal', 0, sqrt(obj.m_share.pr.noise.Observe(i,i)));
                end
                for i = 1 : 2
                   deuler(i) = random('Normal', 0, sqrt(obj.m_share.pr.noise.Observe(i+3,i+3)));
                end
            end
            obj.m_pos_raw   = obj.m_pos_real + dpos;
            obj.m_euler_raw = obj.m_euler_real + deuler;
            
        end
        
        %%  =======================================================================
        function getObservedSystemStateSim(obj)
            % Get drone position and attitude (euler) in
            % simulation
            obj.m_pos_raw   = obj.m_pos_real;            
            obj.m_euler_raw = obj.m_euler_real;
            
            %% adding position observation noise
            if obj.m_simObserveNoise              % if simulated with position observation noise
%                 % ***** adding the observation noise here *****
%                 dpos = zeros(3,1);                  % position observation error
%                 for i = 1 : 3
%                    dpos(i) = random('Normal', 0, sqrt(obj.m_share.pr.noise.Observe(i,i)));
%                 end
%                 obj.m_pos_raw = obj.m_pos_raw + dpos;
%                 dpr = zeros(2,1);
%                 for i = 1 : 2
%                    dpr(i) = random('Normal', 0, sqrt(obj.m_share.pr.noise.Observe(i+3,i+3)));
%                 end
%                 obj.m_euler_raw(1:2) = obj.m_euler_raw(1:2) + dpr;
            end
        end
        
        %%  =======================================================================
        function getSystemState(obj,dt)
            % Get drone position, velocity and attitude (euler)
            % information with filtered version. 
            
            %% Firstly, get observed real-time position and attitude of the drone
            if ~obj.m_ModeSim               % if in experiment
                getObservedSystemState(obj);
            else                            % if in simulation
                getObservedSystemStateSim(obj);
            end
            
            if obj.m_ModeSim == 1 && obj.m_share.cfg.FullSim == 1
                % not use estimator
                obj.m_pos_filt = obj.m_pos_real;
                obj.m_vel_filt = obj.m_vel_real;
                obj.m_euler_filt = obj.m_euler_real;
            else
                %% Read data from bebop2_toolbox
                msg = obj.m_estimated_sub.LatestMessage;
                
                obj.m_pos_estimated = [msg.State.X;
                    msg.State.Y;
                    msg.State.Z];
                
                obj.m_vel_estimated = [msg.State.XDot; %XDot ipv x_dot omdat matlab zo custom messages omzet
                    msg.State.YDot;
                    msg.State.ZDot];
                
                obj.m_euler_estimated = [msg.State.Pitch;
                    msg.State.Roll;
                    msg.State.Yaw];               
                
                obj.m_ang_estimated = [msg.State.PitchDot;
                    msg.State.RollDot;
                    msg.State.YawDot];
                
                obj.m_pos_estimated_cov     = msg.State.PosCov;
                obj.m_vel_estimated_cov     = msg.State.VelCov;
                obj.m_euler_estimated_cov   = msg.State.EulerCov;
                
%                 obj.m_pos_estimated = [ msg.Pose.Pose.Position.X;
%                     msg.Pose.Pose.Position.Y;
%                     msg.Pose.Pose.Position.Z];
%                 
%                 obj.m_q_estimated = [ msg.Pose.Pose.Orientation.W;
%                     msg.Pose.Pose.Orientation.X;
%                     msg.Pose.Pose.Orientation.Y;
%                     msg.Pose.Pose.Orientation.Z];
%     
%                 obj.m_vel_estimated = [msg.Twist.Twist.Linear.X;
%                     msg.Twist.Twist.Linear.Y;
%                     msg.Twist.Twist.Linear.Z];                
%                 
%                 obj.m_ang_estimated = [msg.Twist.Twist.Angular.X;
%                     msg.Twist.Twist.Angular.Y;
%                     msg.Twist.Twist.Angular.Z];
%                 
%                 obj.m_pos_estimated_P = msg.Pose.Covariance;
%                 obj.m_vel_estimated_P = msg.Twist.Covariance;
%                 
%                 % Get rotation matrix to identify quadrotor attitude
%                 % Rotation matrix to Euler angle conversion LaValle http://planning.cs.uiuc.edu/node103.html
%                 R_cw_estimated = quat2rotm(obj.m_q_estimated');
% 
%                 % estimated pitch roll yaw angles
%                 obj.m_euler_estimated(1)   = atan2(-R_cw_estimated(3,1),sqrt(R_cw_estimated(3,2)^2+R_cw_estimated(3,3)^2));
%                 obj.m_euler_estimated(2)   = atan2(R_cw_estimated(3,2),R_cw_estimated(3,3));
%                 obj.m_euler_estimated(3)   = atan2(R_cw_estimated(2,1),R_cw_estimated(1,1));                
                       
                %%
                % estimator is also used in simulation
%                 %% EKF(or UKF) filter    
%                 % control inputs for state estimating: real pitch and roll,
%                 % angle, and commanded vz
%                 u = [obj.m_euler_raw(1:2); obj.m_controlInputs(3)];
%                 z = obj.m_pos_raw;
                 %[state_filt, covP_filt] = obj.m_estimatorState.step(u, dt, obj.m_share.pr, z);
%                 covPosP = covP_filt(1:3,1:3);

                %% UKF full state filter
%                 u = obj.m_controlInputs;
%                 z = [obj.m_pos_raw; obj.m_euler_raw(1:2)];
%                 p = obj.m_euler_raw(3);
%                 [state_filt, covP_filt] = obj.m_estimatorStateFull.step(u, dt, p, z);
%                 covPosP = covP_filt(1:3,1:3);
%                 obj.m_euler_filt(1:2) = state_filt(7:8);
%                 obj.m_euler_filt(3) = obj.m_euler_raw(3);

                %% Prepare output, after filting
%                 obj.m_pos_filt = state_filt(1:3);
%                 obj.m_vel_filt = state_filt(4:6);
%                 obj.m_state_full_filt_P = covP_filt;
%                 obj.m_state_filt_P = covP_filt(1:6,1:6);
%                 obj.m_pos_filt_P   = [diag(covPosP);covPosP(1,2);covPosP(1,3);covPosP(2,3)];
%                 % euler angles use the observed
% %               obj.m_euler_filt = obj.m_euler_raw;
% 
%                 % TODO
%                 obj.m_vel_real = obj.m_vel_filt;
% 
%                 if obj.m_share.cfg.Application == 'chance' || obj.m_share.cfg.Application == 'bound' || obj.m_share.cfg.Application == 'full'
%                         obj.m_state_full_filt_P_Vec = [diag(obj.m_state_full_filt_P); ...
%                             obj.m_state_full_filt_P(1,4); obj.m_state_full_filt_P(2,5); ...
%                             obj.m_state_full_filt_P(3,6); obj.m_state_full_filt_P(4,7); ...
%                             obj.m_state_full_filt_P(5,8); obj.m_state_full_filt_P(1,7); ...
%                             obj.m_state_full_filt_P(2,8)];
%                 end
            end
        end
        
        %%  =======================================================================
        function [covPosP_N_Vec, covP_N] = getPredictionFullCovN(obj, N, u_N, dt)
            % get predicted uncertainty covariance (position in vector form)
            obj.m_covP_N = zeros(8, 8, N); 
            obj.m_covPosP_N = zeros(3, 3, N); 
            obj.m_covPosP_N_Vec = zeros(6, N);
            
            % The current stage is set as the first stage             
            vecP = zeros(15,N);
            vecP(1:8,1)  = diag(obj.m_state_full_filt_P);       % convert current state covariance into vector form
            vecP(9,1)  = obj.m_state_full_filt_P(1,4);
            vecP(10,1) = obj.m_state_full_filt_P(2,5);
            vecP(11,1) = obj.m_state_full_filt_P(3,6);
            vecP(12,1) = obj.m_state_full_filt_P(4,7);
            vecP(13,1) = obj.m_state_full_filt_P(5,8);
            vecP(14,1) = obj.m_state_full_filt_P(1,7);
            vecP(15,1) = obj.m_state_full_filt_P(2,8);
            
            state_now = [obj.m_pos_filt; obj.m_vel_filt; obj.m_euler_filt(1:2); vecP(:,1)];
            
            % propagation, vector form of full state covariance
            p = obj.m_euler_filt(3);
            for i = 2 : N
                u = u_N(:,i-1);
                state_next = RK2(state_now, u, @uncertainty_dynamics_simple_fullUP, dt, p, 1);
                vecP(:, i) = state_next(9:end);
                state_now  = state_next;
            end
            
            % prepare output
            for i = 1 : N
                obj.m_covP_N(:,:,i) = diag(vecP(1:8,i));
                obj.m_covP_N(1,4,i) = vecP(9,i);
                obj.m_covP_N(2,5,i) = vecP(10,i);
                obj.m_covP_N(3,6,i) = vecP(11,i);
                obj.m_covP_N(4,7,i) = vecP(12,i);
                obj.m_covP_N(5,8,i) = vecP(13,i);
                obj.m_covP_N(1,7,i) = vecP(14,i);
                obj.m_covP_N(2,8,i) = vecP(15,i);
                obj.m_covPosP_N(:,:,i) = obj.m_covP_N(1:3,1:3,i);
                obj.m_covPosP_N_Vec(1:3,i) = vecP([1, 2, 3],i);
            end
            covPosP_N_Vec = obj.m_covPosP_N_Vec;
            covP_N = obj.m_covP_N;
                                        
        end
        
                %%  =======================================================================
        function [covPosP_N_Vec, covP_N] = getPredictionCovN(obj, N, dt)
            % get predicted uncertainty covariance (position in vector form)
            obj.m_covP_N = zeros(6, 6, N); 
            obj.m_covPosP_N = zeros(3, 3, N); 
            obj.m_covPosP_N_Vec = zeros(6, N);
            
            % The current stage is set as the first stage             
            vecP = zeros(21,N);
            vecP(1:6,1)  = diag(obj.m_state_filt_P);       % convert current state covariance into vector form
            vecP(7:21,1) = squareform(obj.m_state_filt_P - diag(diag(obj.m_state_filt_P)));
            
            % propagation, vector form of full state covariance
            p = [obj.m_share.pr.D.q; obj.m_share.pr.att.Ag1;0;0;0;diag(obj.m_share.pr.noise.Acc)];
            for i = 2 : N
                vecP(:,i) = RK2(vecP(:,i-1), [], @uncertainty_dynamics_ekf, dt, p, 1); 
            end
            
            % prepare output
            for i = 1 : N
                obj.m_covP_N(:,:,i) = diag(vecP(1:6,i)) + squareform(vecP(7:21,i));
                obj.m_covPosP_N(:,:,i) = obj.m_covP_N(1:3,1:3,i);
                obj.m_covPosP_N_Vec(:,i) = vecP([1, 2, 3, 7, 8, 12],i);
            end
            covPosP_N_Vec = obj.m_covPosP_N_Vec;
            covP_N = obj.m_covP_N;
                                        
        end
        
        %%  =======================================================================
        function [vel_d, vel_yaw, exitflag, info] = computeControlInputs(obj, Joy)
            % Compute control action
            % Default exitflag = 1, OKAY
            exitflag = 1; info = [];
            
            % Default desired velocity, yaw 
            % These are used by the SDK so values are ranging from -1 to 1 (min to max Angle/Velocity command)
            vel_d = [0;0;0]; 
            vel_yaw = 0;
                        
            %% MPC variables
            %%% MPC z vector (contains all stage vars, inputs, slack and states)
%             Z_k = zeros(obj.m_share.model.nvar,1);
%             Z_k(obj.m_share.model.xinitidx) = obj.m_Xk;                 % Set initial condition
%             
%             % Real data stage variables to set current intial conditions
%             Z_k(obj.m_share.index.posStates) = obj.m_pos_filt;          % Set position to estimated position
%             Z_k(obj.m_share.index.velStates) = obj.m_vel_filt;          % Set velocity to estimated velocity
%             Z_k(obj.m_share.index.prStates)  = obj.m_euler_filt(1:2);   % Set pr to estimated pr
%             
%             % only for planned with uncertainty
%             if obj.m_share.cfg.Application == 'full'
%                 Z_k(obj.m_share.index.fullCov)   = obj.m_state_full_filt_P_Vec;
%             end
%             
%             obj.m_Xk = Z_k(obj.m_share.model.xinitidx);                 % Save initial conditions
%             
            %% Depending on mode set the vel_d and vel_yaw
            % For manual and auto control, the inputs are placed into Z_k with the correct
            % units 
            
            if obj.m_manual
                % Manual joystick mode
                vel_d   = Joy.vel;
                vel_yaw = Joy.yawSpeed;
                obj.m_ppi_index_phi = obj.m_ppi_length_phi;
                obj.m_ppi_index_theta = obj.m_ppi_length_theta;
                obj.m_ppi_input = [0;0;0];
                
                % TODO: this fails if joystick has no connection
                %Z_k(obj.m_share.index.veldInputs) = vel_d(obj.m_share.index.veldInputs).*obj.m_maxAngleRad;
                %Z_k(obj.m_share.index.velzInputs) = vel_d(obj.m_share.index.velzInputs).*obj.m_maxVelZ;
            end
            
            if obj.m_ppi
                %Preplanned input mode
                %Activated when auto button is pressed when drone is flying                
                
                %Loop through input vector
                obj.m_ppi_index_theta = obj.m_ppi_index_theta + 1;
                if (obj.m_ppi_index_theta == obj.m_ppi_length_theta + 1) 
                    obj.m_ppi_index_theta = 1;
                end
                
                obj.m_ppi_index_phi = obj.m_ppi_index_phi + 1;
                if (obj.m_ppi_index_phi == obj.m_ppi_length_phi + 1) 
                    obj.m_ppi_index_phi = 1;
                end
                
                obj.m_ppi_index_z = obj.m_ppi_index_z + 1;
                if (obj.m_ppi_index_z == obj.m_ppi_length_z + 1)
                    obj.m_ppi_index_z = 1;
                end                                
                
                phi = obj.m_ppi_phi(obj.m_ppi_index_phi);               
                theta = obj.m_ppi_theta(obj.m_ppi_index_theta);
                
                z = obj.m_ppi_z(obj.m_ppi_index_z);
                obj.m_ppi_input = [phi; theta; z];
                vel_d = [phi; -theta; z]; %nodig ofzo
                vel_yaw = 0;  
                
%                 if (obj.m_ppi_controlYaw == 0) %PD control yaw to 0
%                     if (abs(obj.m_ang_estimated(3)) < 1) %omdat yaw soms verschiet
%                         m_ppi_yc_kd = .5;
%                     else 
%                         m_ppi_yc_kd = 0;
%                     end
%                    vel_yaw = obj.m_ppi_yc_kp*(-obj.m_euler_real(3)) ...
%                        + obj.m_ppi_yc_kd*(-obj.m_ang_estimated(3));                                      
%                 end                
            end
                        
%             if obj.m_auto
%                 % Auto LQR mode
%                 vel_d = obj.controller.step(obj.m_pos,obj.m_vel,obj.m_AUTOsetpoint); % Bring to start setpoint
%                 vel_yaw = Joy.yawSpeed;
%                 
%                 Z_k(obj.m_share.index.veldInputs) = vel_d(obj.m_share.index.veldInputs).*obj.m_maxAngleRad;
%                 Z_k(obj.m_share.index.velzInputs) = vel_d(obj.m_share.index.velzInputs).*obj.m_maxVelZ;
%             end
            
            if obj.m_mpc
%                 % MPC mode
%                 % Get the current and next step stage variables using ForcesPro
%                 [Z_k, Z_k2, exitflag, info] = obj.m_controllerMPC.step(obj.m_Xk);
%                                           
%                 if exitflag == 1
%                     % MPC feasible so execute planned commands
%                     scalefactor_pr = obj.m_maxAngleRad;
%                     scalefactor_velz = obj.m_maxVelZ;
%                     
%                     % Obtain and scale pitch roll commands between -1
%                     % and 1 using maxAngleRad
%                     pr_d = Z_k(obj.m_share.index.veldInputs);
%                     pr_scaled = pr_d/scalefactor_pr;
%                     if obj.m_type == 1
%                         pr_scaled(2) = -pr_scaled(2); % sign convention for commands bebop_autonomy (+ roll is -vely command, -roll is + vely command)
%                     elseif obj.m_type == 2
%                         pr_scaled(2) = pr_scaled(2); % for commands mambo
%                     end
%                         
%                     % Obtain desired vertical velocity and scale between -1
%                     % and 1 using maxVelZ
%                     velz_d = Z_k(obj.m_share.index.velzInputs);
%                     velz_scaled = velz_d/scalefactor_velz;
%                     
%                     vel_d = [pr_scaled; velz_scaled];
%                     
%                     % yaw is always controlled manually
%                     vel_yaw = Joy.yawSpeed;
%                 else
%                     % MPC infeasible so execute joystick commands
%                     vel_d = Joy.vel;
%                     vel_yaw = Joy.yawSpeed;
%                     % if in simulation, no Joystick, then set zero controls
%                     if obj.m_ModeSim == 1
%                         vel_d = [0;0;0]; 
%                         vel_yaw = 0;
%                     end
%                     % if MPC infeasible, then the above initial condition for
%                     % next loop would still lead to infeasibility. Reinitialize
%                     % MPC with current x_start (current position and velocity)
% %                     Z_k = zeros(obj.m_share.model.nvar,1); 
% %                     Z_k(obj.m_share.index.x.posStates) = obj.m_pos_filt;
% %                     Z_k(obj.m_share.index.x.velStates) = obj.m_vel_filt;
% %                     Z_k(obj.m_share.index.x.ssStates) = obj.m_xss_filt;
%                 end
            end
            
            % Set initial condition for next loop and save predicted 
            % next stage variables
%             obj.m_Xk = Z_k(obj.m_share.model.xinitidx);
%             obj.m_Zk = Z_k;
%             if obj.m_mpc
%                 obj.m_Zk2 = Z_k2;
%             end
            
            % Store current commanded inputs for use by Kalman Estimator and simulation
            % These are in real units [pitch (rad); roll (rad); gaz (m/s); yawSpeed (rad/s)];
%             obj.m_controlInputs = Z_k(obj.m_share.index.inputs(1):obj.m_share.index.inputs(end));
            
%             % vertical acceleration
%             gaz_u = obj.m_controlInputs(3);
%             % use for next-loop state estimation
%             obj.m_accz = obj.m_share.pr.att.Ag1 * obj.m_vel_filt(3) ...
%                 + obj.m_share.pr.att.Cg1*obj.m_share.pr.att.Bg1*gaz_u;
%             
            
        end
        
        %%  =======================================================================
        function setVelEarth(obj,vel_d_linear,vel_d_yaw)
            % Set piloting commands and send to the drone to execute, only used in experiment
            % X Y Z -> roll, pitch and gaz commands. Yaw command in angular
            if obj.m_useBodyControl == 0    
                
                % Derotate the vel_d_linear commands into world x,y,z coordinates
                % Direction of this derotation?
                R_deyaw                     = rotm3d(obj.m_euler_real(3),3); %eigenlijk filt
                velRotated                  = R_deyaw*vel_d_linear;
                
                obj.m_setp_vel_msg.Linear.X   = velRotated(1);
                obj.m_setp_vel_msg.Linear.Y   = velRotated(2);
                obj.m_setp_vel_msg.Linear.Z   = velRotated(3);
            else
                obj.m_setp_vel_msg.Linear.X   = vel_d_linear(1);
                obj.m_setp_vel_msg.Linear.Y   = vel_d_linear(2);
                obj.m_setp_vel_msg.Linear.Z   = vel_d_linear(3);
            end
            
            % Yaw command
            obj.m_setp_vel_msg.Angular.Z  = vel_d_yaw;
            
            % Send via publisher
            obj.m_setp_vel_pub.send(obj.m_setp_vel_msg);
        end
        
        %%  =======================================================================
        function [exitflag, info] = step(obj,Joy)
            % Execute control action for one step in experiments
            % firstly, compute the control action for one step
            % index is step in control loop
            [vel_d, vel_yaw, exitflag, info] = computeControlInputs(obj, Joy);
            
            % Command quadrotor to execute
            % Set drone commands vel_d = [pitch, roll, gaz] velYaw =
            % yaw unitless (-1 to 1 range)
            obj.setVelEarth(vel_d, vel_yaw);
        end
        
        %%  =======================================================================
        function [exitflag, info] = stepSim(obj,Joy,dt)
            % Execute control action for one step in simulation, by integrating the dynamical equations
            % firstly, compute the control action for one step
            [~, ~, exitflag, info] = computeControlInputs(obj, Joy);
            
%             % integrating the dynamical equations
%             if obj.m_share.cfg.FullSim ~= 1
%                 xNow = [obj.m_pos_real; obj.m_vel_real; obj.m_euler_real(1:2)];
%                 controlInputs = obj.m_controlInputs;
%                 % adding noise to controls
%                 if obj.m_simControlNoise                % if simulated with control inputs noise
%                     dcontrol = zeros(3,1);              % control inputs error
%                     for i = 1 : 3
%                        dcontrol(i) = random('Normal', 0, sqrt(obj.m_share.pr.noise.Control(i,i)));
%                     end
%                     controlInputs = obj.m_controlInputs + dcontrol;
%                 end
%                 xNext = RK2( xNow, controlInputs, @obj.m_dyn, dt, obj.m_euler_real(3), 1);
%                 obj.m_pos_real = xNext(obj.m_share.index.x.posStates);
%                 obj.m_vel_real = xNext(obj.m_share.index.x.velStates);
%                 obj.m_euler_real(1:2) = xNext(obj.m_share.index.x.prStates);
%             else
%                 obj.m_pos_real = obj.m_Zk2(obj.m_share.index.posStates);
%                 obj.m_vel_real = obj.m_Zk2(obj.m_share.index.velStates);
%                 obj.m_euler_real(1:2) = obj.m_Zk2(obj.m_share.index.prStates);
%             end
%             
        end
        
        %%  =======================================================================
        function takeoff(obj)
            % Takeoff Drone
            obj.m_takeoff_pub.send(obj.m_takeoff_msg);
            obj.m_flying = 1;
        end
        
        %%  =======================================================================
        function land(obj)
            % Land Drone
            obj.m_land_pub.send(obj.m_land_msg);
            obj.m_flying = 0;
        end
        
    end
    
end
