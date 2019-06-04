%% initialization script
% common pre-run set up for simulation and experiment

%% setup path
setPath; %%add all folders for matlab

%% setup ROS
setROS;

%% global variables needed
global pr                       % global physical parameters
global index                    % global index for the problem
global cfg                      % global configurations

%% common indexing
% Relates the quadID and obsID from MATLAB to the IDs set in ROS, should be
% consistent with the ID set in Mocap
mapping             =   [3,4,1,2,5,6];
mapping_obs         =   [1,2,3,4,5,6];

%% common MATLAB and configuration setup
%%% Get new FORCES solver from web
cfg.get_new_forces  =   0;

%%% drone type, set to choose physical parameters
cfg.quadType        =   1;      % 1-Bebop2; 2-Mambo

%%% Set the application, name of file
% cfg.Application = "basic";      % basic set-point maneuver collision avoidance
cfg.Application = "chance";     % chance constrained collision avoidance
% cfg.Application = "bound";     % bounding volume based approach
% cfg.Application = "full";      % full dynamics with uncertainties in stage variables


%%% mode configuration
cfg.ModeSim         =   0;  	% mode, 1-simulation; 0-experiment
cfg.UseRealdtSim    =   0;      % set UseRealdtSim to use real dt for control loop 
                                % even in simulation. (RT factor is always 1).
                                % NOT use this generally
cfg.dtSim           =   0.05; 	% dt in simulation, s
cfg.mindt           =   0.005;  % minimum dt (for experiment)

cfg.FullSim         =   0;      % if use MPC planned trajectory for simulation
cfg.SetPointGui     =   0;      % if get setpoint from gui

% load waypoints
% load RandWayPoints.mat
% wayPoints = RandWayPoints;
% load SwapWayPoints.mat
% wayPoints = SwapWayPoints;

%%% simulated noise for quadrotor
cfg.simQuadObserveNoise = 0;    % simulate observation noise
cfg.simQuadAccNoise     = 0;    % simulate acceleration noise, modeling distubances and model errors
cfg.simQuadPrNoise      = 0;    % simulate pitch and roll noise
cfg.simQuadControlNoise = 0;    % simulate control inputs noise

%%% Simulated obstacles
cfg.obsSim          =   0;   	% if simulated obstacles are used, default 0, 
                                % true in simulation mode
cfg.simObsObserveNoise  = 0; 
cfg.simObsAccNoise      = 0;    % if noise is added in simulation for obstacles, TO BE EXTENDED 

if cfg.ModeSim == 1
    cfg.obsSim = 1; 
end

%%% Define configuration for live plotting in GUI
cfg.Visual.ShowSubplot   = 1;       % only default value at startup, user changeable in GUI
cfg.Visual.ShowMPCPlan   = 1;       % if showing MPC plan
cfg.Visual.ShowObsCube   = 0;       % if showing obstacle cube
cfg.Visual.ShowObsEll    = 1;       % if showing obstacle safe ellipsoid
cfg.Visual.ShowObsEncEll = 1;       % if showing obstacle enclosed ellipsoid 
cfg.Visual.ShowQuadRealSphere = 0;  % if showing a sphere of real radius around the quadrotor
cfg.Visual.ShowQuadCollSphere = 1;  % if showing a sphere of collision radius around the quadrotor

%%% for considering quadrotor uncertainties
cfg.Visual.ShowQuadConfiEll   = 0;  % if showing the current confidence ellipsoid of the quadrotor
cfg.Visual.ShowQuadConfiEnlarge = 0;% if showing the confidence ellipsoid with enlarged collision radius
cfg.Visual.ShowQuadConfiEllPlan = 1;% if showing the predicted confidence ellipsoid of the quadrotor
cfg.Visual.ShowQuadConfiMPCEnlarge = 0; % if showing the confidence ellipsoid with enlarged collision radius
cfg.Visual.ShowQuadConfiEllPlanGap = 5; % Every # steps, confidence ellipsoid of the MPC plan is shown

%%% for considering obstacle uncertainties (TO DO)
cfg.Visual.ShowObsConfiEll    =  0; % if showing obs current confidence ellipsoid
cfg.Visual.ShowObsConfiEnlarge=  0; % if showing obs confidence ellipsoid with enlarged real size
cfg.Visual.ShowObsConfiEllPlan=  0; % if showing obs predicted confidence ellipsoid
cfg.Visual.ShowObsConfiEllPlanEnlarge = 0;  % if showing obs predicted confidence ellipsoid with real size
cfg.Visual.ShowObsConfiEllPlanGap = 4;      % Every # steps, confidence ellipsoid is shown

%%% Control form (bodycontrol allows control in body axes
cfg.UseBodyControl = 0;             % (DO NOT USE IN COMBINATION WITH NON-MANUAL CONTROLLERS))

%%% Set yaw as realtime parameter
cfg.UseRealYaw     = 1;

%%% log configurations
cfg.verbose = true;             % Display progress (iterations)
cfg.logsize = 30000;            % Based on sampling at around 0.02s for 10 minutes 
                                % (Increase for longer experiments)
cfg.DoLog = 1;

%%% Graphic communication
cfg.GraphicCom = 0;

% Motion Capture System type (choose one for experimental)
cfg.UseOpti         =   1;
cfg.UseVicon        =   0;


%% Import drone specific parameters
if cfg.quadType == 1
    Bebop2;
elseif cfg.quadType == 2
    error('[%s] Initialize: Implementation of Mambo is not implemented yet. \n',datestr(now,'HH:MM:SS'));
else
    error('[%s] Initialize: Incorrect Drone type is defined. \n',datestr(now,'HH:MM:SS'));
end


%% quadrotor yaw initialization
pr.realyawQuad = deg2rad(0*ones(6,1));     % real yaw of each quadrotor
pr.realyaw = 0;                            % common use for access


%% common characteristics of environment
% DCSC Lab
pr.ws.xdim = [-3.2 3.2];    	% m
pr.ws.ydim = [-1.4 1.5];     	% m
pr.ws.zdim = [ 0.1 2.6];      	% m
% pr.ws.zdim = [ 0.8 1.2];      	% m
pr.ws.dim  = [pr.ws.xdim(2) - pr.ws.xdim(1), pr.ws.ydim(2) - pr.ws.ydim(1), pr.ws.zdim(2) - pr.ws.zdim(1)];

% gravitational accleleration
pr.g = 9.81;                    % m/s^2

% Mahalanobis distance for plotting the confidence ellipsoid
pr.Mahalanobis = 3;             % changed in initialize_chance with the collision probability threshold

%% Application specific initializatoin script
% if cfg.Application == "basic"
%     initialize_basic;               % a basic version of multi-robot collision avoidance
% elseif cfg.Application == "chance"  % chance constrained multi-robot collision avoidance
%     initialize_chance; 
% elseif cfg.Application == "bound"   % bounding volume based approach
%     initialize_bound; 
% elseif cfg.Application == "full"    % full model with uncertainty optimization
%     initialize_full; 
% else
%     error('[%s] Initialize: Application not set correctly. \n',datestr(now,'HH:MM:SS'));
% end

%% store all initialise variables in share
share.cfg   = cfg;
share.index = index;
share.pr    = pr;
