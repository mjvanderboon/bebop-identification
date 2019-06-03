%% Initialization of Bebop specific parameters

pr.qd.m     = 0.5;                  % quadrotor mass, kg
pr.qd.radius_real = 0.2;            % quadrotor real physical radius
pr.qd.radius_coll = 0.3;           % quadrotor collision radius
pr.D.q = 0.28;                      % Drag coefficient, unitless

% attitude dynamics model parameters
% load('data/Bebop2_identifiedss.mat');
% pr.att.Ap = sys_pitch.A;
% pr.att.Bp = sys_pitch.B;
% pr.att.Cp = sys_pitch.C;
% pr.att.Dp = sys_pitch.D;
% 
% pr.att.Ar = sys_roll.A;
% pr.att.Br = sys_roll.B;
% pr.att.Cr = sys_roll.C;
% pr.att.Dr = sys_roll.D;
% 
% pr.att.Ag = sys_gazacc.A;
% pr.att.Bg = sys_gazacc.B;
% pr.att.Cg = sys_gazacc.C;
% pr.att.Dg = sys_gazacc.D;
% 
% clear ss2pem_gaz ss2pem_pitch ss2pem_roll sys_gaz sys_gazacc sys_pitch sys_roll
% 
% % first order dynamics
% pr.att.Ap1 = -1.741;
% pr.att.Bp1 = 3.46;
% pr.att.Cp1 = 0.8135;
% pr.att.Ar1 = -1.841;
% pr.att.Br1 = 3.188;
% pr.att.Cr1 = 0.8379;
% pr.att.Ag1 = -0.5309;
% pr.att.Bg1 = 0.3533;
% pr.att.Cg1 = 2.357;

% system limit
%% Application specific initializatoin script
if cfg.Application == "basic" || cfg.Application == "chance" || cfg.Application == "bound" || cfg.Application == "full"
    pr.input.maxAngleRad    = deg2rad(12);     % rad (maximum UAV pitch/roll angle)
    pr.input.maxVelZ        = 1;               % maximum vertical velocity, m/s
    pr.input.maxRotSpeed    = deg2rad(120);    % maximum rotation speed, rad/s
    pr.state.maxVel         = 4;               % maximum speed, m/s
elseif cfg.Application == "flycam"
    pr.input.maxAngleRad    = deg2rad(30);     % rad (maximum UAV pitch/roll angle)
    pr.input.maxVelZ        = 1;               % maximum vertical velocity, m/s
    pr.input.maxRotSpeed    = deg2rad(300);    % maximum rotation speed, rad/s
    pr.state.maxVel         = 4;               % maximum speed, m/s
    pr.input.maxGimbalRate  = 2;               % Maximum gimbal rotation speed rad/s
    pr.input.maxGimbalPitch = [-deg2rad(29);   % Maximum gimbal pitch rad
                               deg2rad(14)];
    pr.input.maxGimbalYaw   = deg2rad(25);     % Maximum gimbal yaw rad
                              
else
    error('[%s] Initialize: Drone parameters are not set correctly. \n',datestr(now,'HH:MM:SS'));
end