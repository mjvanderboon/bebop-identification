%% Dynamics describing full simulation model
function [dx,y] = GO_FullModel(t, x, u, Aphi, Atheta, Avz,Avpsi, ...
    Bphi, Btheta, Bvz,Bvpsi,...
    Cphi, Ctheta, Cvz,Cvpsi,...
    kDx, kDy, kDz, ...
    aux)
%% Full general order model of drone dynamics
% x = [
%     xpos,ypos,zpos,
%     vx,vy,
%     Xvz1,Xvz2....     
%     Xphi1,Xphi2,...
%     Xtheta1,Xtheta2,...
%     Xvpsi1,Xvpsi2,....
%     psi
% ]'
% u = [phi_c, theta_c, vz_c, vpsi_c]'
% y = [xpos;ypos;zpos;vx;vy;vz;phi;theta;psi]
g = 9.81;

% System identification toolbox requires input matrices to be square or row
% Therefore C matrices are transposed.
Cphi    = Cphi';
Ctheta  = Ctheta';
Cvz     = Cvz';
Cvpsi   = Cvpsi';

%% Get states from state vector
% 5 states are 'reserverd' for x,y,z,vx,vy
% 6:rest are for non-physical parameters
% (last is psi)
% order can differ between vz, phi, theta, psi

% selecting indices of non-physical parameters
ib_vz = 6;
ie_vz = ib_vz + size(Avz, 1) - 1; %begin + order of model for vz

ib_phi  = ie_vz + 1; % one index past where vz states ended
ie_phi  = ib_phi + size(Aphi,1)-1;    

ib_theta = ie_phi + 1;
ie_theta = ib_theta + size(Atheta,1)-1;

ib_vpsi = ie_theta + 1;
ie_vpsi = ib_vpsi + size(Avpsi,1)-1;

xpos    = x(1);
ypos    = x(2);
zpos    = x(3);

vx      = x(4);
vy      = x(5);

xvz     = x(ib_vz:ie_vz);
xphi    = x(ib_phi:ie_phi);
xtheta  = x(ib_theta:ie_theta);
xvpsi   = x(ib_vpsi:ie_vpsi);

psi     = x(ie_vpsi + 1); %very last state is psi

%% Input commands
phi_c   = u(1);
theta_c = u(2);
vz_c    = u(3);
vpsi_c  = u(4);

%% Output from non-physical parameters
phi     = Cphi*xphi;
theta   = Ctheta*xtheta;
vz      = Cvz*xvz;
vpsi    = Cvpsi*xvpsi;

%% Governing equations of motion
% Below formulaes are found from Fma.m
% az = d(vz)/dt = d(Cvz*xvz)/dt = Cvz*d(xvz)/dt =Cvz*(Avz*xvz+Bvz*vzc)
az = Cvz*(Avz*xvz+Bvz*vz_c);
ax = kDy*sin(psi)*(vy*cos(psi) - vx*sin(psi)) - kDx*cos(psi)*(vx*cos(psi) + vy*sin(psi)) + ((cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi))*(az + g + kDz*vz))/(cos(phi)*cos(theta));
ay = ((sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi))*(az + g + kDz*vz))/(cos(phi)*cos(theta)) - kDx*sin(psi)*(vx*cos(psi) + vy*sin(psi)) - kDy*cos(psi)*(vy*cos(psi) - vx*sin(psi));

%% Setting states
dx = [...
    vx; vy; vz;...
    ax; ay;...
    Avz*xvz       + Bvz*vz_c;...
    Aphi*xphi     + Bphi*phi_c;...
    Atheta*xtheta + Btheta*theta_c;...
    Avpsi*xvpsi    + Bvpsi*vpsi_c;    ...
    vpsi; ...
];

y = [...
    xpos;ypos;zpos;...
    vx;vy;vz;...
    phi;theta;psi;...        
];
end

