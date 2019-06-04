%% ODE Input response model
function [dx,y] = GO_InputResponse(t, x, u, Aphi, Atheta, Avz, Avpsi, Bphi, Btheta, Bvz, Bvpsi, Cphi, Ctheta, Cvz, Cvpsi, aux)
%% State space general order model of input response
% x = [xphi1,xphi2,xphi3...,xtheta1,xtheta2,xtheta3...,xvz1,xvz2,xvz3..., xvpsi1,...]'
% u = [phi_c; theta_c; vz_c,vpsi_c]
% y = [phi; theta; vz,vpsi]
% individual order of input channels does not have to be the same

% Output matrices have to be transposed because of system identification
% toolbox limitations
Cphi    = Cphi';
Ctheta  = Ctheta';
Cvz     = Cvz';
Cvpsi   = Cvpsi';

%% Get states from state vector
ib_phi  = 1;
ie_phi  = ib_phi + size(Aphi,1)-1;

ib_theta = ie_phi + 1;
ie_theta = ib_theta + size(Atheta,1)-1;

ib_vz    = ie_theta + 1;
ie_vz    = ib_vz + size(Avz,1)-1;

ib_vpsi  = ie_vz + 1;
ie_vpsi  = ib_vpsi + size(Avpsi,1)-1;

xphi    = x(ib_phi:ie_phi);
xtheta  = x(ib_theta:ie_theta);
xvz     = x(ib_vz:ie_vz);
xvpsi   = x(ib_vpsi:ie_vpsi);

phi_c   = u(1);
theta_c = u(2);
vz_c    = u(3);
vpsi_c  = u(4);

%% Setting states

dx = [
    Aphi*xphi + Bphi*phi_c; ...
    Atheta*xtheta + Btheta*theta_c; ...
    Avz*xvz + Bvz*vz_c; ...   
    Avpsi*xvpsi + Bvpsi*vpsi_c; ...
];

y = [Cphi*xphi; ...   
    Ctheta*xtheta;  ...
    Cvz* xvz; ...
    Cvpsi*xvpsi;
];
end

