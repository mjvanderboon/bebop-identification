%% Te checken
% Klopt rotatiematrix R van welke naar welk
clear all; clc;
syms theta phi psi kDx kDy kDz a_x a_y a_z vx vy vz T_m g m

T = [0;0;T_m];                          % Thrust vector in body frame
D = [kDx 0 0; 0 kDy 0 ; 0 0 kDz];    %body frame drag coeff matrix
g = [0;0;g];                           %world frame gravity
v = [vx; vy; vz];                    % world frame velocity

Rpsi = [cos(psi), -sin(psi), 0; sin(psi) cos(psi) 0; 0 0 1];
Rphi = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
Rtheta = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
R = Rpsi*Rphi*Rtheta; % Zou kunnen dat dit in het verslag niet klopt

%% Governing equation of motion
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Assumptions:
% aFdrag = Rpsi * D * vb (and not R * D * vb)
% vb = Rpsi\v (and not vb = R \ v)

a = (- m*g + R * T - R * (D*(Rpsi\v)))/m;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Solving for xdd and ydd
% Find T, possible because vzd is from a known input
eqn3 = a_z == a(3);
T_m = solve(eqn3, T_m);

% Solve xdd and ydd with found T_m
eqn1 = m*a_x == simplify(subs(a(1))*m);
eqn2 = m*a_y == simplify(subs(a(2))*m);

lat_T_m = latex(T_m);
lat_eq1 = latex(eqn1);
lat_eq2 = latex(eqn2);

figure() 
title('Governing equations of motion for a_x, a_y, with assumpt velocities body frame ~ velocities world frame');
annotation(gcf,'textbox',[0,.6,1,.3],'string',strcat('$$ T = ',lat_T_m,'$$'),'interpreter','latex','FontSize', 20);
annotation(gcf,'textbox',[0,.3,1,.3],'string',strcat('$$ ',lat_eq1,'$$'),'interpreter','latex','FontSize',16);
annotation(gcf,'textbox',[0,0,1,.3],'string',strcat('$$ ',lat_eq2,'$$'),'interpreter','latex','FontSize',16);
 
 
