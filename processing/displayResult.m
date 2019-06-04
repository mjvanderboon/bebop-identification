%% displayResult
% displays recorded data in single plot

set(0, 'defaultTextInterpreter', 'latex', 'defaultAxesTickLabelInterpreter', 'latex', ...
    'defaultLegendInterpreter', 'latex', 'defaultAxesFontSize', 14)

clear all;
psi = 0; %nodig omdat psi ook een functie in matlab is
[fileName,path]=uigetfile('*.mat','Select the INPUT DATA FILE','MultiSelect','off');
dataName = fileName; %omdat in ingelade data soms filename staat
fprintf('[%s Loading data %s%s \n', datestr(now,'HH:MM:SS'), path, fileName);
load(fullfile(path, fileName));    

u_x = zeros(length(phi),1) ;
u_y = zeros(length(phi),1) ;

for i = 1:length(phi)
u_x(i) = cos(psi(i))*tan(theta(i))/cos(phi(i))+sin(psi(i))*tan(phi(i)) ;
u_y(i) = -1*(sin(psi(i))*tan(theta(i))/cos(phi(i))-cos(psi(i))*tan(phi(i))); %de min in deze regel is misschien niet meer nodig omdat roll input nu goed is
end

sampleT = mean(log_Time(1,:));
Time = zeros(length(phi),1);
for j = 1:length(phi)-1 
   Time(j+1) = Time(j)+log_Time(1,j);
end

vel_0 = vx;

%% Plotting data

%plot(Time,vx,Time,vel_0,'r');
%hold off;

figure('units','normalized','outerposition',[0 0 1 1],'name',dataName)
subplot(3,3,1);
title('Pitch');
hold on
plot(Time,theta_c,Time,theta)
legend('Commanded pitch','Measured pitch') ; 
xlabel('Time (s)')  
ylabel('$\theta$ (rad)');
ylim(deg2rad(12)*[-1, 1]);
hold off

subplot(3,3,2);
title('Roll');
hold on
plot(Time,phi_c,Time,phi)
legend('Commanded roll','Measured roll') ; 
xlabel('Time (s)')  
ylabel('$\phi$ (rad)');
ylim(deg2rad(12)*[-1, 1]);
hold off

subplot(3,3,3);
title('Yaw');
hold on
plot(Time,psi);
legend('Measured yaw') ; 
xlabel('Time (s)')  
ylabel('$\psi$ (rad)');
ylim(deg2rad(12)*[-1, 1]);
hold off

subplot(3,3,4);
title('Velocity x');
hold on
plot(Time,vx)
legend('Measured velocity x') ; 
xlabel('Time (s)')  
ylabel('$v_x$ (m/s)');
ylim([-1, 1]);
hold off

subplot(3,3,5);
title('Velocity y');
hold on
plot(Time,vy)
legend('Measured velocity y') ; 
xlabel('Time (s)')  
ylabel('$v_y$ (m/s)');
ylim([-1, 1]);
hold off

subplot(3,3,6);
title('Velocity z');
hold on
plot(Time,vz_c,Time,vz)
legend('Commanded velocity z','Measured velocity z') ; 
xlabel('Time (s)')  
ylabel('$v_z$ (m/s)');
ylim([-1,1]);
hold off

subplot(3,3,7);
title('Position x');
hold on
plot(Time,x);
legend('Measured position x');
xlabel('Time (s)')  
ylabel('$x$ [m]');
ylim([-2,2]);
hold off

subplot(3,3,8);
title('Position y');
hold on
plot(Time,y);
legend('Measured position y');
xlabel('Time (s)')  
ylabel('$y$ (m)');
ylim([-2,2]);
hold off

subplot(3,3,9);
title('Position z');
hold on
plot(Time,z);
legend('Measured position z');
xlabel('Time (s)')  
ylabel('$z$ (m)');
ylim([-.5,.5]);
hold off