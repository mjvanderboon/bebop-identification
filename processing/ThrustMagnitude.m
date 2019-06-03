%% Plot magnitude of thrust
clear all;

[fileName,path]=uigetfile('*.mat','Select the INPUT DATA FILE','MultiSelect','off');
dataName = fileName; %omdat in ingelade data soms filename staat
fprintf('[%s Loading data %s%s \n', datestr(now,'HH:MM:SS'), path, fileName);
load(fullfile(path, fileName));  

Time_real = zeros(length(phi),1);
for j = 1:length(phi)-1 
    Time_real(j+1) = Time_real(j)+log_Time(1,j);
end

vzd = diff(vz)./diff(Time_real);
m = .5;
g = 9.81;
T = m*(g+vzd)./(cos(phi(1:length(phi)-1)).*cos(theta(1:length(phi)-1)));
T = lowpass(T,.1);

set(0, 'defaultTextInterpreter', 'latex', 'defaultAxesTickLabelInterpreter', 'latex', ...
    'defaultLegendInterpreter', 'latex', 'defaultAxesFontSize', 16)
inputColor = [1,0,0];
simColor = [0,0,1];
valColor = [.7, .7, .7];

inputStyle = '--';
simStyle = '-';
valStyle = '-';

figure();
plot(Time_real(1:length(phi)-1),T);
title('$T = \frac{m(g+\dot{v_z})}{cos \phi cos \theta}$');
xlabel('t (s)');
ylabel('T (N)');