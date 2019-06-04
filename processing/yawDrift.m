%% yawDrift
% Plots yaw data for multiple experiments.
% The plot is used to analysed the randomness of the yaw drift during
% experiments. WARNING: most data in /processing is already deyawed, so the
% recorded yaw for those data sets is 0. Data in /logs contains original
% data.
clear all; clc;
[t_fileName,t_path]=uigetfile('*.mat','Select the INPUT DATA FILE(s)','MultiSelect','on');
yaw = zeros(1);
psi = 0;
set(0, 'defaultTextInterpreter', 'latex', 'defaultAxesTickLabelInterpreter', 'latex', ...
    'defaultLegendInterpreter', 'latex', 'defaultAxesFontSize', 14)
hold off;

tsample = 1/100;

subplot(2,1,1);
title('Yaw during hover (zero input)');
hold on;
xlabel('Time (s)')  
ylabel('$\psi$ [deg]');
ylim([-10,10]);
for t_i=1:length(t_fileName)    
    load(fullfile(t_path,t_fileName{t_i}));
    psi = log_QuadStatesReal(6,:)-log_QuadStatesReal(6,1); %zero-ing yaw
    fprintf('[%s Loading data %s%s \n', datestr(now,'HH:MM:SS'), t_path, t_fileName{t_i});  
    yaw(t_i,1:length(psi)) = rad2deg(psi'); %yaw matrix for all data
    t = linspace(0,length(yaw)*tsample,length(yaw));
    plot(t(1:length(psi)-1), yaw(t_i,1:length(psi)-1));
end
hold off

subplot(2,1,2);
title('Position z');
hold on
for t_i=1:length(t_fileName)    
    load(fullfile(t_path,t_fileName{t_i}));           
    t = linspace(0,length(z)*tsample,length(z));
    plot(t, z);
end
xlabel('Time (s)')  
ylabel('$z$ [m]');
ylim([0, 1]);
hold off