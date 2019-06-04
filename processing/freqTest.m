%% freqTest
% Script used for analysing step response of data with input send at
% different frequencies. Output is a figure overlaying the recorded step
% response for all data files.
clear all; clc;
[t_fileName,t_path]=uigetfile('*.mat','Select the INPUT DATA FILE(s)','MultiSelect','on');
yaw = zeros(1);
psi = 0;
set(0, 'defaultTextInterpreter', 'latex', 'defaultAxesTickLabelInterpreter', 'latex', ...
    'defaultLegendInterpreter', 'latex', 'defaultAxesFontSize', 14)
hold off;

figure();
title('Step response for inputs sent at different frequency');
hold on;
xlabel('Time (s)')  
ylabel('$\theta$ [rad]');
ylim([-1 8]);
xlim([0, .5]);
legend();
for t_i=1:length(t_fileName)    
    load(fullfile(t_path,t_fileName{t_i}));
    command = theta_c;
    measured = theta;
    
    %setting up time vector
    sampleT = mean(log_Time(1,:));
    Time = zeros(length(phi),1);
    for j = 1:length(phi)-1 
        Time(j+1) = Time(j)+log_Time(1,j);
    end
    
    %Finding step input and shifting data to it
    start = find(command>.01,1);       
    iend = start + floor(.4 / sampleT);
    Time = Time(start:iend) - Time(start);
    command = command(start:iend);
    measured = measured(start:iend);   
    
    %Generating color for line (linear map from 0Hz to 120Hz
    first = [0,1,0];
    second = [1,0,0];
    freqMap = 70;
    color = first - (first - second)/max((freqMap*sampleT),1);
    
    command = rad2deg(command);
    measured = rad2deg(measured);
    if (t_i == 1)
        plot(Time,command,'b');  
    end
    plot(Time,measured,'color',color);
    
    fprintf('[%s Loading data %s%s \n', datestr(now,'HH:MM:SS'), t_path, t_fileName{t_i});        
end
hold off