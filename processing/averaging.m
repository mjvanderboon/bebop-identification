[t_fileName,t_path]=uigetfile('*.mat','Select the INPUT DATA FILE(s)','MultiSelect','on');
set(0, 'defaultTextInterpreter', 'latex', 'defaultAxesTickLabelInterpreter', 'latex', ...
    'defaultLegendInterpreter', 'latex', 'defaultAxesFontSize', 14)
hold off;


theta_avg = 0;

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
    iend = start +4*16;
    Time = Time(start:iend) - Time(start);
    command = command(start:iend);
    measured = measured(start:iend);   
    theta = theta_avg;
    
    
    theta_avg = measured + theta_avg;    
    fprintf('[%s Loading data %s%s \n', datestr(now,'HH:MM:SS'), t_path, t_fileName{t_i});        
end



