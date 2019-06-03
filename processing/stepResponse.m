%STEPRESPONSE Resturns estimated bandwith for phi, theta, vz
%   BW[Hz] = 0.35 / rise time [s]
%   rise time = 10-90% rise time van command
clc;
[files,path]=uigetfile('*.mat','Select the INPUT DATA FILES','MultiSelect','on');

bw_theta_avg = 0;
bw_phi_avg = 0;
bw_vz_avg = 0;

bw_phis = zeros(1,length(files));
bw_thetas = zeros(1,length(files));
bw_vzs = zeros(1,length(files));

for t_i=1:length(files)
    fileName = files{t_i};
    dataName = fileName; %omdat in ingelade data soms filename staat
    fprintf('[%s Loading data %s%s \n', datestr(now,'HH:MM:SS'), path, fileName);     
    load(fullfile(path,fileName));

    Time = zeros(length(phi),1);
    for j = 1:length(phi)-1 
       Time(j+1) = Time(j)+log_Time(1,j);
    end
    
    measured = [phi,theta,vz];
    measured = measured(:,:) - measured(1,:); %0 stellen op begin, soms is er wat mis met de calibratie (tov hover) waardoor nooit 90% bereikt wordt. Dit voorkomt dat.
    commanded = [phi_c,theta_c,vz_c];
    bw_phi      = 0;
    bw_theta    = 0;
    bw_vz       = 0;
    rise_phi    = 0;
    rise_theta  = 0;
    rise_vz     = 0;

    for i = 1:3
        command = commanded(:,i);
        measure = measured(:,i);

        A = command(find(command > .01, 1));    %Final commanded step height  
        i_commandstart = find(command > .01, 1);
        command = command(i_commandstart:length(command));
        measure = measure(i_commandstart:length(measure));
        time_loop = Time(i_commandstart:length(Time));

        if isempty(A) %no input on this axis
            bw = 0;
            risetime = 0;
        else         
            %10% and 90% values
            lowend = .1*A;
            highend = .9*A;

            %time index where response reaches those values
            i_start = find(measure > lowend, 1);
            i_end = find(measure > highend, 1);

            risetime = time_loop(i_end) - time_loop(i_start)
            bw = .35 / risetime;
        end

        if (i == 1)
            bw_phi = bw;
            rise_phi = risetime;
            bw_phis(t_i) = bw;
        elseif (i == 2)
            bw_theta = bw;
            rise_theta = risetime;
            bw_thetas(t_i) = bw;
        elseif (i ==3)
            bw_vz = bw;
            rise_vz = risetime;
            bw_vzs(t_i) = bw;
        end                
    end
end
