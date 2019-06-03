%% Batch process all log files
clear all;
[t_fileName,t_path]=uigetfile('*.mat','Select the INPUT DATA FILE(s)','MultiSelect','on');

%% Input
% log_QuadInputs             
%   [pitch_c, roll_c, gaz_c]
%   scaled -1,1   
% log_QuadStatesReal
%   [x;y;z;pitch;-roll;yaw]
%   (m,m,m,rad,rad,rad), gemeten door mocap
% log_QuadStatesEstimated
%   [x;y;z;xdot;ydot;zdot;pitch;-roll;yaw;pitch_dot;roll_dot:yaw_dot]
%   (m,m,m,m/s,m/s,m/s,rad,rad,rad,rad/s,rad/s,rad/s), uit full state
%   estimator. Geeft gekke waarden voor hoeken.
% log_QuadStatesEstimatedCov     %pos cov vel cov euler cov
% log_Time              

%% Output
% log_QuadInputs             
%   [pitch_c, roll_c, vz_c]
%   scaled -12,12  
% log_QuadStatesReal 
%   [x;y;z;pitch;roll;yaw]
%   (m,m,m,rad,rad,rad), gemeten door mocap
% log_QuadStatesEstimated 
%   [x;y;z;pitch;roll;yaw;x_dot;y_dot;z_dot;pitch_dot;roll_dot;yaw_dot]
%   (m,m,m,rad,rad,rad,m/s,m/s,m/s,rad/s,rad/s,rad/s),
% log_QuadStatesEstimatedCov    
% log_Time 
% x,y,z,theta,phi,psi,vx,vy,vz,vtheta,vphi,vpsi,theta_c,phi_c,vz_c
% Time_real real time vector (from measured sample times)

%% Options
batch_deyaw         = 1;    %variables prefixed with t_ are not saved
batch_flipRoll_c    = 0;    %Bij data voor 3 mei moet de rollinput omgedraaid worden
batch_cut           = 1;    %Cut off landen en starten?

%% Batch
fprintf('[%s Batching log files with deyaw: %s, flipRoll_c: %s cut: %s \n\n', datestr(now,'HH:MM:SS'), num2str(batch_deyaw), num2str(batch_flipRoll_c,batch_cut));

for t_i=1:length(t_fileName)
    %Load data
    load(fullfile(t_path,t_fileName{t_i}));
    fprintf('[%s Loading data %s%s \n', datestr(now,'HH:MM:SS'), t_path, t_fileName{t_i});
    
    %Bij data voor 3 mei is de roll input niet volgens conventie opgenomen
    if (batch_flipRoll_c == 1)
        log_QuadInputs(2,:) = -log_QuadInputs(2,:);        
    end
    
    %Shuffle
    %SDK to world
    log_QuadInputs(1:2,:)   = log_QuadInputs(1:2,:) * deg2rad(12);
    log_QuadInputs(3,:)     = log_QuadInputs(3,:) * 1;
       
    % estimates vector naar pos, euler, vel, vel euler zetten
    t_dummy = log_QuadStatesEstimated(4:6,:);
    log_QuadStatesEstimated(4:6,:) = log_QuadStatesEstimated(7:9,:);
    log_QuadStatesEstimated(7:9,:) = t_dummy;
    
    %lowpass filter    	
    log_QuadStatesReal = lowpass(log_QuadStatesReal', .1)'; %' omdat lowpass kolommen filtert
    log_QuadStatesEstimated = lowpass(log_QuadStatesEstimated', .1)';     
    
    %Cut opstijgen en landen
    if (batch_cut ==1 )
        t_startindex = find(any(log_QuadInputs),1);
        t_endindex = find(any(log_QuadInputs),1,'last');
        prepend = 20;
                
        log_QuadInputs          = [zeros(3,prepend).*log_QuadInputs(:,t_startindex-1), log_QuadInputs(:,t_startindex:t_endindex)];
        log_QuadStatesReal          = [[ones(3,prepend);zeros(3,prepend)].*log_QuadStatesReal(:,t_startindex-1), log_QuadStatesReal(:,t_startindex:t_endindex)];
        log_QuadStatesEstimated          = [ones(12,prepend).*log_QuadStatesEstimated(:,t_startindex-1), log_QuadStatesEstimated(:,t_startindex:t_endindex)];
        log_QuadStatesEstimatedCov          = [zeros(27,prepend).*log_QuadStatesEstimatedCov(:,t_startindex-1), log_QuadStatesEstimatedCov(:,t_startindex:t_endindex)];
            
%         log_QuadInputs          = [log_QuadInputs(:,t_startindex), log_QuadInputs(:,t_startindex:t_endindex)];
%         log_QuadStatesReal      = log_QuadStatesReal(:,t_startindex:t_endindex);
%         log_QuadStatesEstimated = log_QuadStatesEstimated(:,t_startindex:t_endindex);
%         log_QuadStatesEstimatedCov=log_QuadStatesEstimatedCov(:,t_startindex:t_endindex);
%         log_Time                = log_Time(:,t_startindex:t_endindex);
    end

    %0 stellen op begin
    log_QuadStatesReal(1:3,:) = log_QuadStatesReal(1:3,:) - log_QuadStatesReal(1:3,1);
    log_QuadStatesEstimated(1:3,:) = log_QuadStatesEstimated(1:3,:) - log_QuadStatesEstimated(1:3,1);
           
    %Variabelen klaarzetten voor system identification
    x       = log_QuadStatesReal(1,:)';
    y       = log_QuadStatesReal(2,:)';
    z       = log_QuadStatesReal(3,:)';
    theta   = log_QuadStatesReal(4,:)';
    phi     = log_QuadStatesReal(5,:)';
    psi     = log_QuadStatesReal(6,:)';
    
    vx      = log_QuadStatesEstimated(7,:)';
    vy      = log_QuadStatesEstimated(8,:)'; 
    vz      = log_QuadStatesEstimated(9,:)';
    vtheta  = log_QuadStatesEstimated(10,:)';
    vphi    = log_QuadStatesEstimated(11,:)';
    vpsi    = log_QuadStatesEstimated(12,:)';
    
    theta_c = log_QuadInputs(1,:)';
    phi_c   = log_QuadInputs(2,:)';
    vz_c    = log_QuadInputs(3,:)';    
    
    %Deyaw
    t_vel = zeros(3,1);
    if (batch_deyaw == 1)
        for t_j = 1:length(psi)
            t_R_deyaw = rotm3d(-psi(t_j),3);    
            t_pos     = t_R_deyaw*[x(t_j);y(t_j);z(t_j)];
            t_vel     = t_R_deyaw*[vx(t_j);vy(t_j);vz(t_j)];
            t_angles  = t_R_deyaw*[theta(t_j);phi(t_j);psi(t_j)];
    
            x(t_j) = t_pos(1);
            y(t_j) = t_pos(2);
            z(t_j) = t_pos(3);
    
            vx(t_j)= t_vel(1);
            vy(t_j)= t_vel(2);
            vz(t_j)= t_vel(3);
        
            %Roll en pitch wordt in body frame gemeten dus hoeft niet
            %Check dit nog een keer, ook, worden
            %teruggedraaid te worden.
            %theta(t_j)  = t_angles(1);
            %phi(t_j)    = t_angles(2);
            psi(t_j)    = 0;
        end 
    end
    
    %Variabelen klaarzetten voor system identification
    log_QuadStatesReal(1,:)     = x';
    log_QuadStatesReal(2,:)     = y';
    log_QuadStatesReal(3,:)     = z';
    log_QuadStatesReal(4,:)     = theta';
    log_QuadStatesReal(5,:)     = phi';
    log_QuadStatesReal(6,:)     = psi';
    
    log_QuadStatesEstimated(7,:)= vx';
    log_QuadStatesEstimated(8,:)= vy';
    log_QuadStatesEstimated(9,:)= vz';
    log_QuadStatesEstimated(10,:)=vtheta';
    log_QuadStatesEstimated(11,:)=vpsi';
    log_QuadStatesEstimated(12,:)=vphi';
    
    log_QuadInputs(1,:) = theta_c';
    log_QuadInputs(2,:) = phi_c';
    log_QuadInputs(3,:) = vz_c';
    
    Time_real = zeros(length(phi),1);
    for j = 1:length(phi)-1 
        Time_real(j+1) = Time_real(j)+log_Time(1,j);
    end
    
    %Save data except variables prefixed with t_
    [t_filepath,t_name,t_ext] = fileparts(fullfile(t_path, t_fileName{t_i}));
    t_newFile = strcat(pwd,filesep,'data',filesep,'rbs',filesep,t_name,'_processed','.mat'); %filesep is platform specific separator (linux '/', windows '\')
    save(t_newFile,'-regexp','^(?!t_.*$).');
    fprintf('[%s] Saved processed log data to %s \n',datestr(now,'HH:MM:SS'),t_newFile)
end