function [deadsteps] = deadTimeEstimation(channel,dataName,path)
%% deadTimeEstimation uses delayest to estimate dead time of channel
% channel: 1 (phi), 2 (theta), 3 (vz), 4 (vpsi)
% TODO: vpsi not functional

%% Settings
DefaultDatapath = '..\processing\data\rbs'; 

%% Loading data
% If no input, open ui to select one
DefaultDatapath = strcat('..', filesep,'processing', filesep, 'data', filesep,'rbs');

psi = 0;
if (isempty(dataName))
    [dataName,path]=uigetfile('*.mat','Select the data File',DefaultDatapath);
    load(fullfile(path,dataName));
    fprintf('[%s] Loading data %s \n',datestr(now,'HH:MM:SS'),dataName);
else
    load(fullfile(path,dataName));    
end

dummy = zeros(length(phi),1);
vpsi_c = dummy;
vpsi = dummy; %because there is no data

%% Create Data object
sampleT = mean(log_Time(1,:));
InputAttitude  = [phi_c, theta_c, vz_c, vpsi_c];   
OutputAttitude = [phi, theta, vz, vpsi];

data = iddata(OutputAttitude, InputAttitude, sampleT, 'Name','Measured data');
data.InputName = {'Commanded Roll','Commanded Pitch','Commanded vz', 'Commanded yaw rate'};
data.InputUnit =  {'(rad)','(rad)','(m/s)','(rad/s)'};
data.OutputName = {'Measured Roll','Measured Pitch','Velocity in z','Yaw rate'};
data.OutputUnit = {'(rad)','(rad)','(m/s)','rad/s'};
data.Tstart = 0;
data.TimeUnit = 's';

deadsteps = delayest(data(:,channel,channel));
end