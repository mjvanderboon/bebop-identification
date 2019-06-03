function [data] = createFullDataObject(dataName,dataPath)
% Create full iddata object from data file
% Input = [phi_c, theta_c, vz_c,psi_c]
% Ouput = [x;y;z;vx;vy;vz;phi;theta;psi]

%% Loading data
% If no input, open ui to select one
DefaultDataPath = strcat('..', filesep,'processing', filesep, 'data', filesep,'rbs');

psi = 0;
if (isempty(dataName))
    [dataName,dataPath]=uigetfile('*.mat','Select the data File',DefaultDataPath);
    load(fullfile(dataPath,dataName));    
else    
    load(fullfile(dataPath,dataName));    
end

%because there is no data. If wanted, have to use kalman filter on data
dummy = zeros(length(phi),1);
vpsi_c = dummy;
vpsi = dummy; 

%% Create Data object
sampleT = mean(log_Time(1,:));
InputAttitude  = [phi_c, theta_c, vz_c,vpsi_c];   
OutputAttitude = [x,y,z,vx,vy,vz,phi,theta,psi];

data = iddata(OutputAttitude, InputAttitude, sampleT, 'Name','Measured data');
data.InputName = {'Commanded Roll','Commanded Pitch','Commanded vz', 'Commanded Yaw rate'};
data.InputUnit =  {'(rad)','(rad)','(m/s)', '(rad/s)'};                                            
data.OutputName = {'X', 'Y', 'Z', 'Velocity in x','Velocity in y','Velocity in z', 'Roll', 'Pitch', 'Yaw'};
data.OutputUnit = {'(m)','(m)', '(m)',' (m/s)',' (m/s)',' (m/s)', '(rad)', '(rad)', '(rad)'};                                            
data.Tstart = 0;
data.TimeUnit = 's';
data.ExperimentName = dataName;

end
