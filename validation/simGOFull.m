function [valData,simData, fit_score] = simGOFull(model, data)
%SIMGOFULL simulate full general order identified model over given dataset
%   model : model to simulate, if empty, UI opens
%   data  : dataset to simulate model over, if empty UI opens
%           data is unshifted. If model has timedelay in userdata it gets added for simulation


%% Settings
kDx = .25;
kDy = .33;
kDz = 0;
modelFile = 'GO_FullModel';
simBegin = 20; % time step from where to start simulation from state estimation

%% Init
DefaultDataPath = 'C:\Users\Matthijs\Desktop\BEP\BEP2019\processing\data\rbs';
DefaultModelPath = strcat('..', filesep,'systemidentification', filesep, 'Results');

if (isempty(model))
	[modelName,modelPath]=uigetfile('*.mat','Select the model file',DefaultModelPath);
    load(fullfile(modelPath,modelName));
    fprintf('[%s] Loading model %s \n',datestr(now,'HH:MM:SS'),modelName); 
end
if (isempty(data))
    [dataName,dataPath]=uigetfile('*.mat','Select the data File',DefaultDataPath);
    fprintf('[%s] Loading data %s \n',datestr(now,'HH:MM:SS'),dataName);
    [data] = createFullDataObject(dataName, dataPath);
end

modelOrder = model.Order.nx;
%modelOrder = 4; %for plotting FOPTD
order = [9 4 (modelOrder+6)]; %ny, nu, nx(6 more from input response model)

%% Creating full model object
aux = {};

Aphi    = model.Parameters(1).Value;
Atheta  = model.Parameters(2).Value;
Avz     = model.Parameters(3).Value;
Avpsi   = model.Parameters(4).Value;

Bphi    = model.Parameters(5).Value;
Btheta  = model.Parameters(6).Value;
Bvz     = model.Parameters(7).Value;
Bvpsi   = model.Parameters(8).Value;

Cphi    = model.Parameters(9).Value;
Ctheta  = model.Parameters(10).Value;
Cvz     = model.Parameters(11).Value;
Cvpsi   = model.Parameters(12).Value;

%% Parameters nikhal found
% Aphi =[-2.789 -4.978; 9.302 -13.72];
% Atheta=[-4.301 -2.877; 10.92 -10.37];
% Avz=[-4.875, 1.848; -6.062, -2.203];
%  
% Bphi=[-5.41; -18.04];
% Btheta=[-0.6893; -16.32];
% Bvz=[0.6029; 3.681];
%  
% Cphi=[1.996; 0.4657];
% Ctheta=[1.763; 4.586e-3];
% Cvz=[4.029; -0.7253];

%% Parameters FOPTD
% Aphi = -1/0.1598;
% Atheta = -1/0.1621;
% Avz = -1/0.2599;
% Avpsi = 0;
% 
% Bphi = 0.9855/0.1598;
% Btheta = 0.9795/0.1621;
% Bvz = 1.1635/0.2599;
% Bvpsi = 0;
% 
% Cphi = 1;
% Ctheta = 1;
% Cvz = 1;
% Cvpsi = 0;

%% Parameters FO
% Aphi = -1/0.2368;
% Atheta = -1/0.2318;
% Avz = -1/0.3367;
% Avpsi = 0;
% Bphi = 1.126/0.2368;
% Btheta = 1.1075/0.2318;
% Bvz = 1.2270 / 0.3367;
% Bvpsi = 0;
% Cphi = 1;
% Ctheta = 1;
% Cvz = 1;
% Cvpsi = 1;

parameters = {Aphi, Atheta, Avz, Avpsi,...
    Bphi, Btheta, Bvz, Bvpsi, ...
    Cphi, Ctheta, Cvz, Cvpsi, ...
    kDx, kDy, kDz
    };

              
% Kalman state estimation to find initial states of non-physical parameters              
[estXphi,~] = ssEstEKF(data(1:simBegin,7,1).OutputData, data(1:simBegin,7,1).InputData, Aphi, Bphi, Cphi);
[estXtheta,~] = ssEstEKF(data(1:simBegin,8,2).OutputData, data(1:simBegin,8,2).InputData, Atheta, Btheta, Ctheta);
[estXvz,~] = ssEstEKF(data(1:simBegin,6,3).OutputData, data(1:simBegin,6,3).InputData, Avz, Bvz, Cvz);
estvpsi = 0*estXvz; % vpsi was not logged during experiments, for now, 0
estXvpsi = 0*estvpsi; % vpsi was not logged during experiments, for now, 0

data = data(simBegin+1:length(data.OutputData), :,:);    % Cutting off data that was used for the state estimation

initialstates = [data.OutputData(1,1:5)'; ... %x,y,z,vx,vy
    estXvz;estXphi; estXtheta; estXvpsi; ... %initial values for non-physical states
    data.OutputData(1, 9)';... %psi   
];

fullModel = idnlgrey(modelFile, order, parameters,initialstates, 0); %continous time model
fullModel.simulationOptions.Solver = 'ode45';
set(fullModel, 'InputName', {'Commanded Roll','Commanded Pitch','Commanded vz', 'Commanded Yaw rate'}, ...
                  'InputUnit', {'(rad)','(rad)','(m/s)', '(rad/s)'},    ...
                  'OutputName', {'X', 'Y', 'Z', 'Velocity in x','Velocity in y','Velocity in z' 'Roll',' Pitch', 'Yaw'}, ...
                  'OutputUnit', {'(m)', '(m)','(m)', '(m/s)','(m/s)','(m/s)','(rad)','(rad)','(rad)'},  ...
                  'TimeUnit', 's');

% Now that states are estimated for this data set, set them fixed
trueArray = {};      
for i = 1:length(initialstates) %for all states
    trueArray{i} = true;
end
fullModel = setinit(fullModel, 'Fixed', trueArray);

%% Time delay
% The determined delay for the model. It is assumed that the response for
% all inputs is delayed by a different delay.  
phi_c   = data.InputData(:,1);
theta_c = data.InputData(:,2);
vz_c    = data.InputData(:,3);
vpsi_c  = data.InputData(:,4);

TD_theta = 0;
TD_phi   = 0;
TD_vz    = 0;
TD_vpsi  = 0;

TimeDelay = model.UserData; %model.UserData = [TD_theta, TD_phi, TD_vz, TD_vpsi];
%TimeDelay = [1, 1, 1, 0]; %for simulating FOPTD
if isempty(TimeDelay)   % Add time delay if the model uses a time delay                                                           
    TD_theta = 0;
    TD_phi   = 0;
    TD_vz    = 0;
    TD_vpsi  = 0;
else        
    TD_theta = TimeDelay(1);
    TD_phi   = TimeDelay(2);
    TD_vz    = TimeDelay(3);
    TD_vpsi  = TimeDelay(4);
end

dummy = zeros(length(theta_c),1);
for i = TD_theta+1:length(dummy)
    dummy(i,1) = theta_c(i-TD_theta,1);
end
theta_c_delay = dummy;

dummy = zeros(length(phi_c),1);
for i = TD_phi+1:length(dummy)
    dummy(i,1) = phi_c(i-TD_phi,1);    
end
phi_c_delay = dummy;

dummy = zeros(length(vz_c),1);
for i = TD_vz+1:length(dummy)
    dummy(i,1) = vz_c(i-TD_vz,1);
end
vz_c_delay = dummy;

dummy = zeros(length(vpsi_c),1);
% for i = TD_vpsi+1:length(dummy)
%     dummy(i,1) = vpsi_c(i-TD_vz,1);
% end
vpsi_c_delay = dummy;

%% Simulating response
simData = data;
simData.InputData(:,1) = phi_c_delay;
simData.InputData(:,2) = theta_c_delay;
simData.InputData(:,3) = vz_c_delay;
simData.InputData(:,4) = vpsi_c_delay;

simData = sim(fullModel, simData);
valData = data;

%% Calculating fitness
simOutput = simData.OutputData;
sim_vz  = simOutput(:,6);
sim_phi = simOutput(:,7);
sim_theta= simOutput(:,8);

vz      = valData.OutputData(:,6);
phi     = valData.OutputData(:,7);
theta   = valData.OutputData(:,8);  

fit_vz = round( goodnessOfFit(sim_vz, vz, 'NRMSE')*100, 2) ;
fit_phi = round( goodnessOfFit(sim_phi, phi, 'NRMSE')*100, 2) ;        
fit_theta = round( goodnessOfFit(sim_theta, theta, 'NRMSE')*100, 2) ;

fit_score = mean([fit_phi, fit_theta, fit_vz]);

end

