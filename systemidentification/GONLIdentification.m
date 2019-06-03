function [fit] = GONLIdentification(dataName,path,initParams,prefix,order,estimateMethod,timeDelay)
%% Function to identify parameters of second order model
% Input for function:
% dataName: Name of the data file used for identification
% path:     Path to the folder in which data file is contained
%           If both are empty a UI menu is opened to select a data file
% initParams: multidimensional struct initParams.phi.A/B/C, initParams.theta..... C MATRICES HAVE TO BE SAME DIMENSION AS B! ROWS
% prefix:    Prefix string used in final name of saved model
% order:    model order
% estimate<Method:0 no estimation, 1 greyest, 2 ssest
% timeDelay: vector for time delay [phi, theta, vz, vpsi]

%% Settings
fileName = 'GO_InputResponse'; 
modelOrder= [4 4 order*4]; %Model orders [ny nu nx(phi, theta, vz, vpsi)]

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

%% Initial parameters
% If no initial values are given, generate random ones
if (isempty(initParams))
    initParams.phi.A = 2*rand(order)-1;
    initParams.theta.A  = 2*rand(order)-1;
    initParams.vz.A     = 2*rand(order)-1;
    initParams.vpsi.A   = 2*zeros(order);

    initParams.phi.B    = 2*rand(order,1)-1;
    initParams.theta.B  = 2*rand(order,1)-1;
    initParams.vz.B     = 2*rand(order,1)-1;
    initParams.vpsi.B   = 2*zeros(order,1)-1;

    %C are [;;], matrix parameters for sys ident seem to have to be square or [;;] (not [,,])
    initParams.phi.C    = 2*rand(order,1)-1; 
    initParams.theta.C  = 2*rand(order,1)-1;
    initParams.vz.C     = 2*rand(order,1)-1;
    initParams.vpsi.C   = 2*zeros(order,1)-1;
end

Aphi    = initParams.phi.A;
Bphi    = initParams.phi.B;
Cphi    = initParams.phi.C;

Atheta    = initParams.theta.A;
Btheta    = initParams.theta.B;
Ctheta    = initParams.theta.C;

Avz    = initParams.vz.A;
Bvz    = initParams.vz.B;
Cvz    = initParams.vz.C;

Avpsi    = initParams.vpsi.A;
Bvpsi    = initParams.vpsi.B;
Cvpsi    = initParams.vpsi.C;

if (order == 2) %Use earlier found params (from nikhal?)
%     Aphi = [-1.389 -4.762; 9.2927 -12.3912];
%     Atheta = [-2.4786 -3.5531; 10.3409 -7.6167];
%     Avz = Aphi;
%         
%     Bphi = [-5.3858; -17.9041];
%     Btheta = [-0.8068; -15.9086];
%     Bvz = Bphi;
% 
%     Cphi = [1.2691 -0.4839]';
%     Ctheta = [0.8400 -0.1292]';    
%     Cvz = Cphi;
end

%% Time delay
TD_phi      = timeDelay(1);
TD_theta    = timeDelay(2);
TD_vz       = timeDelay(3);
TD_vpsi     = timeDelay(4);

dummy = zeros(length(phi),1);
for i = TD_phi+1:length(phi)
    dummy(i,1) = phi_c(i-TD_phi,1);    
end
phi_c = dummy;

dummy = zeros(length(phi),1);
for i = TD_theta+1:length(phi)
    dummy(i,1) = theta_c(i-TD_theta,1);
end
theta_c = dummy;

dummy = zeros(length(phi),1);
for i = TD_vz+1:length(phi)
    dummy(i,1) = vz_c(i-TD_vz,1);
end
vz_c = dummy;
    
dummy = zeros(length(phi),1);
% for i = TD_vz+1:length(phi) %commented because vpsi is often not used in experiments
%     dummy(i,1) = vpsi_c(i-TD_vz,1);
% end
vpsi_c = dummy;
vpsi = dummy; %because there is no data. If wanted, have to use kalman filter on data

%% Create Data object
sampleT = mean(log_Time(1,:));
InputAttitude  = [phi_c, theta_c, vz_c, vpsi_c]; %is shifted with timedelay
OutputAttitude = [phi, theta, vz, vpsi];

estData = iddata(OutputAttitude, InputAttitude, sampleT, 'Name','Measured data');
estData.InputName = {'Commanded Roll','Commanded Pitch','Commanded vz', 'Commanded yaw rate'};
estData.InputUnit =  {'(rad)','(rad)','(m/s)','(rad/s)'};
estData.OutputName = {'Measured Roll','Measured Pitch','Velocity in z','Yaw rate'};
estData.OutputUnit = {'(rad)','(rad)','(m/s)','rad/s'};
estData.Tstart = 0;
estData.TimeUnit = 's';

%% Grey box creation and estimation
if (estimateMethod == 2) % Use state space estimation   
    ssopt = ssestOptions('Display', 'off','SearchMethod','lsqnonlin','EnforceStability',true);      
	[Aphi, Bphi, Cphi,phi0]       = stateSpaceEstimation(estData, 1, ssopt, initParams.phi);
    [Atheta, Btheta, Ctheta,theta0] = stateSpaceEstimation(estData, 2, ssopt, initParams.theta);
    [Avz, Bvz, Cvz,vz0]          = stateSpaceEstimation(estData, 3, ssopt, initParams.vz);
    [Avpsi, Bvpsi, Cvpsi,vpsi0]    = stateSpaceEstimation(estData, 4, ssopt, initParams.vpsi);    
    initialstates = [phi0; theta0; vz0; vpsi0];
else
    initialstates = ones(order*4,1)*.1; %Initial states are later set to free, greyest changes init states
end

aux = {};
parameters    = {Aphi; Atheta; Avz; Avpsi; Bphi; Btheta; Bvz; Bvpsi; Cphi; Ctheta; Cvz; Cvpsi};            
Ts            = 0;                                                      
model = idnlgrey(fileName, modelOrder, parameters, initialstates, Ts, 'Name',strcat(num2str(order), ' Order Model'));
    
set(model, 'InputName', estData.InputName, ...
                  'InputUnit', estData.InputUnit,    ...
                  'OutputName', estData.OutputName, ...
                  'OutputUnit', estData.OutputUnit,  ...
                  'TimeUnit', estData.TimeUnit);

%Set all initial (non-physical) states to not be fixed              
falseArray = {};      
for i = 1:order*4 %for all states
    falseArray{i} = false;
end
model = setinit(model, 'Fixed', falseArray);
                                                                        
if (estimateMethod == 1) %use non-linear greybox estimation
    opt = nlgreyestOptions('Display','on','SearchMethod','lsqnonlin');
    opt.SearchOptions.MaxIterations = 50;
    model = nlgreyest(estData, model, opt);
end

[output, fit, x0] = compare(estData,model);

%Add the time delay data to the model object for availability in simulation
model.UserData = [TD_theta, TD_phi, TD_vz, TD_vpsi];

%% Save second order model 
folderLocation = strcat(pwd, filesep, 'Results', filesep, 'GOModels', filesep,num2str(order),'_');

if (all(timeDelay) ~= 0)
    folderLocation = strcat(folderLocation,'PTD_');
end

savename = strcat(folderLocation,prefix,'_', dataName);
save(savename,'model');  

end