%% batchIdentify
% Identifies models for selected data sets. Settings can be adjusted to 
% select model order. Initial parameters values are selected from a uniform
% distribution on a selected range. Multiple models can be identified for 
% the same data set (using different initial parameters), to prevent 
% getting stuck in local minima.

%% Settings
order           = 4;  % Model order (per input, all input orders are the same)
timeDelay       = [0,0,0,0]; %phi, theta, vz, vpsi
estMethod       = 1; %1 = greyest, 2= ssest, 0 = no estimation, take initial values
Arange          = 10; %half range of initial sampels for A matrix, from -range/2 to range/2
Brange          = 10;
Crange          = 10;
iterations      = 60; %iterations per data file
DefaultDataPath = '..\processing\data';


%% Model generation
[dataFileNames,path]=uigetfile('*.mat','Select the INPUT DATA FILE(s)','MultiSelect','on',DefaultDataPath);

% If one is selected it is a char array
% Make it into cell array to it to work as well
if (ischar(dataFileNames))
    dataFileNames = {dataFileNames};
end

%% Model identification
for i = 1:length(dataFileNames)     
	dataName = string(dataFileNames(i));   
    
    for j=1:iterations
        initParams.phi.A    = rand(order).*Arange - Arange / 2;
        initParams.theta.A  = rand(order).*Arange - Arange / 2;
        initParams.vz.A     = rand(order).*Arange - Arange / 2;
        initParams.vpsi.A   = zeros(order);

        initParams.phi.B    = rand(order,1).*Brange - Brange / 2;
        initParams.theta.B  = rand(order,1).*Brange - Brange / 2;
        initParams.vz.B     = rand(order,1).*Brange - Brange / 2;
        initParams.vpsi.B   = zeros(order,1);

        initParams.phi.C    = rand(order,1).*Crange - Crange / 2;
        initParams.theta.C  = rand(order,1).*Crange - Crange / 2;
        initParams.vz.C     = rand(order,1).*Crange - Crange / 2;
        initParams.vpsi.C   = zeros(order,1);

        %(dataName,path,initParams,index,order,estimateMethod,timeDelay)        
        GONLIdentification(dataName,path,initParams,num2str(j),order,estMethod,[0,0,0,0]);          
    end
end


