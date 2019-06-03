%% Batch identify models
% Settings
order           = 4;  % Model order
timeDelay       = [0,0,0,0]; %phi, theta, vz, vpsi
estMethod       = 2; %1 = greybox, 2= ss, 0 = no estimation
Arange          = 10; %half range for A matrix, from -range/2 to range/2
Brange          = 10;
Crange          = 10;
iterations      = 60; % iterations per data file
DefaultDataPath = '..\processing\data';


%% Model generation
[dataFileNames,path]=uigetfile('*.mat','Select the INPUT DATA FILE(s)','MultiSelect','on',DefaultDataPath);

% If one is selected it is a char array
% Make it into cell array to it to work as well
if (ischar(dataFileNames))
    dataFileNames = {dataFileNames};
end

%% fourth order
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
        %GONLIdentification(dataName,path,initParams,num2str(j),order,estMethod,[2,2,2,2]);
    end
end


