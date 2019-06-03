%% Batch identify dead time of all channels
% Settings
clear all;
DefaultDataPath = '..\processing\data\step response';

% Outputs deadTimes
% deadTimes = [
% phi....;
% theta...;
% vz....;
% vpsi....;
%];

%%
deadSteps = zeros(4,1); %dead times for phi, theta, vz, vpsi for all data sets
for i = 1:4 %for phi, theta, vz, vpsi
    [dataFileNames,path]=uigetfile('*.mat',strcat('Select the INPUT DATA FILE(s) for channel ', num2str(i)),'MultiSelect','on',DefaultDataPath);

    % If one is selected it is a char array
    % Make it into cell array to it to work as well
    if (ischar(dataFileNames))
        dataFileNames = {dataFileNames};
    end
    
    for j = 1:length(dataFileNames)     
        dataName = string(dataFileNames(j));               
        deadSteps(i, j) = deadTimeEstimation(i,dataName,path); % channel: 1 (phi), 2 (theta), 3 (vz), 4 (vpsi)
    end
end