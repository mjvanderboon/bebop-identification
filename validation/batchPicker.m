%% batchPicker
% For every data set multiple models are identified based on different
% initial parameter values. 
% This script picks the best performing identified model for each dataset.
% Running the script opens ui dialogs to select the data sets, and models
% for each data set.
% The best models for every data set are returned in the variable
% bestModels = {data set name; modelname; fit percentage}

%% Setup
validationFnc = @simGOFull;
% Default opening directories
modelPath = '..\systemidentification\Results\GOModels';
dataPath = '..\processing\data';           

[dataNames,dataPath]=uigetfile('*.mat','Select data files',dataPath,'Multiselect','on');
% If one is selected it is a char array
% Make it into cell array to it to work as well
if (ischar(dataNames))
    dataNames = {dataNames};
end

bestModels  = {}; %{dataname; modelname; fit percentage, 
for i = 1:length(dataNames)    
    fprintf('\n\n[%s Loading data set %s \n', datestr(now,'HH:MM:SS'), dataNames{i});
    data = createFullDataObject(dataNames{i},dataPath); % create iddata object from data
    
    [modelNames,modelPath]=uigetfile('*.mat',strcat('Select model files for ',dataNames{i}),modelPath,'Multiselect','on');
        
    bestModel       = 0;
    bestFit         = 0;
    allFits         = {};
    for j = 1:length(modelNames)                
        fprintf('[%s Loading model %s%s \n', datestr(now,'HH:MM:SS'), modelPath, modelNames{j});
        load(fullfile(modelPath,modelNames{j})); 

        if isempty(model)
            return 
        end

        [val, sim, fit_score] = validationFnc(model,data);                        
        allFits{1,j} = fit_score;
        allFits{2,j} = modelNames{j};

        % Save dataset that has best fit
        if fit_score > bestFit        
            bestFit = fit_score;
            bestModel = modelNames{j};
        end       
    end

    [bestfit,bestindex] = max([allFits{1,:}]);
    fprintf('\n[%s Best model: %s, fit: %s \n', datestr(now,'HH:MM:SS'), allFits{2,bestindex}, allFits{1,bestindex});
    bestModels{i,1} = dataNames{i};
    bestModels{i,2} = allFits{2,bestindex};
    bestModels{i,3} = allFits{1,bestindex};    
end