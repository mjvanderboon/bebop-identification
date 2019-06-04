%% batchValidation
% Models are validated over data sets that are selected, except for the
% dataset the particular model was identified on. This script opens ui
% dialogs to select models to validate and data sets to validate the models
% over. A struct results is returned containing validation results for all
% models
% results:
%     - model: variable containing greybox model
%     - modelName: name of model file
%     - allFits: fit percentages of model over validation data sets
%     - avgFit: mean(allFits)
%     - stdFit: std(allFits)
%     - bestDataset: name of validation data set model fits best on
%     - bestFit: fit percentage of bestDataset

%% Settings
validationFnc = @simGOFull;
modelPath = '..\systemidentification\Results\Models';
dataPath = '..\processing\data\rbs';           

%% Calculations
[modelNames, modelPath]=uigetfile('*.mat','Select the  model files',modelPath,'Multiselect', 'on');
[dataNames,dataPath]=uigetfile('*.mat','Select data files',dataPath,'Multiselect','on'); 

% If one is selected it is a char array
% Make it into cell array to it to work as well
if (ischar(modelNames))
    modelNames = {modelNames};
end

% Initializing result struct
results = struct('model', 0, 'modelName', 0, 'allFits', 0,'avgFit',0,'stdFit',0, 'bestDataset', 0, 'bestFit', 0);

% Generating results for all models
for i = 1:length(modelNames)                
    fprintf('[%s Loading model %s%s \n', datestr(now,'HH:MM:SS'), modelPath, modelNames{i});
    load(fullfile(modelPath,modelNames{i})); 

    if isempty(model)
        return 
    end            
    
    % Set up return struct for this model
    modelResult.model = model;
    modelResult.modelName = modelNames{i};   
    modelResult.allFits = {};
    bestFit = 0;
    bestDataset = '';
    for j=1:length(dataNames) 
        % Get timestamp in the name of model and data (to see if it is the same data set)
        modelStamp = getNameTimeStamp(modelNames{i});
        dataStamp = getNameTimeStamp(dataNames(j));
            
        if ~strcmp(dataStamp, modelStamp) %Validation does not occur over own data set
            data = createFullDataObject(dataNames{j},dataPath);
            [val, sim, fit_score] = validationFnc(model,data);                                
            modelResult.allFits(1, j) = dataNames(j);
            modelResult.allFits(2, j) = {fit_score};

            % Save validation dataset that had best fit
            if (fit_score > bestFit)
                bestFit = fit_score;
                bestDataset = dataNames{j};
                modelResult.bestDataset = bestDataset;
                modelResult.bestFit = bestFit;
            end    
        end
    end    
    
    modelResult.avgFit = mean([modelResult.allFits{2,:}]);
    modelResult.stdFit = std([modelResult.allFits{2,:}]);
    results(i) = modelResult;
end

resultFileName = char(input('Name for results file: ','s'));
save(resultFileName, 'results');

function [stamp] = getNameTimeStamp(fullName) 
    % Gets timestamp in the filenames of data and model files
    % e.g. 2_27_rbs_rbs_rbs_log_09052019_120935_processed --> 120935
        fullName = split(fullName,'_');
        fullName = flipud(fullName);
        stamp = fullName{2};
end