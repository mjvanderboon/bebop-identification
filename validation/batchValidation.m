%% Batchvalidation
% validate multiple models files over data sets
% returns model that (on average) is best over all datasets
%% Settings
validationMode          = 0;    %0 = sim, 1 = predict
predictionHorizon       = 10;   % prediction horizon to use for prediction
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
        % Get timestamp of model and data (to see if it is the same data set)
        modelStamp = getNameTimeStamp(modelNames{i});
        dataStamp = getNameTimeStamp(dataNames(j));
            
        if ~strcmp(dataStamp, modelStamp) %not over own data set
            data = createFullDataObject(dataNames{j},dataPath);
            [val, sim, fit_score] = validationFnc(model,data);                                
            modelResult.allFits(1, j) = dataNames(j);
            modelResult.allFits(2, j) = {fit_score};

            % Save dataset that had best fit, except for model's own data set
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

function [stamp] = getNameTimeStamp(fullName) % Gets timestamp of fullName
        fullName = split(fullName,'_');
        fullName = flipud(fullName);
        stamp = fullName{2};
end