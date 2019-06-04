%% CCDInitial 
% Runs model identification for first order and first order plus time delay
% models. Initial parameters values are found from central composite
% design. It generates a set of 100 initial samples within the ranges 
% supplied. These samples are run for every dataset (5x) and every model 
% (2x). So 1000x in total. 

clear all; close all;
DefaultDataPath = '..\processing\data\rbs';
Index = 0;
Function = 1;       %1 = FOPTD BestBuess files verzamelen
                    %2 = Bestguess files genereren (long run time)
switch Function
    case 1
        DefaultDataPath = '/Results/Models';
        BestGuessFO = cell(5,3);
        BestGuessFOPTD = cell(5,3);
        Names =    cellstr(["nlFirstOrderest_rbs_rbs_rbs_log_09052019_120935_processed.mat";
                            "nlFirstOrderest_rbs_rbs_rbs_log_09052019_121028_processed.mat";
                            "nlFirstOrderest_rbs_rbs_rbs_log_09052019_121250_processed.mat";
                            "nlFirstOrderest_rbs_rbs_rbs_log_09052019_122515_processed.mat";
                            "nlFirstOrderest_rbs_rbs_rbs_log_09052019_122633_processed.mat";
                            "nlFirstOrderestPTD_rbs_rbs_rbs_log_09052019_120935_processed.mat";
                            "nlFirstOrderestPTD_rbs_rbs_rbs_log_09052019_121028_processed.mat";
                            "nlFirstOrderestPTD_rbs_rbs_rbs_log_09052019_121250_processed.mat";
                            "nlFirstOrderestPTD_rbs_rbs_rbs_log_09052019_122515_processed.mat";
                            "nlFirstOrderestPTD_rbs_rbs_rbs_log_09052019_122633_processed.mat"]);
                
        for i = 1:5
            [fileName,path]=uigetfile('*.mat','Select the data File',DefaultDataPath);
            load(fullfile(path,fileName));
        
            for J = 1:size(Fit,2)
                FitM(i) = mean([Fit(J,1), Fit(J,2), Fit(J,3), Fit(J,4), Fit(J,5)]);
            end
            MaxFit = max(FitM);
            k = find(abs(FitM-MaxFit) < 0.0001);
            BestGuessFO(i,1) = Names(i);
            BestGuessFO(i,2) = cellstr(num2str(InitialSamples(k,:)));
            BestGuessFO(i,3) = cellstr(num2str(k));
        end
        for i = 1:5
            [fileName,path]=uigetfile('*.mat','Select the data File',DefaultDataPath);
            load(fullfile(path,fileName));
        
            for J = 1:size(Fit_TD,2)
                FitM(i) = mean([Fit_TD(J,1), Fit_TD(J,2), Fit_TD(J,3), Fit_TD(J,4), Fit_TD(J,5)]);
            end
            MaxFit = max(FitM);
            k = find(abs(FitM-MaxFit) < 0.0001);
            BestGuessFOPTD(i,1) = Names(5+i);
            BestGuessFOPTD(i,2) = cellstr(num2str(InitialSamples(k,:)));
            BestGuessFOPTD(i,3) = cellstr(num2str(k));
        end
        save('..\systemidentification\Results\Models\BestGuessFOPTD','BestGuessFOPTD');
        save('..\systemidentification\Results\Models\BestGuessFO','BestGuessFO'); 
% Run CCD  
    case 2
        [t_fileName,t_path]=uigetfile('*.mat','Select the INPUT DATA FILE(s)','MultiSelect','on');
        for t_i = 1:length(t_fileName)   
        kPhiRange     =   [0.01 5];   %1.13
        tauPhiRange   =   [0.01 5];   %0.24
        kThetaRange   =   [0.01 5];   %1.06
        tauThetaRange =   [0.01 5];   %0.22
        kDxRange      =   [0.01 7];   %0.26
        kDyRange      =   [0.01 7];   %0.47
        kvzRange      =   [0.01 5];   %1.20
        tauzRange     =   [0.01 5];   %0.37

        Range = [kPhiRange;
                tauPhiRange;
                kThetaRange;
                tauThetaRange;
                kDxRange;
                kDyRange;
                kvzRange;
                tauzRange];

        	
             dCC = ccdesign(8,'type','circumscribed','center',1);     
        InitialSamples = zeros(size(dCC));


        %%mapping the dCC matrix
        for i = 1:size(dCC,2)
         InitialSamples(:,i) = rescale(dCC(:,i),Range(i,1),Range(i,2));
        end

        dataName = string(t_fileName(t_i));
        Path = string(t_path);

        Fit = zeros(size(InitialSamples,1),5);
        FitM = zeros(size(InitialSamples,1),1);


        %% FONL
        for J = 1:size(InitialSamples,1)
        Index = Index + 1;
        fprintf('FONL, Iternation FUCKING %s \n', num2str(Index));
        InitialSample = InitialSamples(J,:);
        [fit] = FONLIFnc(dataName, Path, InitialSample, Index);
        Fit(J,:) = fit';
        FitM(J) = mean([Fit(J,1), Fit(J,2), Fit(J,5)]);  %without velocities
        end
        MaxFit = max(FitM);

        k = find(Fit==MaxFit);
        BestGuess = InitialSamples(k,:);

        FolderName =strcat('nlFirstOrderest_',dataName,'FOLDER');
        FolderLocation = strcat(pwd, '/Results/Models/', FolderName);
        save(strcat(FolderLocation, '/BestGuess'),'BestGuess', 'Range', 'MaxFit', 'Fit', 'InitialSamples');    


        %% FONLPTD
        for J = 1:size(InitialSamples,1)
        Index = Index + 1;
        fprintf('FONLPTD, Iternation FUCKING %s \n', num2str(Index));
        InitialSample = InitialSamples(J,:);
        [fit] = FOPTDNIFnc(dataName, Path, InitialSample, Index);
        Fit_TD(J,:) = fit';
        FitM_TD(J) = mean([Fit_TD(J,1), Fit_TD(J,2), Fit_TD(J,5)]);  %without velocities
        end
        MaxFit_TD = max(FitM);

        k_TD = find(Fit==MaxFit);
        BestGuess_TD = InitialSamples(k,:);

        FolderName =strcat('nlFirstOrderestPTD_',dataName,'FOLDER');
        FolderLocation = strcat(pwd, '/Results/Models/', FolderName);
        save(strcat(FolderLocation, '/BestGuess'),'BestGuess_TD', 'Range', 'MaxFit_TD', 'Fit_TD', 'InitialSamples');    
        end
end