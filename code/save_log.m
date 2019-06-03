%% Save Log script
%
% Run this file after performing experiments in the MATLAB instance which 
% had the run_main.m running. This will save the log variables to a MAT
% file
% Saves all variables from workspace starting with log or logidx

%% Prompt user for file name prefix

prefix = input('(Optional) Enter a prefix for MAT file (output: <prefix>_log_DATE.mat: ','s');

%% Identify activations of MPC controller
%%% Create array of 1's where MPC mode was active in log
% log_triggerMPC = ~isnan(log_MPC(1,:)); 

%%% Determine indicies where MPC is triggered
% triggerMPC_diff = diff(log_triggerMPC);

% If MPC is initialised from beginning (simulation) include index 1
% triggerMPC_up = [];
% if ~isnan(log_MPC(1,:))
%     triggerMPC_up = [1];
% end

% triggerMPC_up = [triggerMPC_up find(triggerMPC_diff == 1)];
% triggerMPC_down = find(triggerMPC_diff == -1);
% 
% % Fill with NaNs if up-down not in pair
% sztriggerMPC_up = size(triggerMPC_up,2);
% sztriggerMPC_down = size(triggerMPC_down,2);
% 
% if sztriggerMPC_up > sztriggerMPC_down
%     triggerMPC_down = [triggerMPC_down, nan(1,(sztriggerMPC_up - sztriggerMPC_down))];
% elseif sztriggerMPC_down > sztriggerMPC_up
%     triggerMPC_up = [triggerMPC_up, nan(1,(sztriggerMPC_down - sztriggerMPC_up))];
% end
% 
% logidx_triggerMPC = vertcat(triggerMPC_up,triggerMPC_down);

%% Prepare data for log storage

%%% Store scenario in scn struct

% scn = struct;
% scn.quad.quadStartPos = quadStartPos;
% scn.quad.quadEndPos = quadEndPos;
% 
% if nObs > 0
%     scn.obs.obsStartPos = obsStartPos;
%     scn.obs.obsEndPos = obsEndPos;
%     scn.obs.obsTimer = obsTimer;
%     scn.obs.obsDim = obsDim;
%     scn.obs.obsBuffers = obsBuffers;
%     scn.obs.obsStatic = obsStatic;
% end



%%% Time log
% The dt logged is the control loop time for the previous loop iteration. 
% Therefore, we must shift the recorded dt left by 1 to match other logs

% 
% 
% %%% Time log
% % The dt logged is the control loop time for the previous loop iteration. 
% % Therefore, we must shift the recorded dt left by 1 to match other logs
% 
% log_TimeShifted = [log_Time(:,2:end) NaN(size(log_Time,1),1)];

%% Save variables to MAT file

datenow = datestr(now,'ddmmyyyy_HHMMSS');

curdir = pwd; 
iddir = strfind(curdir,'/');
savedir = curdir; %curdir(1:iddir(end)-1);

if ~isempty(prefix)
    filename = strcat(savedir,'/logs/',prefix,'_log_', datenow,'.mat');
else
    filename = strcat(savedir,'/logs/log_', datenow,'.mat');
end


%save(filename,'share','scn','-regexp','^log','^logidx');
save(filename,'-regexp','^log');

fprintf('[%s] Saved log data to %s \n',datestr(now,'HH:MM:SS'),filename);
