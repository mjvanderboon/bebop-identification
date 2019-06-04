function [] = plotResultAndSave(valData, simData)
%% plotResultAndSave Plots the simulation against the measurements
% Takes 2 iddata objects, simulation and measurements
% iddata objects are according to convention in createFullDataObject.m
% val en sim datasets are generated in simGOFull

%% Settings
inputColor = [1,0,0];
simColor = [0,0,1];
valColor = [0,0,0];
inputStyle = '-.';
simStyle = '-';
valStyle = '--';
lineWidth = 1;

saveFigs = 1;          %Save figs generated

% For figures in appendix (full size)
figpos = [0 0 1 1];         %figure position
figposinput = [0 0 .66 1];  %figure position
fontSize = 18;
dataCutoff = 1;             %percentage of data to plot

% For figures in results section (smaller size)
dataCutoff = .5;            %percentage of data to plot
figpos = [0 0 .5 1];        %figure position
figposinput = [0 0 .5 1];   %figure position
fontSize = 18;
lineWidth = 2;

%% Reading data
set(0, 'defaultTextInterpreter', 'latex', 'defaultAxesTickLabelInterpreter', 'latex', ...
    'defaultLegendInterpreter', 'latex', 'defaultAxesFontSize', fontSize, 'DefaultLineLineWidth',lineWidth);

valData = valData(1:floor(dataCutoff*length(valData.SamplingInstants)),:,:);
simData = simData(1:floor(dataCutoff*length(simData.SamplingInstants)),:,:);
simOutput = simData.OutputData;
valOutput = valData.OutputData;
valInput = valData.InputData;

t = valData.SamplingInstants;

sim_x   = simOutput(:,1);
sim_y   = simOutput(:,2);
sim_z   = simOutput(:,3);
sim_vx  = simOutput(:,4);
sim_vy  = simOutput(:,5);
sim_vz  = simOutput(:,6);
sim_phi = simOutput(:,7);
sim_theta= simOutput(:,8);
sim_psi = simOutput(:,9);

x   = valOutput(:,1);
y   = valOutput(:,2);
z   = valOutput(:,3);
vx  = valOutput(:,4);
vy  = valOutput(:,5);
vz  = valOutput(:,6);
phi = valOutput(:,7);
theta=valOutput(:,8);
psi = valOutput(:,9);

phi_c       = valInput(:,1);
theta_c     = valInput(:,2);
vz_c        = valInput(:,3);
omega_c     = valInput(:,4);   

lengthSim = length(phi);

%% Calculate fit parameters  
    %calculate NRMSE for phi, theta, vx, vy and vz between the simulated and 
    %measured data. Rounded on 2 decimal points     
fit_x = round( goodnessOfFit(sim_x, x, 'MSE')*100, 2);
fit_y = round( goodnessOfFit(sim_y, y, 'MSE')*100, 2);
fit_z = round( goodnessOfFit(sim_z, z, 'MSE')*100, 2);
fit_vx = round( goodnessOfFit(sim_vx, vx, 'NRMSE')*100, 2) ;
fit_vy = round( goodnessOfFit(sim_vy, vy, 'NRMSE')*100, 2) ;
fit_vz = round( goodnessOfFit(sim_vz, vz, 'NRMSE')*100, 2) ;
fit_phi = round( goodnessOfFit(sim_phi, phi, 'NRMSE')*100, 2) ;        
fit_theta = round( goodnessOfFit(sim_theta, theta, 'NRMSE')*100, 2) ;
fit_psi = round( goodnessOfFit(sim_psi, psi, 'NRMSE')*100, 2) ;

%% Calculate absolute and relative error
absErrorStates = zeros(lengthSim,size(valOutput,2));
relErrorStates = zeros(lengthSim,size(valOutput,2));

for i = 1:lengthSim-1 % Compute absolute and relative error
    absErrorStates(i,:) = abs(simOutput(i,:)-valOutput(i,:));              
    relErrorStates(i,:) = (simOutput(i,:)-valOutput(i,:))/valOutput(i,:);  
end 

errAbs_x	= absErrorStates(:,1);                               
errAbs_y	= absErrorStates(:,2);
errAbs_z	= absErrorStates(:,3);
errAbs_vx	= absErrorStates(:,4);
errAbs_vy	= absErrorStates(:,5);
errAbs_vz	= absErrorStates(:,6);
errAbs_phi	= absErrorStates(:,7);                                          
errAbs_theta= absErrorStates(:,8);
errAbs_psi	= absErrorStates(:,9);

errRel_x	= relErrorStates(:,1);
errRel_y	= relErrorStates(:,2);
errRel_z	= relErrorStates(:,3);
errRel_vx	= relErrorStates(:,4);
errRel_vy	= relErrorStates(:,5);
errRel_vz 	= relErrorStates(:,6);
errRel_phi	= relErrorStates(:,7);                                          
errRel_theta= relErrorStates(:,8);
errRel_psi	= relErrorStates(:,9);

%% Plottings
figs = [];

%% Input output response
m = 3; n = 1;                                                          %mxn plot                

set(0, 'defaultTextInterpreter', 'latex', 'defaultAxesTickLabelInterpreter', 'latex', ...
    'defaultLegendInterpreter', 'latex', 'defaultAxesFontSize', fontSize, 'DefaultLineLineWidth',lineWidth);

figs(1) = figure('Name',strcat('Simulated response',simData.ExperimentName{1},valData.ExperimentName{1}),'units','normalized','outerposition',figposinput);                                    
%sgt = sgtitle('Simulation $\theta$, $\phi$, $v_z$    of $4^{th}$ order model','interpreter','latex');
sgt.FontSize = 20;
subplot(m,n,1)
    title(['Roll angle $\phi$, NRMSE =' num2str(fit_phi) '\%']);
    hold on
    plot(t,phi_c,'color',inputColor,'linestyle',inputStyle,'linewidth',1);        
    plot(t,phi, 'color', valColor,'linestyle', valStyle);
    plot(t,sim_phi,'color', simColor,'linestyle', simStyle);
    legend('Input', 'Measured Data', 'Simulation Result');
    xlabel('t (s)');  
    ylabel('$\phi$ (rad)');

subplot(m,n,2)
    title(['Pitch angle $\theta$, NRMSE = ' num2str(fit_theta) '\%']);
    hold on;
    plot(t,theta_c,'color',inputColor,'linestyle',inputStyle,'linewidth',1);        
    plot(t,theta, 'color', valColor,'linestyle', valStyle);
    plot(t,sim_theta,'color', simColor,'linestyle', simStyle);    
    xlabel('t (s)');
    ylabel('$\theta$ (rad)');

subplot(m,n,3)
    title(['Vertical velocity $v_z$, NRMSE = ' num2str(fit_vz) '\%'])
    hold on;
    plot(t, vz_c,'color',inputColor,'linestyle',inputStyle,'linewidth',1);        
    plot(t, vz,'color', valColor,'linestyle', valStyle);
    plot(t, sim_vz,'color', simColor,'linestyle', simStyle);    
    xlabel('t (s)');
    ylabel('$v_z$ (m/s)'); 

%% Velocity (x,y)
m = 2; n = 1;                                                               %mxn plot                

set(0, 'defaultTextInterpreter', 'latex', 'defaultAxesTickLabelInterpreter', 'latex', ...
    'defaultLegendInterpreter', 'latex', 'defaultAxesFontSize', fontSize, 'DefaultLineLineWidth',lineWidth);

figs(2) = figure('Name','Simulated vx, vy','units','normalized','outerposition',figpos);                                    ;                                              
%sgt = sgtitle('Simulation $v_x$, $v_y$ of $4^{th}$ order model','interpreter','latex');
sgt.FontSize = 20;
subplot(m,n,1)
    title(['Velocity in x, NRMSE =' num2str(fit_vx) '\%']);
    hold on     
    plot(t,vx, 'color', valColor,'linestyle', valStyle);
    plot(t,sim_vx,'color', simColor,'linestyle', simStyle);
    legend('Measured Data', 'Simulation Result');
    xlabel('t (s)');
    ylabel('$v_x$ (m/s)');

subplot(m,n,2)
    title(['Velocity in y, NRMSE =' num2str(fit_vy) '\%']);
    hold on     
    plot(t,vy, 'color', valColor,'linestyle', valStyle);
    plot(t,sim_vy,'color', simColor,'linestyle', simStyle);    
    xlabel('t (s)');  
    ylabel('$v_y$ (m/s)');
    
%% XYZ plot with errors
iend = length(t)-1; 
m = 3; n = 2;                                                          %mxn plot                
figs(3) = figure('Name','Simulation x, y, z of 2 order model with errors','units','normalized','outerposition',figpos);                                          
%sgt = sgtitle('Simulation $x$, $y$, $z$ of $4^{th}$ order model with errors','interpreter','latex');
sgt.FontSize = 20;
subplot(m,n,1)
%   title(['x position, NRMSE =' num2str(fit_x) '\%']);
    title(['x position'])
    hold on        
    plot(t,x, 'color', valColor,'linestyle', valStyle);
    plot(t,sim_x, 'color', simColor,'linestyle', simStyle);
    legend('Measured Data', 'Simulation Result');
    xlabel('t (s)');  
    ylabel('$x$ (m)');
            subplot(m,n,2)
                title('Absolute error x');
                hold on;
                plot(t(1:iend),errAbs_x(1:iend),'k');
                ylabel('Error (m)')
                xlabel('t (s)'); 
                

subplot(m,n,3)
%    title(['y position, NRMSE = ' num2str(fit_y) '\%']);
        title(['y position'])
    hold on;
    plot(t,y, 'color', valColor,'linestyle', valStyle);
    plot(t,sim_y, 'color', simColor,'linestyle', simStyle);
    xlabel('t (s)');
    ylabel('$y$ (m)');
            subplot(m,n,4)
                title('Absolute error y ');
                hold on;
                plot(t(1:iend),errAbs_y(1:iend),'k');
                ylabel('Error (m)');
                xlabel('t (s)');             
subplot(m,n,5)
    %title(['z position, NRMSE = ' num2str(fit_z) '\%'])
    title(['z position'])
    hold on;
    plot(t, z,'color', valColor,'linestyle', valStyle);
    plot(t, sim_z, 'color', simColor,'linestyle', simStyle);    
    xlabel('t (s)');
    ylabel('$z$ (m)');
subplot(m,n,6)
    title('Absolute error z');
    hold on;
    plot(t(1:iend),errAbs_z(1:iend),'k');
    ylabel('Error (m/s)');
    xlabel('t (s)');  
                   
if (saveFigs == 1)
    prefix = char(input('Prefix for plot files: ','s'));
    savefig(figs,strcat('figs', filesep, prefix,simData.ExperimentName{1}, valData.ExperimentName{1}, '.fig')); %Bestandsnamen van val en sim data
    saveas(figs(1),strcat('figs', filesep, prefix,'_input','.png'),'png');
    saveas(figs(2),strcat('figs', filesep, prefix,'_velocities','.png'),'png');
    saveas(figs(3),strcat('figs', filesep, prefix,'_position','.png'),'png');
end
    
end

