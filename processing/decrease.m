close all;
clear all ;
t_Multiselect = 1;

switch t_Multiselect
    case 0
        [t_fileName,t_path]=uigetfile('*.mat');
         t_fileName = {t_fileName} ; 
    case 1
        [t_fileName,t_path]=uigetfile('*.mat','Select the INPUT DATA FILE(s)','MultiSelect','on'); 
end

for t_i=1:length(t_fileName)
    %Load data
    t_dummy = t_fileName{t_i};
    load(fullfile(t_path,t_fileName{t_i}));
    fprintf('[%s Loading data %s%s \n', datestr(now,'HH:MM:SS'), t_path, t_fileName{t_i});

t_tbegin = 58;
t_tend = length(phi) ;
phi = phi(t_tbegin:t_tend) ;
phi_c = phi_c(t_tbegin:t_tend) ; 
psi = psi(t_tbegin:t_tend) ; 
theta = theta(t_tbegin:t_tend) ;
theta_c = theta_c(t_tbegin:t_tend) ;
Time_real = Time_real(t_tbegin:t_tend) ;
vphi = vphi(t_tbegin:t_tend) ;
vpsi = vpsi(t_tbegin:t_tend) ;
vtheta = vtheta(t_tbegin:t_tend) ;
vx = vx(t_tbegin:t_tend) ;
vy = vy(t_tbegin:t_tend) ;
vz = vz(t_tbegin:t_tend) ;
vz_c = vz_c(t_tbegin:t_tend) ;
x = x(t_tbegin:t_tend) ;
y = y(t_tbegin:t_tend) ;
z = z(t_tbegin:t_tend) ;

newFile = strcat(pwd,filesep,'data',filesep,'steadystate',filesep,t_dummy); %filesep is platform specific separator (linux '/', windows '\')
save(newFile,'-regexp','^(?!t_.*$).');
end

