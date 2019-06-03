%% ROS Setup for MATLAB

% --- FOR MULTI COMPUTER ---
% Enter the local ip address here on the ROS network
% Remember to edit ~/.bashrc with the local ip for export ROS_IP on host computer.
% Also edit ~/.bashrc on remote pc

ROS_MASTER_IP = '192.168.1.110';
ROS_IP        = '192.168.1.110';

% --- FOR SINGLE COMPUTER ---
ROS_MASTER_IP = 'localhost';
ROS_IP = 'localhost';

setenv('ROS_MASTER_URI', ['http://',ROS_MASTER_IP,':11311']);
setenv('ROS_IP', ROS_IP);
%setenv('ROS_HOSTNAME', ROS_IP); % This is required on the visualisation computer

try
    fprintf('[%s] ROS: Attempting ROS init... \n',datestr(now,'HH:MM:SS'));
    rosinit;
    
catch
    fprintf('[%s] ROS: ROS will be reinitialised \n',datestr(now,'HH:MM:SS'));
    rosshutdown
    rosinit;
end

