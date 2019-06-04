function [A, B, C, x0] = stateSpaceEstimation(data, channel, opt, initParams)
%% stateSpaceEstimation uses ssest to find A,B,C of 1 output channel
% channel: [phi, theta, vz, vpsi]
% data: input response data

A = initParams.A;
B = initParams.B;
C = initParams.C';
D = zeros(1,1);

m = idss(A,B,C,D,'Ts',0);
data = data(:,channel,channel);
[m,x0] = ssest(data,m,opt);

A = m.Structure.A.Value;
B = m.Structure.B.Value;
C = m.Structure.C.Value';
end