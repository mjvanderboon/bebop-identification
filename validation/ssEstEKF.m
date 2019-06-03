function [estState,estStates] = ssEstEKF(output,input,A,B,C)
%ssEstEKF EKF for estimating state space state from output and input time vector
%for single input single output system, arbitrary state size
% Estimates state up to the last index
%   output:     measured output
%   input:      commanded input
%   A,B,C:      identified matrices
%   TODO: add adjustable sampling time

 %% EKF
initial = zeros(size(C,1),1);
xstates = zeros(size(C,1),length(input))';

C = C'; %vanwege gek iets in system identification toolbox
output = output';
input = input';

Kalman = extendedKalmanFilter(@StateTransitionFcn,@MeasurementFcn,initial);

for k = 1:length(output)
    [CorrectedState,CorrectedStateCovariance] = correct(Kalman,output(:,k),C);
    [PredictedState,PredictedStateCovariance] = predict(Kalman,A,B,input(:,k));
    xstates(k,:) = CorrectedState;
end

estState = xstates(length(output),:)';
estStates = xstates';

end

%% State transistion
function [x] = StateTransitionFcn(x,A,B,input)
dx = A*x+B*input ;
x = x + dx * 0.05; %have to return state vector
end
%% Measure
function [y] = MeasurementFcn(x,C)
y = C*x;
end


