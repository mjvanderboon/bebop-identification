function [estState,estStates] = ssEstEKF(output,input,A,B,C)
%% ssEstEKF EKF for estimating state space model state from output and input time vector
%for single input single output system, arbitrary state size
% Estimates state up to the last index
%   output:     measured output
%   input:      commanded input
%   A,B,C:      identified matrices
%   TODO: add adjustable sampling time, currently fixed at 0.05

 %% EKF
initial = zeros(size(C,1),1);
xstates = zeros(size(C,1),length(input))';

C = C'; %C matrices have to be transposed because of system identification toolbox limitations
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
x = x + dx * 0.05;
end
%% Measure
function [y] = MeasurementFcn(x,C)
y = C*x;
end


