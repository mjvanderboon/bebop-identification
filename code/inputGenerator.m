% Generate signal
% Upper bandlimit is system bandwidth
% Lower limit is aan lab limitaties (ruimte)
% rbs omdat dat het meeste lijkt op uiteindelijke toepassing dus waarschijnlijk ook beste fit voor toepassing
function [u,n] = inputGenerator(low, high)
fs = 20;       % sampling frequency (Hz)
cutoff = high;  % high cutoff frequency (Hz) - volgt uit system bandwidth van step response
lowcut = low;    % low cutoff frequency (Hz) - volgt uit lab ruimte limitaties
StopTime = 60;  % lengte signaal (s)
A = .5;         % amplitude signaal
plotting = 0;   % if plotting

dt = 1/fs;
t = (0:dt:StopTime-dt)';
n = size(t,1);
nyquist = fs/2;

% Random binary signal
band = [lowcut, cutoff]/nyquist;
u = idinput(n,'rbs',band); %random binary signal with bandwidth [0, cutoff frequency]
u = u * A;

% Square wave
% u = zeros(1,n);
% omega = .5;
% u = square(omega*t*(2*pi));

if plotting
    data = iddata([],u,dt);

    figure();
    subplot(2,1,1);
    plot(data);
    xlabel('time (s)');
    title('input signal');

    % Frequency domain
    y = fft(u);
    f = (0:n-1)*(fs/n);     % frequency range
    power = abs(y).^2/n;    % power of the DFT
    subplot(2,1,2);
    plot(f,power)
    xlabel('Frequency (Hz)')
    ylabel('Power')
end