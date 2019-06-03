%% Input analyse
% Plots time domain and frequency spectrum
function [] = inputAnalyse(u);
u = u';
fs = 100;    % Sampling frequency
StopTime = length(u)*1/fs;
plotting = 1;       % if plotting

dt = 1/fs;
t = (0:dt:StopTime-dt)';
n = size(t,1);
nyquist = fs/2;


figure();
subplot(2,1,1);
plot(t, u);
xlabel('time (s)');
ylabel('magnitude');
title('Random binary signal ');

% Frequency domain
y = fft(u);
f = (0:n-1)*(fs/n);     % frequency range
power = abs(y).^2/n;    % power of the DFT
subplot(2,1,2);
plot(f,power)
xlabel('frequency (Hz)')
ylabel('power')