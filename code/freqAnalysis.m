close all; clear all; clc;

[ppi, n] = preplannedinput(40);
n = 150;
ppi = idinput(n);
X = ppi;
L = n;             % Length of signal

Fs = 100;            % Sampling frequency                    
T = 1/Fs;             % Sampling period       

t = (0:L-1)*T;        % Time vector
figure();
plot(t,X);

Y = fft(X);
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(L/2))/L;
figure();
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')