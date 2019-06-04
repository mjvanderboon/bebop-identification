function [ppi, n] = preplannedinput(type) %type, length, amplitude
%% preplannedinput
% Returns preplannedinput (ppi) vector
% Called initially on construction to generate preplanned input vectors    
% Amplitude is [-1,1] for SDK to maxangl
sampleT = .01; %change to average sample time to get inputs of desired frequency

switch type   %cases staan gedefineerd in de drive 
    case 0 %% 0 nothing 
        n = 150;
        ppi = zeros(1,n);
    %%Block wave%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 10     %Long block wave, steady state has to be reached                                    
        A = 1;
        n = 200; 
        ppi = zeros(1,n);
        ppi(1,1:ceil(n/2)) = A;        
    case 11     %Short block wave, long period between the waves. 
        A = 1;
        n = 150; 
        ppi = zeros(1,n);
        ppi(1,1:ceil(n/3)) = A;        
    case 12     %Short block wave, Short perios between the waves.
        A = 1;
        n = 150; 
        ppi = zeros(1,n);
        ppi(1,1:ceil(n/4)) = A;        
    case 13     %Long block wave, steady state has to be reached A = .5.
        A = .5;
        n = 200;
        ppi = zeros(1,n);
        ppi(1,1:ceil(n/2)) = A;        
    case 14     %Short block wave, long period between the waves. A = .5.
        A = .5;
        n = 150; 
        ppi = zeros(1,n);
        ppi(1,1:ceil(n/3)) = A;        
    case 15     %Short block wave, Short perios between the waves.
        A = .5;
        n = 150; 
        ppi = zeros(1,n);
        ppi(1,1:ceil(n/4)) = A;      
    case 16
        A = 1;
        n = 200;
        ppi = zeros(1,n);
        ppi(1,1:ceil(n/7)) = A;
       
    case 17
       A = 1;
       n = 500;
       ppi = zeros(1,n);
       ppi(1,1:ceil(n/2)) = A;
    
    %%Step%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    case 20      %half pause 0, n/2-pause A, pause 0, n/2-pause -A, half pause 0  Om een zo lang mogelijke step te kunnen krijgen.                                   
        A = 1;
        n = 100; 
        pause = 10;
        
        ppi = zeros(1,n);
        ppi(1,1+pause/2:ceil(n/2-pause/2)) = A;
        ppi(1,ceil(n/2+pause/2):n-pause/2) = -A;        
    case 21
        A = .5; %.5
        n = 1300; 
        pause = 200;
        
        ppi = zeros(1,n);
        ppi(1,1+pause/2:ceil(n/2-pause/2)) = A;
        ppi(1,ceil(n/2+pause/2):n-pause/2) = -A;     
    case 22
        A = 1;
        n = 500;         
        
        ppi = zeros(1,n);
        ppi(1,1:ceil(n/2)) = A;
        ppi(1,ceil(n/2)+1:n) = -A;      
    case 23      %half pause 0, n/2-pause A, pause 0, n/2-pause -A, half pause 0  Om een zo lang mogelijke step te kunnen krijgen.                                   
        A = .5;
        n = 1000; 
        pause = 200;
        
        ppi = zeros(1,n);
        ppi(1,1+pause/2:ceil(n/2-pause/2)) = A;
        ppi(1,ceil(n/2+pause/2):n-pause/2) = -A;  
    case 24
        A = 1;
        n = 800;
        
        ppi = zeros(1,n);
        ppi(1,1:ceil(n/2)) = A;
        ppi(1,ceil(n/2)+1:n) = -A;
    case 25      %half pause 0, n/2-pause A, pause 0, n/2-pause -A, half pause 0  Om een zo lang mogelijke step te kunnen krijgen.                                   
        A = 1;
        n = 800; 
        pause = 100;
        
        ppi = zeros(1,n);
        ppi(1,1+pause/2:ceil(n/2-pause/2)) = A;
        ppi(1,ceil(n/2+pause/2):n-pause/2) = -A;
    %%sine wave%%%%%%%%%%%%%%%%%%%%%%%%%%%5
    case 30 
        A = 1;
        n = 150;         
        ppi = A*sin(linspace(1,n,n)/ n * 2*pi);
    case 31
        A = .5;
        n = 500;         
        ppi = A*sin(linspace(1,n,n)/ n * 2*pi);
    case 32
        A = 1;
        n = 150;         
        ppi = A*sin(linspace(1,n,n)/ n * 2*pi);
    case 33
        A = 0.5;
        n = 600;         
        ppi = A*sin(linspace(1,n,n)/ n * 2*pi);
        
    %% Full blockwave (van +A naar -A)
    case 40
        A = 1;
        n = 800;
        ppi = zeros(1,n);
        ppi(1,1:ceil(n/2)) = A;
        ppi(1,ceil(n/2)+1:n) = -A;
    case 41
        A = .5;
        n = 250;
        ppi = zeros(1,n);
        ppi(1,1:ceil(n/2)) = A;
        ppi(1,ceil(n/2)+1:n) = -A;
    case 50 %sinusgolf met lange pauze erna
        A = 1;
        n = 130;
        ppi = zeros(1,4*n);
        ppi(1,1:ceil(n/2)) = A;
        ppi(1,ceil(n/2)+1:n) = -A;
        n = 4*n;
    case 51
        A = 1;
        n = 130;
        ppi = zeros(1,4*n);
        ppi(1,2*n:2*n+ceil(n/2)) = A;
        ppi(1,2*n+ceil(n/2)+1 : 3*n) = -A;  
        n = 4*n;
    case 60 %sinusgolf met lange pauze erna
        A = 1;
        n = 130;
        ppi = zeros(1,4*n);
        ppi(1,1:n) = A*sin(linspace(1,n,n)/ n * 2*pi);        
        n = 4*n;
    case 61
        A = 1;
        n = 130;
        ppi = zeros(1,4*n);
        ppi(1,2*n+1:3*n) = A*sin(linspace(1,n,n)/ n * 2*pi);        
        n = 4*n;
    case 62
        A = 1;        
        n = 300;
        ppi = zeros(1,200);
        ppi(1,1:49) = 0.001; 
        ppi(1,50:300) = 1;
    case 63
        freq = 2; %desired frequency in Hertz
        
        n = 1/(freq*sampleT);
        ppi = zeros(1,n);
        ppi(1,1:ceil(n/2)) = .5;
        ppi(1,ceil(n/2)+1:n) = -.5;
    case 64
        freq = .5; %desired frequency in Hertz
        
        n = 1/(freq*sampleT);
        ppi = zeros(1,n);
        ppi(1,1:ceil(n/2)) = .5;
        ppi(1,ceil(n/2)+1:n) = -.5;
    case 65
        freq = .25; %desired frequency in Hertz
        
        n = 1/(freq*sampleT);
        ppi = zeros(1,n);
        ppi(1,1:ceil(n/2)) = .5;
        ppi(1,ceil(n/2)+1:n) = -.5;
    case 66
        ppi = zeros(1,1200);
        ppi(1,1:100) = .5;
        ppi(1,101:200) = -.5;
        ppi(1,900:1000) = .5;
        ppi(1,1001:1100) = -.5;                   
    case 67
        ppi = zeros(1,1200);
        ppi(1,300:400) = .5;
        ppi(1,401:500) = -.5;
        ppi(1,900:1000) = .5;
        ppi(1,1001:1100) = -.5;                   
    case 68
        ppi = zeros(1,1200);
        ppi(1,600:700) = .5;
        ppi(1,700:800) = -.5;
        ppi(1,900:1000) = .5;
        ppi(1,1001:1100) = -.5;    
    case 69
        A = 1;        
        n = 300;
        ppi = zeros(1,n);
        ppi(1,:) = .0001;
end 
