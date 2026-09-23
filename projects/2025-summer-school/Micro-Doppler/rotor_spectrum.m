
% Init matlab. 
clearvars
close all
clc 

% Init frequency variables. 
frotor = 16; 
prf = 2000; 
Np = 128; 
Nb = 1; 

% Received signal for 1 rotor, after pulse compression. 
r = zeros(1, Np); 
for p = 1:Np
    for b = 1:Nb
        r(p) = r(p) + exp(1j*pi*sin(2*pi*p*frotor/prf + (b-1)*pi/2)); 
    end 
end 

% Perform the fft of r. 
R = fft(r, 1024); 
R_db = 20*log10(abs(R)); 

% Define frequency axis for plotting
f = (0:length(R_db)-1) * (prf / length(R_db));
c = 3e8; 
fc = 10e9; 
v = f*c/2/fc; 

% plot phase of received pulses. 
fig = figure; 
grid on 
plot(1:Np, angle(r)*180/pi); 
xlabel('Pulse');
ylabel('Received phase [deg]'); 

% Plot result. 
fig = figure; 
grid on 
plot(v, R_db); 
xlabel('Velocity [m/s]'); 
ylabel('Amplitude [dB]');

