
% Init Matlab. 
clearvars
close all
clc

% Init element spacing. 
fc = 10e9; 
c = 3e8; 
lambda = c / fc; 
dx_ana = lambda/2;
dx_dig = 2*lambda; 
dx_comb = dx_ana; 
N_ana = 4; 
N_dig = 2; 
N_comb = 8; 

% Obtain k. 
k = 2*pi/lambda; 

% Define u vector. 
nof_u = 1024; 
u = linspace(-1, 1, nof_u); 

% Obtain beam pattern analog subarray. 
taper_ana = ones(N_ana, 1); 
x_ana = (0:dx_ana:(N_ana-1)*dx_ana).'; 
af_ana = sum(taper_ana.*exp(1j*k*u.*x_ana)); 
af_ana = af_ana ./ max(abs(af_ana));
af_ana_db = 20*log10(abs(af_ana)); 

% Obtain digital weights. 
taper_dig = ones(N_dig, 1); 
digital_steer = 0; 
u_steer = sind(-digital_steer);
steer_vec = [1; exp(1j*k*u_steer*dx_dig)]; 
taper_dig = taper_dig .* steer_vec; 

% Obtain beam pattern digital.
x_dig = (0:dx_dig:(N_dig-1)*dx_dig).'; 
af_dig = sum(taper_dig.*exp(1j*k*u.*x_dig)); 
af_dig = af_dig ./ max(abs(af_dig));
af_dig_db = 20*log10(abs(af_dig));

% Calculate combined array factor. 
af_comb = af_ana .* af_dig; 
af_comb_db = 20*log10(abs(af_comb)); 

% Init p, legend_vec. 
p = [];
legend_vec = {};

% Find y_max. 
y_max = max([af_ana_db af_dig_db af_comb_db]);
y_lim_vec = y_max + [-50 5]; 

% Create figure. 
fig = figure; 
grid on 
hold on
p(1) = plot(u, af_ana_db);
legend_vec{1} = 'Array factor analog subarray'; 
p(2) = plot(u, af_dig_db);
legend_vec{2} = 'Array factor digital'; 
p(3) = plot(u, af_comb_db, 'k');
legend_vec{3} = 'Combined array factor';
xlabel('u [sine]');
ylabel('Amplitude [dB]'); 
title('Combined array factor fixed hybrid subarrays');
legend(p, legend_vec); 
ylim(y_lim_vec);

% Save figure. 
if digital_steer == 0
    file_name = 'no_steer'; 
else 
    file_name = sprintf('steer_%d_degrees', digital_steer);
end 
saveas(fig, [file_name '.emf']); 

