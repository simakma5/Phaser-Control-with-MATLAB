
% Init Matlab. 
clearvars
close all
clc 

% Init element positions. 
fc = 10e9; 
c = 3e8; 
lambda = c/fc; 
dx = lambda/2; 
Nx = 8; 
x = 0:dx:(Nx-1)*dx;  

% Init ideal taper. 
taper_init = taylorwin(Nx, 2, -50).'; 

% Apply a scan to the taper. 
u_scan = sind(30); 
k = 2*pi / lambda; 
taper_init = taper_init.*exp(1j*k*u_scan*x);

% Set maximum to 0 dB. 
taper_init = taper_init / max(abs(taper_init)); 

% Init different quantization levels. 
lsb_att = [0.5 1 2 4 8 16]; 
nof_bits = [7 6 5 4 3 2]; 
lsb_phase = 2*pi ./ 2.^nof_bits; 
nof_q = length(lsb_att); 

% Init figure. 
fig = figure; 
hold on 
grid on 

% Init p, legend_vec. 
p = []; 
legend_vec = {}; 

% Init u. 
nof_u = 1024;
u = linspace(-1, 1, nof_u); 

% Loop over the different quantization levels. 
for i_q = 1:nof_q

    % Init taper again. 
    taper = taper_init; 

    % Select relevant quantization values. 
    lsb_att_sel = lsb_att(i_q);
    lsb_phase_sel = lsb_phase(i_q); 

    % Quantize the taper. 
    taper_angle = angle(taper); 
    taper_amp_db = 20*log10(abs(taper));            
    taper_angle = round(taper_angle / lsb_phase_sel) * lsb_phase_sel; 
    taper_amp_db_q = round(taper_amp_db / lsb_att_sel) * lsb_att_sel; 
    taper_amp_q = 10.^(taper_amp_db_q/20);     
    taper = taper_amp_q.*exp(1i*taper_angle); 

    % Calculate the beam pattern for this taper.     
    beam = sum(taper.'.*exp(-1j*k*u.*x.')); 

    % Calculate beam_db. 
    beam_db = 20*log10(abs(beam));
    beam_db = beam_db - max(beam_db); 

    % Add to plot.
    p(i_q) = plot(u, beam_db); 
    legend_vec{i_q} = sprintf('Nof PS bits = %d, lsb att = %.2f dB.', nof_bits(i_q), lsb_att_sel);     

end 

% Add axes and legend. 
legend(p, legend_vec); 
xlabel('u [sine]'); 
ylabel('Amplitude [dB]'); 
title('Beam pattern for different quantization levels.')
ylim([-50 5]); 

% Save the figure. 
file_name = 'quantized_beam_patterns'; 
saveas(fig, [file_name '.emf']); 

