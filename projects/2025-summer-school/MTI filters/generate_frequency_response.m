% Init Matlab. 
clearvars
close all
clc 

% Init MTI filter. 
mti_vec = {[1 -1], [1 -2 1], taylorwin(40, 4, -60).'.*exp(2*pi*1i*0.27*(0:39))}; % prf = 250 Hz. 
name = {'Order 1', 'Order 2', 'Taylor window'}; 
nof_mti = length(mti_vec); 
% N = length(mti); 

% Init p, legend_vec. 
p = [];
legend_vec = {}; 

% Init figure. 
fig = figure;

for i_mti = 1:nof_mti

    % Select mti filter. 
    mti = mti_vec{i_mti}; 

    % Create the frequency response. 
    Nfft = 1024; 
    mti_fft = fftshift(fft(mti, Nfft)); 
    mti_fft_db = 20*log10(abs(mti_fft));
    
    % Determine frequency vector. 
    fs = 1; 
    df = fs / Nfft; 
    f = -fs/2 : df : fs/2-df; 
    
    % Plot the frequency response in dB    
    hold on
    p(i_mti) = plot(f, mti_fft_db);
    legend_vec{i_mti} = name{i_mti}; 
    legend(p, legend_vec); 
    xlabel('Frequency (Hz)');
    ylabel('Magnitude (dB)');
    title('MTI Filter Frequency Response');
    grid on;

end 

% Save figure. 
% file_name = sprintf('order_%d', N-1);
file_name = 'Comparison MTI filters'; 
saveas(fig, [file_name '.emf'])
fprintf('%s saved.\n', file_name);

