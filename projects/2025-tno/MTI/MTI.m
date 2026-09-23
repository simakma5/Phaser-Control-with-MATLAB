%% FMCW Radar Lab: Moving Target Indicator (MTI) Filtering
clear; close all;
warning('off','MATLAB:system:ObsoleteSystemObjectMixin');

% 1. System Setup
fc = 10e9;
prf = 250;
nPulses = 64;
fs = 1e6;
rampbandwidth = 500e6;
c = 3e8; % Speed of light
lambda = c/fc;

[rx,tx,bf,bf_TDD] = setupLabRadar(fc,prf,nPulses,fs,rampbandwidth);

% =========================================================================
% 2. MTI Filter Design & Analysis
% =========================================================================

% --- DSP LOGIC (SOLUTION) ---
% Question 1 & 2: Theoretical Responses
mti_2pulse = [1 -1];
mti_3pulse = [1 -2 1];

figure('Name', 'MTI Filter Responses', 'Position', [100 100 800 400]);
[h2, f2] = freqz(mti_2pulse, 1, 1024, prf);
[h3, f3] = freqz(mti_3pulse, 1, 1024, prf);
plot(f2, 20*log10(abs(h2)), 'LineWidth', 1.5); hold on;
plot(f3, 20*log10(abs(h3)), 'LineWidth', 1.5);

% Question 4: Design an MTI filter that passes 1 m/s with 50 dB suppression
v_target = 1; 
fd_target = 2 * v_target / lambda;

% Design using fir1 with order 39 and Taylor window for 50dB suppression
filter_order = 39; 
normalized_fd = fd_target / (prf/2);
bw = 0.05; % passband width
% Bandpass filter centered at 1 m/s
b_custom = fir1(filter_order, [max(0.01, normalized_fd-bw) min(0.99, normalized_fd+bw)], 'bandpass', taylorwin(filter_order+1, 4, -50));

[h_cust, f_cust] = freqz(b_custom, 1, 1024, prf);
plot(f_cust, 20*log10(abs(h_cust)), 'LineWidth', 1.5);

xlabel('Doppler Frequency (Hz)'); ylabel('Magnitude (dB)');
title('Theoretical Frequency Response of MTI filters');
grid on; ylim([-60 10]);
legend('2-Pulse', '3-Pulse', 'Custom 1m/s Filter (-50dB)');
% ---------------------------

% Select which filter to use for the actual radar processing below
mti_coeff = mti_3pulse; % Set to b_custom to test the 1m/s specific filter
ncoeff = length(mti_coeff);

% Toggle MTI filtering on and off
applyFilter = true;

% =========================================================================
% 3. Radar Data Collection and Processing
% =========================================================================
tSweep = double(bf.FrequencyDeviationTime)/1e6;
sweepSlope = rampbandwidth/tSweep;

tCapture = 30;
minRange = 0; maxRange = 10;

figure('Name', 'Range-Doppler Display', 'Position', [100 100 600 500]);
ax = axes;

max_rd_val = -inf;
dynamic_range = 60; % dB

captureTransmitWaveform(rx,tx,bf);
t = tic;
while toc(t) < tCapture
    data = captureTransmitWaveform(rx,tx,bf);
    data = arrangePulseData(data,rx,bf,bf_TDD);
    
    [N, M] = size(data);
    
    % --- DSP LOGIC (SOLUTION) ---
    if applyFilter
        data_filtered = filter(mti_coeff, 1, data, [], 2);
        data = data_filtered(:, ncoeff:end);
        M = size(data, 2); 
    end
    
    win = hamming(N) * hamming(M)';
    data_win = data .* win;
    
    range_fft = fft(data_win, [], 1);
    rd_response = fftshift(fft(range_fft, [], 2), 2);
    
    f_beat = (0:N-1)'/N * fs;
    range = f_beat * c / (2 * sweepSlope);
    
    doppler_freq = (-M/2:M/2-1) / M * prf;
    speed = -doppler_freq * lambda / 2; % Inverted so negative is towards radar
    % ---------------------------
    
    keepRange = range >= minRange & range <= maxRange;
    rd_db = 20*log10(abs(rd_response(keepRange, :)));
    
    current_max_rd = max(rd_db(:));
    if current_max_rd > max_rd_val
        max_rd_val = current_max_rd;
    end
    
    imagesc(ax, speed, range(keepRange), rd_db);
    axis(ax, 'xy');
    xlabel(ax, 'Speed (m/s)'); ylabel(ax, 'Range (m)');
    if applyFilter
        title(ax, 'Range-Doppler Response (MTI Filter ON)');
    else
        title(ax, 'Range-Doppler Response (MTI Filter OFF)');
    end
    colorbar(ax);
    caxis(ax, [max_rd_val - dynamic_range, max_rd_val]);
    
    drawnow;
end

cleanupAntenna(rx,tx,bf,bf_TDD);
