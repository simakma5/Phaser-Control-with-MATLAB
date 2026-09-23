%% FMCW Radar Lab: Micro-Doppler & Range-Doppler
clear; close all;
warning('off','MATLAB:system:ObsoleteSystemObjectMixin')

% 1. System Setup
fc = 10e9;
prf = 2000; % Set to 2000 Hz for fan micro-Doppler
nPulses = 128;
fs = 1e6;
rampbandwidth = 500e6;
c = 3e8; % Speed of light
lambda = c/fc;

[rx,tx,bf,bf_TDD] = setupLabRadar(fc,prf,nPulses,fs,rampbandwidth);

% 2. Setup DSP & Axes
tSweep = double(bf.FrequencyDeviationTime)/1e6;
sweepSlope = rampbandwidth/tSweep;

tCapture = 30;
minRange = 0; maxRange = 10;

figure('Name', 'Radar Displays', 'Position', [100 100 1000 400]);
ax_rd = subplot(1,2,1);
ax_dti = subplot(1,2,2);
history_dop = []; 

max_rd_val = -inf;
max_dti_val = -inf;
dynamic_range = 60; % dB

% Capture initial waveform
captureTransmitWaveform(rx,tx,bf);
t = tic;
while toc(t) < tCapture
    % 3. Capture & Arrange Data
    data = captureTransmitWaveform(rx,tx,bf);
    data = arrangePulseData(data,rx,bf,bf_TDD);
    data = data - mean(data, 2);  % Remove zero-Doppler returns
    
    [N, M] = size(data); % N: fast-time samples, M: slow-time pulses
    
    % --- DSP LOGIC (SOLUTION) ---
    % Apply 2D window
    win = hamming(N) * hamming(M)';
    data_win = data .* win;
    
    % Range FFT (across fast-time)
    range_fft = fft(data_win, [], 1);
    
    % Doppler FFT (across slow-time) and shift
    rd_response = fftshift(fft(range_fft, [], 2), 2);
    
    % Compute Axes
    f_beat = (0:N-1)'/N * fs;
    range = f_beat * c / (2 * sweepSlope);
    
    doppler_freq = (-M/2:M/2-1) / M * prf;
    speed = -doppler_freq * lambda / 2; % Inverted so negative is towards radar
    % ---------------------------
    
    % Plot Range-Doppler
    keepRange = range >= minRange & range <= maxRange;
    rd_db = 20*log10(abs(rd_response(keepRange, :)));
    
    current_max_rd = max(rd_db(:));
    if current_max_rd > max_rd_val
        max_rd_val = current_max_rd;
    end
    
    imagesc(ax_rd, speed, range(keepRange), rd_db);
    axis(ax_rd, 'xy');
    xlabel(ax_rd, 'Speed (m/s)'); ylabel(ax_rd, 'Range (m)');
    title(ax_rd, 'Range-Doppler Response');
    colorbar(ax_rd);
    caxis(ax_rd, [max_rd_val - dynamic_range, max_rd_val]);
    
    % Doppler-Time extraction
    % Project the Range-Doppler map onto the Doppler axis by taking the maximum across ranges
    dop_profile = max(abs(rd_response(keepRange, :)), [], 1);
    history_dop = [history_dop; dop_profile];
    dti_db = 20*log10(history_dop);
    
    current_max_dti = max(dti_db(:));
    if current_max_dti > max_dti_val
        max_dti_val = current_max_dti;
    end
    
    % Plot Doppler-Time
    imagesc(ax_dti, speed, 1:size(history_dop,1), dti_db);
    axis(ax_dti, 'xy');
    xlabel(ax_dti, 'Speed (m/s)'); ylabel(ax_dti, 'Time Step');
    title(ax_dti, 'Doppler-Time (Micro-Doppler)');
    colorbar(ax_dti);
    caxis(ax_dti, [max_dti_val - dynamic_range, max_dti_val]);
    
    drawnow;
end

% Disable TDD Trigger
cleanupAntenna(rx,tx,bf,bf_TDD);
