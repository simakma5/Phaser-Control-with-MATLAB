%% MIMO Radar Summer School 2026 - Exercise 1: Contiguous Virtual ULA
% 
% Objective:
% Demonstrate spatial convolution and virtual array synthesis using:
%   - Physical Rx: 8-element patch ULA with d_Rx = lambda/2
%   - Physical Tx: 2 Vivaldi antennas spaced d_Tx = 4*lambda = 8*d_Rx apart
%   - Synthesized Virtual Array: 16-element contiguous ULA with d = lambda/2
%
% This doubles the effective aperture (3.5*lambda -> 7.5*lambda), halving the
% 3dB beamwidth (~14.5 deg -> ~6.8 deg) and resolving two closely spaced targets
% that are unresolved by the physical receiver alone.
%
% Supported Modes:
%   'simulated'  : Pure MATLAB simulation (ideal for testing without hardware)
%   'electronic' : Hardware acquisition with software RF switching (bf.EnableOut1)
%   'manual'     : Hardware acquisition with manual single-Tx repositioning on rail
%
% Copyright 2026 Microwave Sensing, Signals and Systems (MS3), TU Delft.

clear; close all; clc;
warning('off','MATLAB:system:ObsoleteSystemObjectMixin');

%% 1. Configuration & Mode Selection
% =========================================================================
% Select mode: 'simulated' | 'electronic' | 'manual'
mode = 'simulated'; 

% Radar Carrier & Geometry
fc = 10e9;                     % 10 GHz carrier frequency
c = physconst('LightSpeed');   % Speed of light (m/s)
lambda = c / fc;               % Wavelength (~0.03 m = 3 cm)

% Array Geometry
nRx = 8;                       % Number of physical Rx elements
dRx = lambda / 2;              % Physical Rx element spacing (1.5 cm)
nTx = 2;                       % Number of physical Tx positions
dTx = 4 * lambda;              % Tx separation = 4*lambda = 8*dRx (12 cm)

% Physical Element Coordinates (along x-axis)
x_rx = (0:(nRx - 1)) * dRx;    % [0, 0.5, 1.0, ..., 3.5] * lambda
x_tx = [0, dTx];               % [0, 4.0] * lambda

% Virtual Array Coordinates via Spatial Convolution: r_virt = r_tx (+) r_rx
x_virt = [x_tx(1) + x_rx, x_tx(2) + x_rx]; 
nVirt = length(x_virt);        % 16 contiguous elements

fprintf('=========================================================\n');
fprintf(' MIMO Radar Exercise 1: Contiguous Virtual ULA Synthesis\n');
fprintf(' Mode: %s\n', upper(mode));
fprintf(' Carrier: %.2f GHz (Wavelength: %.2f cm)\n', fc/1e9, lambda*100);
fprintf(' Physical Rx: %d elements, Aperture: %.2f lambda\n', nRx, max(x_rx)/lambda);
fprintf(' Physical Tx: %d elements, Separation: %.2f lambda\n', nTx, dTx/lambda);
fprintf(' Virtual Array: %d elements, Aperture: %.2f lambda\n', nVirt, max(x_virt)/lambda);
fprintf('=========================================================\n\n');

%% 2. Data Acquisition (Hardware or Simulation)
% =========================================================================
nSnapshots = 64;               % Number of temporal snapshots / pulses

if strcmp(mode, 'simulated')
    % ---------------------------------------------------------------------
    % Simulation Scene: Two closely-spaced targets at boresight
    % Angular separation: 9 deg (-4.5 deg and +4.5 deg)
    % Physical 8-element 3dB beamwidth is ~14.5 deg -> UNRESOLVED
    % Virtual 16-element 3dB beamwidth is ~6.8 deg -> RESOLVED
    % ---------------------------------------------------------------------
    theta_targets = [-4.5, 4.5];  % Target angles in azimuth (degrees)
    snr_db = 20;                  % Signal-to-Noise Ratio (dB)
    
    fprintf('Simulating two targets at angles: [%.1f, %.1f] degrees...\n', ...
            theta_targets(1), theta_targets(2));
    
    % Steering vectors: a_rx(theta) in C^(8x1) and a_virt(theta) in C^(16x1)
    k_wavenum = 2 * pi / lambda;
    a_rx1 = exp(-1j * k_wavenum * x_rx.' * sind(theta_targets(1)));
    a_rx2 = exp(-1j * k_wavenum * x_rx.' * sind(theta_targets(2)));
    
    a_virt1 = exp(-1j * k_wavenum * x_virt.' * sind(theta_targets(1)));
    a_virt2 = exp(-1j * k_wavenum * x_virt.' * sind(theta_targets(2)));
    
    % Random complex amplitudes across snapshots (uncorrelated signals)
    s1 = (randn(1, nSnapshots) + 1j * randn(1, nSnapshots)) / sqrt(2);
    s2 = (randn(1, nSnapshots) + 1j * randn(1, nSnapshots)) / sqrt(2);
    
    % Clean signals
    y_virt_clean = a_virt1 * s1 + a_virt2 * s2;
    
    % Add complex AWGN
    noise_power = 10^(-snr_db / 10);
    noise = sqrt(noise_power / 2) * (randn(nVirt, nSnapshots) + 1j * randn(nVirt, nSnapshots));
    y_virt = y_virt_clean + noise;
    
    % The physical Rx array corresponds to Tx 1 (first 8 virtual elements)
    y_rx_meas = y_virt(1:nRx, :);

else
    % ---------------------------------------------------------------------
    % Hardware Execution (ADALM-PHASER + PlutoSDR)
    % ---------------------------------------------------------------------
    fprintf('Initializing ADALM-PHASER hardware...\n');
    
    % Setup radar waveform parameters
    prf = 2000;
    nPulses = nSnapshots;
    fs = 1e6;
    rampbandwidth = 500e6;
    
    % Load calibration weights
    if exist('CalibrationWeights.mat', 'file')
        calibrationweights = loadCalibrationWeights();
    else
        warning('Calibration weights not found. Using unity weights.');
        calibrationweights.AnalogWeights = ones(4, 2);
        calibrationweights.DigitalWeights = [1; 1];
    end
    
    % Initialize Pluto and Phaser hardware
    [rx, tx, bf, bf_TDD] = setupLabRadar(fc, prf, nPulses, fs, rampbandwidth);
    
    % Storage for extracted snapshots from Tx 1 and Tx 2
    y_rx1 = zeros(nRx, nSnapshots);
    y_rx2 = zeros(nRx, nSnapshots);
    
    % Loop over the two transmit configurations
    for iTx = 1:2
        fprintf('\n--- Preparing Transmission for Tx %d (x = %.2f lambda) ---\n', ...
                iTx, x_tx(iTx)/lambda);
        
        if strcmp(mode, 'electronic')
            % Digital RF Switch: true -> SMA Out 1, false -> SMA Out 2
            if iTx == 1
                bf.EnableOut1 = true;
                fprintf('RF switch: Out 1 active (Tx 1).\n');
            else
                bf.EnableOut1 = false;
                fprintf('RF switch: Out 2 active (Tx 2).\n');
            end
            pause(0.1); % Settle switch
            
        elseif strcmp(mode, 'manual')
            if iTx == 1
                input('Set Tx antenna at Position 1 (x = 0). Press Enter to capture...');
            else
                input('Move Tx antenna to Position 2 (x = 4*lambda = 12 cm). Press Enter to capture...');
            end
        end
        
        % Sequential 8-Element Rx Readout across ADAR1000 chips
        % ADAR1000_1: Channels 1-4 (Pluto Rx1)
        % ADAR1000_2: Channels 5-8 (Pluto Rx2)
        rx_raw_8ch = zeros(rx.SamplesPerFrame, nPulses, nRx);
        
        for ch = 1:4
            % Power down all 8 channels
            bf.RxPowerDown(:) = 1;
            % Enable channel ch on Chip 1 and ch+4 on Chip 2
            bf.RxPowerDown(ch) = 0;
            bf.RxPowerDown(ch + 4) = 0;
            bf.LatchRxSettings();
            
            % Capture coherent burst
            data = captureTransmitWaveform(rx, tx, bf);
            data = arrangePulseData(data, rx, bf, bf_TDD);
            
            % Store isolated channel responses
            rx_raw_8ch(:, :, ch)     = data(:, :, 2); % Pluto Rx1
            rx_raw_8ch(:, :, ch + 4) = data(:, :, 1); % Pluto Rx2
        end
        
        % Restore all channels
        bf.RxPowerDown(:) = 0;
        bf.LatchRxSettings();
        
        % Range Processing: Extract target peak complex amplitude
        % Subtract clutter across pulses
        rx_mti = rx_raw_8ch - mean(rx_raw_8ch, 2);
        
        % Range FFT across fast-time dimension
        range_fft = fft(rx_mti, [], 1);
        range_prof = squeeze(mean(abs(range_fft(:, :, :)), [2, 3]));
        [~, target_bin] = max(range_prof(1:floor(end/2)));
        
        % Extract complex snapshot across pulses at target range bin
        for el = 1:nRx
            if iTx == 1
                y_rx1(el, :) = range_fft(target_bin, :, el);
            else
                y_rx2(el, :) = range_fft(target_bin, :, el);
            end
        end
    end
    
    % Clean up hardware triggers
    disableTddTrigger(bf_TDD);
    
    % Combine the two 8-element snapshots into the 16-element virtual vector
    y_rx_meas = y_rx1;
    y_virt = [y_rx1; y_rx2];
end

%% 3. Spatial Covariance & Forward-Backward Averaging
% =========================================================================
% Sample Covariance for physical 8-element array
R_rx = (y_rx_meas * y_rx_meas') / nSnapshots;

% Sample Covariance for virtual 16-element array
R_virt = (y_virt * y_virt') / nSnapshots;

% Forward-Backward Spatial Smoothing (enhances resolution for coherent returns)
J_rx = fliplr(eye(nRx));
R_rx_fb = 0.5 * (R_rx + J_rx * conj(R_rx) * J_rx);

J_virt = fliplr(eye(nVirt));
R_virt_fb = 0.5 * (R_virt + J_virt * conj(R_virt) * J_virt);

%% 4. Spatial Spectrum Estimation (Bartlett, MVDR, MUSIC)
% =========================================================================
scanAngles = -90:0.2:90;        % Azimuth angle grid (degrees)
nAngles = length(scanAngles);

% Pre-allocate spatial spectrum arrays
P_bartlett_rx   = zeros(1, nAngles);
P_mvdr_rx       = zeros(1, nAngles);
P_music_rx      = zeros(1, nAngles);

P_bartlett_virt = zeros(1, nAngles);
P_mvdr_virt     = zeros(1, nAngles);
P_music_virt    = zeros(1, nAngles);

% Eigendecomposition for MUSIC (assuming 2 signal sources)
nSources = 2;

% Physical Rx Eigendecomposition
[V_rx, D_rx] = eig(R_rx_fb);
[~, idx_rx] = sort(diag(D_rx), 'descend');
V_rx = V_rx(:, idx_rx);
En_rx = V_rx(:, (nSources + 1):end); % Noise subspace (dim: 8 x 6)

% Virtual Array Eigendecomposition
[V_virt, D_virt] = eig(R_virt_fb);
[~, idx_virt] = sort(diag(D_virt), 'descend');
V_virt = V_virt(:, idx_virt);
En_virt = V_virt(:, (nSources + 1):end); % Noise subspace (dim: 16 x 14)

% Regularized inverses for MVDR (Capon)
invR_rx   = pinv(R_rx_fb + 1e-3 * trace(R_rx_fb) * eye(nRx));
invR_virt = pinv(R_virt_fb + 1e-3 * trace(R_virt_fb) * eye(nVirt));

k_wave = 2 * pi / lambda;

for iA = 1:nAngles
    ang = scanAngles(iA);
    
    % Steering vector for physical Rx array (8x1)
    a_rx = exp(-1j * k_wave * x_rx.' * sind(ang));
    
    % Steering vector for virtual 16-element array (16x1)
    a_virt = exp(-1j * k_wave * x_virt.' * sind(ang));
    
    % --- Physical 8-element Array Spectra ---
    % Bartlett
    P_bartlett_rx(iA) = real(a_rx' * R_rx_fb * a_rx) / (a_rx' * a_rx);
    % MVDR (Capon)
    P_mvdr_rx(iA)     = 1 / real(a_rx' * invR_rx * a_rx);
    % MUSIC
    P_music_rx(iA)    = 1 / real(a_rx' * (En_rx * En_rx') * a_rx);
    
    % --- Virtual 16-element Array Spectra ---
    % Bartlett
    P_bartlett_virt(iA) = real(a_virt' * R_virt_fb * a_virt) / (a_virt' * a_virt);
    % MVDR (Capon)
    P_mvdr_virt(iA)     = 1 / real(a_virt' * invR_virt * a_virt);
    % MUSIC
    P_music_virt(iA)    = 1 / real(a_virt' * (En_virt * En_virt') * a_virt);
end

% Normalize spectra to 0 dB peak
norm_db = @(p) 10 * log10(p / max(p));
P_bartlett_rx_db   = norm_db(P_bartlett_rx);
P_mvdr_rx_db       = norm_db(P_mvdr_rx);
P_music_rx_db      = norm_db(P_music_rx);

P_bartlett_virt_db = norm_db(P_bartlett_virt);
P_mvdr_virt_db     = norm_db(P_mvdr_virt);
P_music_virt_db    = norm_db(P_music_virt);

%% 5. Visualization
% =========================================================================

% --- Figure 1: Array Geometry & Virtual Synthesis ---
figure('Name', 'MIMO Array Geometry', 'Position', [80, 100, 950, 480]);

subplot(3, 1, 1);
stem(x_rx / lambda, ones(1, nRx), 'filled', 'LineWidth', 1.5, 'Color', [0 0.447 0.741]);
xlim([-1, 9]); ylim([0, 1.5]); grid on;
title('Physical Rx Array (8 elements, d = \lambda/2)');
xlabel('Position along baseline (\lambda)'); ylabel('Active');
set(gca, 'YTick', [0, 1]);

subplot(3, 1, 2);
stem(x_tx / lambda, ones(1, nTx), 'filled', 'LineWidth', 1.5, 'Color', [0.85 0.325 0.098]);
xlim([-1, 9]); ylim([0, 1.5]); grid on;
title('Physical Tx Positions (2 Vivaldi antennas, d_{Tx} = 4\lambda = 8\cdot d_{Rx})');
xlabel('Position along baseline (\lambda)'); ylabel('Active');
set(gca, 'YTick', [0, 1]);

subplot(3, 1, 3);
stem(x_virt / lambda, ones(1, nVirt), 'filled', 'LineWidth', 1.5, 'Color', [0.466 0.674 0.188]);
xlim([-1, 9]); ylim([0, 1.5]); grid on;
title('Synthesized Virtual Array (16 elements contiguous, d = \lambda/2)');
xlabel('Position along baseline (\lambda)'); ylabel('Active');
set(gca, 'YTick', [0, 1]);

% --- Figure 2: Comparative Spatial Spectra ---
figure('Name', 'MIMO Resolution Comparison', 'Position', [120, 150, 1100, 520]);

% Subplot 1: Physical Rx (8 elements)
subplot(1, 2, 1);
plot(scanAngles, P_bartlett_rx_db, 'LineWidth', 1.8, 'DisplayName', 'Bartlett'); hold on;
plot(scanAngles, P_mvdr_rx_db,     'LineWidth', 1.8, 'DisplayName', 'Capon (MVDR)');
plot(scanAngles, P_music_rx_db,    'LineWidth', 1.8, 'DisplayName', 'MUSIC');
grid on; xlim([-30, 30]); ylim([-35, 2]);
xlabel('Azimuth Angle (degrees)'); ylabel('Normalized Power (dB)');
title(sprintf('Physical 8-Element Rx Array\n(Aperture: 3.5\\lambda, Unresolved)'));
legend('Location', 'south');
if strcmp(mode, 'simulated')
    xline(theta_targets(1), '--k', 'Target 1');
    xline(theta_targets(2), '--k', 'Target 2');
end

% Subplot 2: Virtual Array (16 elements)
subplot(1, 2, 2);
plot(scanAngles, P_bartlett_virt_db, 'LineWidth', 1.8, 'DisplayName', 'Bartlett (Virtual)'); hold on;
plot(scanAngles, P_mvdr_virt_db,     'LineWidth', 1.8, 'DisplayName', 'Capon (MVDR, Virtual)');
plot(scanAngles, P_music_virt_db,    'LineWidth', 1.8, 'DisplayName', 'MUSIC (Virtual)');
grid on; xlim([-30, 30]); ylim([-35, 2]);
xlabel('Azimuth Angle (degrees)'); ylabel('Normalized Power (dB)');
title(sprintf('Synthesized 16-Element Virtual ULA\n(Aperture: 7.5\\lambda, Sharply Resolved!)'));
legend('Location', 'south');
if strcmp(mode, 'simulated')
    xline(theta_targets(1), '--k', 'Target 1');
    xline(theta_targets(2), '--k', 'Target 2');
end

fprintf('\nExecution complete. Inspect the generated figures to evaluate MIMO resolution gain.\n');
