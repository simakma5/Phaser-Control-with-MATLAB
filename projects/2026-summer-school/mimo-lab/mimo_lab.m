%% MIMO Radar Summer School 2026 - Contiguous Virtual ULA (2x8 MIMO)
%
% Synthesizes a 16-element contiguous virtual uniform linear array (ULA) from 
% an 8-element physical Rx array (ADALM-PHASER) and two electronically switched 
% transmit antennas (SMA Out 1 & Out 2 spaced by 4*lambda).
%
% Workflow:
%   1. Hardware Initialization: setupFMCWRadar with verified TDD Ch2 timing,
%      buffer flush, and calibration.
%   2. Virtual Array Geometry: Computes and visualizes physical Rx, physical Tx,
%      and synthesized virtual 16-element contiguous ULA geometry.
%   3. Hardware Acquisition: Switched single-element capture across Tx 1 & Tx 2
%      (8 rapid bursts total) to collect all 16 virtual channel snapshots.
%   4. Precise Range Measurement: Generates calibrated range profile, identifies
%      target distance, and extracts complex envelopes at target range bin.
%   5. Measured Spatial Spectrum & AOA Detection: Computes measured spatial
%      spectrum via digital beamforming and automatically detects target AOAs.
%   6. Data-Driven Theoretical Simulation: Produces theoretical spatial spectra
%      using the detected target AOAs for direct, clean benchmark comparison.
%   7. Visualizations:
%      - Figure 1: MIMO Array Geometry & Virtual Synthesis (Stem plot)
%      - Figure 2: Calibrated Range Profile (Target distance identification)
%      - Figure 3: Side-by-Side Spatial Spectrum (Simulated vs Measured)
%      - Figure 4: 2D MIMO Range-Azimuth Profile
%      - Figure 5: Individual Tx-to-Rx Channel Range Profiles
%
% Copyright 2026 Microwave Sensing, Signals and Systems (MS3), TU Delft.

clear; close all; clc;
warning('off','MATLAB:system:ObsoleteSystemObjectMixin');

%% 1. Configuration & Parameters
% =========================================================================

% Operating mode:
%   '2x8_switched' - Full 16-element contiguous virtual ULA via sequential element switching
%   '2x2_subarray' - Dual-subarray baseline (colleague's verified 2x2 mode)
arrayMode = '2x8_switched';

% Calibration and Target Gating
calRange        = 1.6;         % Hardware range offset calibration (m)
targetRangeGate = [0.5, 5.0];  % Search interval for reflectors (calibrated m)
portSwitchPause = 0.10;        % Switch settling time (s)
tx_phase_cal    = 0.0;         % Tx cable/switch phase calibration (deg)

% System & Radar Parameters
fc = 10e9;                     % Carrier frequency (10 GHz)
c = physconst('LightSpeed');   % Speed of light (m/s)
lambda = c / fc;               % Wavelength (0.03 m)
maxRange = 10;                 % Maximum instrumented range (m)
rangeResolution = 0.1;         % Desired range resolution (m)
maxSpeed = 5;                  % Max speed for PRF selection (m/s)
speedResolution = 1/2;         % Speed resolution for chirp count (m/s)

% Array Geometry
nRxPhysical = 8;               % Number of physical Rx elements
dRx = lambda / 2;              % Physical Rx spacing (0.015 m)
dTx = 4 * lambda;              % Tx separation = 8 * dRx (0.12 m)

x_rx = (0:(nRxPhysical - 1)) * dRx;       % Physical Rx coordinates (m)
x_tx = [0, dTx];                          % Physical Tx coordinates (m)
x_virt = (0:(2 * nRxPhysical - 1)) * dRx; % Virtual 16-element ULA coordinates (m)
nVirt = length(x_virt);

% Angular Evaluation Grid (Digital Beamforming)
azimuthGrid = -60:0.5:60;      % Azimuth evaluation grid (deg)
nAngles = length(azimuthGrid);
k = 2 * pi / lambda;

%% 2. Figure 1: Virtual Array Geometry & Spatial Convolution
% =========================================================================
% Virtual positions correspond to spatial convolution: r_virt = r_Tx (+) r_Rx

figure('Name', 'MIMO Array Geometry', 'Position', [80, 100, 950, 480]);

subplot(3, 1, 1);
stem(x_rx / lambda, ones(1, nRxPhysical), 'filled', 'LineWidth', 1.5, 'Color', [0 0.447 0.741]);
xlim([-1, 9]); ylim([0, 1.5]); grid on;
title('Physical Rx Array (8 elements, d = \lambda/2)');
xlabel('Position along baseline (\lambda)'); ylabel('Active');
set(gca, 'YTick', [0, 1]);

subplot(3, 1, 2);
stem(x_tx / lambda, ones(1, 2), 'filled', 'LineWidth', 1.5, 'Color', [0.85 0.325 0.098]);
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

%% 3. FMCW Radar Parameter Derivation & Hardware Setup
% =========================================================================
rampBandwidth = ceil(rangeres2bw(rangeResolution) / 1e6) * 1e6;
fmaxdop = speed2dop(2 * maxSpeed, lambda);
prf = 2 * fmaxdop;
nPulses = ceil(2 * maxSpeed / speedResolution);
tpulse = ceil((1 / prf) * 1e3) * 1e-3;
tsweep = getFMCWSweepTime(tpulse, tpulse);
sweepSlope = rampBandwidth / tsweep;
fmaxbeat = sweepSlope * range2time(maxRange);
fs = max(ceil(2 * fmaxbeat), 520834);

fprintf('FMCW bandwidth: %.1f MHz, sweep time: %.3f ms, chirps/CPI: %d\n', ...
    rampBandwidth / 1e6, tsweep * 1e3, nPulses);
fprintf('Initializing ADALM-PHASER and PlutoSDR...\n');

% Load calibration weights
calibrationweights = loadCalibrationWeights();

% Configure Pluto, Phaser, and TDD engine using verified setupFMCWRadar
[rx, tx, bf, bf_TDD] = setupFMCWRadar(fc, fs, tpulse, tsweep, nPulses, rampBandwidth);
cleanupGuard = onCleanup(@() cleanupAntenna(rx, tx, bf, bf_TDD));

amp = 0.9 * 2^15;
txWaveform = amp * ones(rx.SamplesPerFrame, 2);

% Discard first capture to flush Pluto DMA buffer
rx();

% Apply default analog calibration phase shifts to ADAR1000
phaseshifts = wrapTo360(rad2deg(angle(calibrationweights.AnalogWeights)));
bf.RxPhase(:) = [phaseshifts(:, 1)', phaseshifts(:, 2)'];
bf.RxGain(:) = 127;
bf.LatchRxSettings();

%% 4. Hardware Data Acquisition
% =========================================================================

switch arrayMode
    case '2x8_switched'
        fprintf('Executing 2x8 MIMO capture (Switched Single-Element Mode)...\n');
        % 4 element pairs across 2 ADAR1000 chips:
        %   Pair 1: Element 1 (Chip 1) & Element 5 (Chip 2)
        %   Pair 2: Element 2 (Chip 1) & Element 6 (Chip 2)
        %   Pair 3: Element 3 (Chip 1) & Element 7 (Chip 2)
        %   Pair 4: Element 4 (Chip 1) & Element 8 (Chip 2)
        % Pluto Ch 2 = Subarray 1 (Elements 1..4)
        % Pluto Ch 1 = Subarray 2 (Elements 5..8)
        
        nRxChannels = 8;
        rawPulseData = cell(2, 4); % 2 Tx x 4 pairs
        
        for iTx = 1:2
            bf.EnableOut1 = (iTx == 1);
            pause(portSwitchPause);
            
            for nPair = 1:4
                % Power down all elements except current pair
                bf.RxPowerDown(:) = 1;
                bf.RxPowerDown(nPair)     = 0; % Chip 1 (Subarray 1)
                bf.RxPowerDown(nPair + 4) = 0; % Chip 2 (Subarray 2)
                bf.LatchRxSettings();
                pause(0.01);
                
                rawData = captureTransmitWaveform(rx, tx, bf, txWaveform);
                rawPulseData{iTx, nPair} = arrangeMimoPulseData(rawData, rx, bf, bf_TDD);
            end
        end
        
        % Restore all elements
        bf.RxPowerDown(:) = 0;
        bf.LatchRxSettings();
        
        % Assemble into [nFastTime, nPulses, 8, 2] matrix
        nFastTime = size(rawPulseData{1, 1}, 1);
        mimoPulseData = zeros(nFastTime, nPulses, 8, 2, 'like', rawPulseData{1, 1});
        
        for iTx = 1:2
            for nPair = 1:4
                % Pluto Ch 2 is Subarray 1 (elements 1..4)
                mimoPulseData(:, :, nPair, iTx)     = rawPulseData{iTx, nPair}(:, :, 2);
                % Pluto Ch 1 is Subarray 2 (elements 5..8)
                mimoPulseData(:, :, nPair + 4, iTx) = rawPulseData{iTx, nPair}(:, :, 1);
            end
        end
        
    case '2x2_subarray'
        fprintf('Executing 2x2 MIMO capture (Dual-Subarray Mode)...\n');
        nRxChannels = 2;
        bf.RxPowerDown(:) = 0;
        bf.LatchRxSettings();
        
        mimoCaptures = cell(2, 1);
        for iTx = 1:2
            bf.EnableOut1 = (iTx == 1);
            pause(portSwitchPause);
            
            rawData = captureTransmitWaveform(rx, tx, bf, txWaveform);
            mimoCaptures{iTx} = arrangeMimoPulseData(rawData, rx, bf, bf_TDD);
        end
        
        nFastTime = size(mimoCaptures{1}, 1);
        mimoPulseData = zeros(nFastTime, nPulses, 2, 2, 'like', mimoCaptures{1});
        mimoPulseData(:, :, :, 1) = mimoCaptures{1};
        mimoPulseData(:, :, :, 2) = mimoCaptures{2};
end

fprintf('Data acquisition complete.\n\n');

%% 5. Precise Range Measurement & Target Range Identification (Figure 2)
% =========================================================================
nFastTime = size(mimoPulseData, 1);
nFft = 2^nextpow2(nFastTime);
window = localHann(nFastTime);
windowedData = mimoPulseData .* reshape(window, [], 1, 1, 1);

% Range FFT
rangeSpectrum = fft(windowedData, nFft, 1);
rangeSpectrum = rangeSpectrum(1:nFft/2, :, :, :);

% Calibrated Range Axis (accounts for internal radar delay via calRange)
rawRangeAxis = (0:nFft/2 - 1).' * fs / nFft * c / (2 * sweepSlope);
rangeAxis = rawRangeAxis - calRange;

% Coherently integrate across pulses in the CPI
meanRangeSpectrum = squeeze(mean(rangeSpectrum, 2)); % [nRangeBins, nRx, nTx]
meanRangePower = abs(meanRangeSpectrum).^2;

% Locate target range bin by summing power across all channels in targetRangeGate
searchMask = rangeAxis >= targetRangeGate(1) & rangeAxis <= targetRangeGate(2);
searchBins = find(searchMask);
assert(~isempty(searchBins), 'Target range gate contains no range bins.');

combinedPower = sum(sum(meanRangePower, 2), 3);
[~, maxSearchIdx] = max(combinedPower(searchMask));
targetBin = searchBins(maxSearchIdx);
targetRangeMeas = rangeAxis(targetBin);

fprintf('Detected target range: %.2f m (FFT bin %d)\n\n', targetRangeMeas, targetBin);

% Figure 2: Calibrated Range Profile
figure('Name', 'Calibrated Range Profile', 'Position', [80, 150, 650, 380]);
profileSumDb = 10 * log10(max(combinedPower, eps) / max(combinedPower));
plot(rangeAxis, profileSumDb, 'LineWidth', 1.5);
hold on;
xline(targetRangeMeas, '--r', sprintf('Target Range: %.2f m', targetRangeMeas), ...
      'LabelVerticalAlignment', 'bottom', 'LineWidth', 1.2);
grid on; xlim(targetRangeGate); ylim([-40, 5]);
xlabel('Range (m)'); ylabel('Normalized Power (dB)');
title('Calibrated Range Profile');

%% 6. Measured Spatial Spectrum & AOA Detection
% =========================================================================
norm_db = @(p) 10 * log10(max(p, eps) / max(p));

A_rx_grid   = exp(1j * k * x_rx.' * sind(azimuthGrid));     % 8 x nAngles
A_virt_grid = exp(1j * k * x_virt.' * sind(azimuthGrid));   % 16 x nAngles

switch arrayMode
    case '2x8_switched'
        % Extract complex response across all 8 Rx elements for Tx 1 and Tx 2
        H_tx1 = squeeze(meanRangeSpectrum(targetBin, :, 1)).'; % 8 x 1
        H_tx2 = squeeze(meanRangeSpectrum(targetBin, :, 2)).'; % 8 x 1
        H_tx2_cal = H_tx2 * exp(-1j * deg2rad(tx_phase_cal));
        
        % 16-element contiguous virtual array snapshot
        H_virt = [H_tx1; H_tx2_cal]; % 16 x 1
        
        % Digital Beamforming (Bartlett Spatial Spectrum)
        P_rx_meas   = abs(A_rx_grid' * H_tx1).^2;
        P_virt_meas = abs(A_virt_grid' * H_virt).^2;
        
        P_rx_meas_db   = norm_db(P_rx_meas);
        P_virt_meas_db = norm_db(P_virt_meas);
        
        % Automatic AOA Detection from measured 16-element virtual spectrum
        [detectedAOAs, detectedPowers] = findSpatialPeaks(P_virt_meas_db, azimuthGrid, 2, 3.0);
        
    case '2x2_subarray'
        H_sub = squeeze(meanRangeSpectrum(targetBin, :, :)); % 2 Rx x 2 Tx
        h_vec = H_sub(:); % 4 x 1
        
        rxSubSpacing = 4 * dRx;
        txSubSpacing = 4 * dRx;
        rxPosSub = ((0:1) - 0.5) * rxSubSpacing;
        txPosSub = ((0:1) - 0.5) * txSubSpacing;
        
        virtPosSub = zeros(4, 1);
        idx = 0;
        for iTx = 1:2
            for iRx = 1:2
                idx = idx + 1;
                virtPosSub(idx) = txPosSub(iTx) + rxPosSub(iRx);
            end
        end
        
        A_sub_grid = exp(1j * k * virtPosSub * sind(azimuthGrid));
        P_virt_meas = abs(A_sub_grid' * h_vec).^2;
        P_virt_meas_db = norm_db(P_virt_meas);
        P_rx_meas_db = P_virt_meas_db;
        
        [detectedAOAs, detectedPowers] = findSpatialPeaks(P_virt_meas_db, azimuthGrid, 2, 3.0);
end

if isempty(detectedAOAs)
    detectedAOAs = 0.0;
    detectedPowers = 0.0;
end

fprintf('Detected Target AOAs: %s deg\n\n', mat2str(round(detectedAOAs, 1)));

%% 7. Data-Driven Theoretical Simulated Spatial Spectrum
% =========================================================================
% Synthesize theoretical response using the exact detected AOAs and relative powers

detectedAmps = 10.^(detectedPowers / 20);
detectedAmps = detectedAmps / max(detectedAmps);

s_rx_sim   = zeros(nRxPhysical, 1);
s_virt_sim = zeros(nVirt, 1);

for iT = 1:length(detectedAOAs)
    theta_t = detectedAOAs(iT);
    amp_t   = detectedAmps(iT);
    
    a_rx_t   = exp(1j * k * x_rx.' * sind(theta_t));
    a_virt_t = exp(1j * k * x_virt.' * sind(theta_t));
    
    s_rx_sim   = s_rx_sim + amp_t * a_rx_t;
    s_virt_sim = s_virt_sim + amp_t * a_virt_t;
end

P_rx_sim   = abs(s_rx_sim.' * conj(A_rx_grid)).^2;
P_virt_sim = abs(s_virt_sim.' * conj(A_virt_grid)).^2;

P_rx_sim_db   = norm_db(P_rx_sim);
P_virt_sim_db = norm_db(P_virt_sim);

%% 8. Figure 3: Side-by-Side Spatial Spectrum Comparison
% =========================================================================

figure('Name', 'Spatial Spectrum Comparison', 'Position', [120, 150, 1150, 480]);

% Left Subplot: Simulated Spatial Spectrum (Driven by Detected AOAs)
subplot(1, 2, 1);
plot(azimuthGrid, P_rx_sim_db,   'LineWidth', 1.8, 'DisplayName', 'Physical Rx (8 elements, 3.5\lambda)'); hold on;
plot(azimuthGrid, P_virt_sim_db, 'LineWidth', 1.8, 'DisplayName', 'Virtual Array (16 elements, 7.5\lambda)');
for iT = 1:length(detectedAOAs)
    xline(detectedAOAs(iT), ':k', sprintf('%.1f^\\circ', detectedAOAs(iT)), ...
          'LabelVerticalAlignment', 'top', 'HandleVisibility', 'off');
end
grid on; xlim([min(azimuthGrid), max(azimuthGrid)]); ylim([-35, 2]);
xlabel('Azimuth Angle (deg)'); ylabel('Normalized Power (dB)');
title(sprintf('Simulated Spectrum (AOAs: %s^\\circ)', mat2str(round(detectedAOAs, 1))));
legend('Location', 'south');

% Right Subplot: Measured Spatial Spectrum
subplot(1, 2, 2);
plot(azimuthGrid, P_rx_meas_db,   'LineWidth', 1.8, 'DisplayName', 'Physical Rx (8 elements, 3.5\lambda)'); hold on;
plot(azimuthGrid, P_virt_meas_db, 'LineWidth', 1.8, 'DisplayName', 'Virtual Array (16 elements, 7.5\lambda)');
for iT = 1:length(detectedAOAs)
    xline(detectedAOAs(iT), ':k', sprintf('%.1f^\\circ', detectedAOAs(iT)), ...
          'LabelVerticalAlignment', 'top', 'HandleVisibility', 'off');
end
grid on; xlim([min(azimuthGrid), max(azimuthGrid)]); ylim([-35, 2]);
xlabel('Azimuth Angle (deg)'); ylabel('Normalized Power (dB)');
title('Measured Spatial Spectrum');
legend('Location', 'south');

%% 9. Figure 4: 2D MIMO Range-Azimuth Map
% =========================================================================

if strcmp(arrayMode, '2x8_switched')
    channelRangeSpectrum = zeros(numel(rangeAxis), 16);
    channelRangeSpectrum(:, 1:8)  = squeeze(meanRangeSpectrum(:, :, 1));
    channelRangeSpectrum(:, 9:16) = squeeze(meanRangeSpectrum(:, :, 2)) * exp(-1j * deg2rad(tx_phase_cal));
    
    rangeAzimuth = abs(channelRangeSpectrum * conj(A_virt_grid)); % [nRange x nAngles]
    rangeAzimuthDb = 20 * log10(max(rangeAzimuth, eps) / max(rangeAzimuth, [], 'all'));
    
    plotMask = rangeAxis >= targetRangeGate(1) & rangeAxis <= targetRangeGate(2);
    
    figure('Name', 'Range-Azimuth Profile', 'Position', [160, 150, 700, 480]);
    imagesc(azimuthGrid, rangeAxis(plotMask), rangeAzimuthDb(plotMask, :));
    set(gca, 'YDir', 'normal');
    colormap(jet);
    cb = colorbar; cb.Label.String = 'Relative Power (dB)';
    caxis([-30, 0]);
    xlabel('Azimuth Angle (deg)'); ylabel('Range (m)');
    title('MIMO Range-Azimuth Map (16 Virtual Elements)');
    grid on;
end

%% 10. Figure 5: Individual Tx-to-Rx Channel Range Profiles
% =========================================================================

figure('Name', 'Individual Tx-to-Rx Channel Range Profiles', 'Position', [100, 100, 1250, 520]);
tiledlayout(2, nRxChannels, 'TileSpacing', 'compact', 'Padding', 'compact');
globalPeakPower = max(meanRangePower(searchMask, :, :), [], 'all');

for iTx = 1:2
    for iRx = 1:nRxChannels
        nexttile;
        chPowerDb = 10 * log10(max(meanRangePower(:, iRx, iTx), eps) / globalPeakPower);
        plot(rangeAxis, chPowerDb, 'LineWidth', 1.2);
        hold on;
        xline(targetRangeMeas, '--r');
        grid on;
        xlim(targetRangeGate);
        ylim([-50, 5]);
        title(sprintf('Tx%d - Rx%d', iTx, iRx));
        if iTx == 2
            xlabel('Range (m)');
        end
        if iRx == 1
            ylabel('Power (dB)');
        end
    end
end

%% 11. Local Helpers
% =========================================================================

function pulseData = arrangeMimoPulseData(rawData, rx, bf, bf_TDD)
% Extract and calibrate dual-channel FMCW pulses from Pluto raw data
    calibrationWeights = loadCalibrationWeights().DigitalWeights;
    calibrationWeights = reshape(conj(calibrationWeights), 1, []);
    calibratedData = rawData .* calibrationWeights;
    
    fs = rx.SamplingRate;
    tsweep = double(bf.FrequencyDeviationTime) / 1e6;
    sweepStartTime = bf_TDD.Ch0On;
    pulseTime = bf_TDD.FrameLength / 1e3;
    nPulses = bf_TDD.BurstCount;
    
    sweepOffsetSamples = ceil(sweepStartTime * fs);
    sweepSamples = (1:ceil(tsweep * fs)) + sweepOffsetSamples;
    pulseEndSample = round(pulseTime * fs);
    pulseStartSamples = (0:nPulses - 1) * pulseEndSample;
    sampleIndices = sweepSamples.' + pulseStartSamples;
    
    nRequiredSamples = max(sampleIndices, [], 'all');
    if nRequiredSamples > size(calibratedData, 1)
        error('Pluto buffer underflow. Rerun the script.');
    end
    
    nSweep = size(sampleIndices, 1);
    pulseData = zeros(nSweep, nPulses, size(calibratedData, 2), 'like', calibratedData);
    for iRx = 1:size(calibratedData, 2)
        pulseData(:, :, iRx) = reshape(calibratedData(sampleIndices, iRx), nSweep, nPulses);
    end
end

function window = localHann(nSamples)
    if nSamples == 1
        window = 1;
    else
        window = 0.5 - 0.5 * cos(2 * pi * (0:nSamples - 1).' / nSamples);
    end
end

function [detectedAngles, peakPowers] = findSpatialPeaks(spectrumDb, angleGrid, maxPeaks, minProminence)
% Local peak finder for spatial spectra with minimum prominence and separation
    if nargin < 3, maxPeaks = 2; end
    if nargin < 4, minProminence = 3; end
    
    isMax = false(size(spectrumDb));
    for i = 2:(length(spectrumDb) - 1)
        if (spectrumDb(i) > spectrumDb(i - 1)) && (spectrumDb(i) > spectrumDb(i + 1))
            isMax(i) = true;
        end
    end
    
    peakIdx = find(isMax);
    if isempty(peakIdx)
        [~, maxIdx] = max(spectrumDb);
        detectedAngles = angleGrid(maxIdx);
        peakPowers = spectrumDb(maxIdx);
        return;
    end
    
    peakVals = spectrumDb(peakIdx);
    
    % Filter out weak peaks (> 25 dB below spectrum max)
    valid = (peakVals >= (max(spectrumDb) - 25));
    peakIdx = peakIdx(valid);
    peakVals = spectrumDb(peakIdx);
    
    [~, sortIdx] = sort(peakVals, 'descend');
    peakIdx = peakIdx(sortIdx);
    
    % Enforce minimum angular separation of 4.0 degrees
    selectedIdx = [];
    minSeparation = 4.0;
    for i = 1:length(peakIdx)
        cand = angleGrid(peakIdx(i));
        if isempty(selectedIdx) || all(abs(cand - angleGrid(selectedIdx)) >= minSeparation)
            selectedIdx(end + 1) = peakIdx(i); %#ok<AGROW>
            if length(selectedIdx) >= maxPeaks
                break;
            end
        end
    end
    
    [detectedAngles, sortAngIdx] = sort(angleGrid(selectedIdx));
    peakPowers = spectrumDb(selectedIdx(sortAngIdx));
end
