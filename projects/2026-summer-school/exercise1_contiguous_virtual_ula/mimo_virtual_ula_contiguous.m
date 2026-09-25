%% MIMO Radar Summer School 2026 - Exercise 1: Contiguous Virtual ULA
% 
% Synthesizes a 16-element contiguous virtual ULA from an 8-element physical
% Rx array and two electronically switched Tx antennas (ADALM-PHASER SMA Out 1 & Out 2).
% Compares physical 8-element and virtual 16-element spatial spectra for both
% simulated targets and hardware captures.
%
% Copyright 2026 Microwave Sensing, Signals and Systems (MS3), TU Delft.

clear; close all; clc;
warning('off','MATLAB:system:ObsoleteSystemObjectMixin');

%% 1. Configuration & Parameters
% =========================================================================

% Target Geometry (Used for Simulation and Hardware Range-Gating)
target_range    = 2.0;         % Target distance (m)
target1_azimuth = -4.5;        % Target 1 azimuth angle (deg)
target2_azimuth = 4.5;         % Target 2 azimuth angle (deg)

% System & Radar Parameters
fc = 10e9;                     % Carrier frequency (10 GHz)
c = physconst('LightSpeed');   % Speed of light (m/s)
lambda = c / fc;               % Wavelength (0.03 m)

% Array Geometry
nRx = 8;                       % Number of physical Rx elements
dRx = lambda / 2;              % Physical Rx spacing (0.015 m)
dTx = 4 * lambda;              % Tx separation = 8 * dRx (0.12 m)

x_rx = (0:(nRx - 1)) * dRx;    % Physical Rx coordinates (m)
x_tx = [0, dTx];               % Physical Tx coordinates (m)
x_virt = (0:(2 * nRx - 1)) * dRx; % Virtual ULA coordinates (m)
nVirt = length(x_virt);        % 16 virtual elements

% Angular Scan Grid
scanAngles = -30:0.5:30;       % Azimuth evaluation grid (deg)
nAngles = length(scanAngles);

% FMCW Waveform Configuration
prf = 2000;                    % Pulse repetition frequency (Hz)
nPulses = 32;                  % Pulses per burst
fs = 1e6;                      % Pluto sampling rate (1 MHz)
rampbandwidth = 500e6;         % FMCW sweep bandwidth (500 MHz)

%% 2. Simulated Spatial Spectrum Calculation
% =========================================================================
k = 2 * pi / lambda;

% Target steering vectors
a_rx_t1   = exp(-1j * k * x_rx.' * sind(target1_azimuth));
a_rx_t2   = exp(-1j * k * x_rx.' * sind(target2_azimuth));
a_virt_t1 = exp(-1j * k * x_virt.' * sind(target1_azimuth));
a_virt_t2 = exp(-1j * k * x_virt.' * sind(target2_azimuth));

% Amplitudes for Target 1 and Target 2
amp1 = 1.0;
amp2 = 0.8;

s_rx_sim   = amp1 * a_rx_t1 + amp2 * a_rx_t2;
s_virt_sim = amp1 * a_virt_t1 + amp2 * a_virt_t2;

P_rx_sim   = zeros(1, nAngles);
P_virt_sim = zeros(1, nAngles);

for iA = 1:nAngles
    ang = scanAngles(iA);
    a_rx   = exp(-1j * k * x_rx.' * sind(ang));
    a_virt = exp(-1j * k * x_virt.' * sind(ang));
    
    P_rx_sim(iA)   = abs(a_rx' * s_rx_sim)^2 / (nRx^2);
    P_virt_sim(iA) = abs(a_virt' * s_virt_sim)^2 / (nVirt^2);
end

norm_db = @(p) 10 * log10(p / max(p));
P_rx_sim_db   = norm_db(P_rx_sim);
P_virt_sim_db = norm_db(P_virt_sim);

%% 3. Hardware Initialization
% =========================================================================
fprintf('Initializing ADALM-PHASER and PlutoSDR...\n');

% Load calibration weights
calibrationweights = loadCalibrationWeights();

% Configure Pluto, Phaser, and TDD engine
[rx, tx, bf, bf_TDD] = setupLabRadar(fc, prf, nPulses, fs, rampbandwidth);

% Radar timing parameters for pulse slicing
tSweep = double(bf.FrequencyDeviationTime) / 1e6;
sweepSlope = rampbandwidth / tSweep;
tstartsweep = bf_TDD.Ch0On;
tpulse = bf_TDD.FrameLength / 1e3;

sweepoffsetsamples = ceil(tstartsweep * fs);
sweepsamples = (1:ceil(tsweep * fs)) + sweepoffsetsamples;
pulseendsample = round(tpulse * fs);
pulsestartsamples = (0:(nPulses - 1)) * pulseendsample;
sampleidxs = repmat(sweepsamples.', 1, nPulses) + pulsestartsamples;
nFastTime = length(sweepsamples);
fast_win = 0.5 * (1 - cos(2 * pi * (0:nFastTime - 1)' / (nFastTime - 1)));

digitalWeights = digitalWeightsCalAdjustment([1; 1], calibrationweights.DigitalWeights);

%% 4. Range-Gating: Locate Target Range Bin
% =========================================================================
fprintf('Capturing reference range profile at boresight...\n');

% Set analog beamformer to boresight (broadside)
boresight_weights = analogWeightsCalAdjustment([ones(4,1), ones(4,1)], calibrationweights.AnalogWeights);
setAnalogBfWeights(bf, boresight_weights);
bf.EnableOut1 = true;
pause(0.05);

ref_raw = captureTransmitWaveform(rx, tx, bf);
ref_combined = ref_raw * conj(digitalWeights);
ref_pulses = ref_combined(sampleidxs);

% Range FFT
ref_fft = fft(ref_pulses .* fast_win, [], 1);
range_profile = mean(abs(ref_fft), 2);

% Range Axis
f_beat = (0:(nFastTime - 1))' / nFastTime * fs;
range_axis = f_beat * c / (2 * sweepSlope);

% Identify target range bin within +/- 0.8 m of specified target_range
range_mask = (range_axis >= (target_range - 0.8)) & (range_axis <= (target_range + 0.8));
search_indices = find(range_mask);

if isempty(search_indices)
    search_indices = 4:floor(nFastTime / 2);
end

[~, max_idx] = max(range_profile(search_indices));
target_bin = search_indices(max_idx);
target_range_meas = range_axis(target_bin);

fprintf('Target range bin: %d (Estimated distance: %.2f m)\n\n', target_bin, target_range_meas);

%% 5. Hardware Data Acquisition: Electronic Beam Scan
% =========================================================================
s1_meas = zeros(1, nAngles);
s2_meas = zeros(1, nAngles);

% --- Transmit 1: SMA Out 1 (x = 0) ---
fprintf('Executing beam scan for Tx 1 (SMA Out 1)...\n');
bf.EnableOut1 = true;
pause(0.05);

for iA = 1:nAngles
    ang = scanAngles(iA);
    
    % Steering weights for physical 8-element array
    steerweights = exp(-1j * k * x_rx.' * sind(ang));
    analogsteer = analogWeightsCalAdjustment([steerweights(1:4), steerweights(5:8)], calibrationweights.AnalogWeights);
    setAnalogBfWeights(bf, analogsteer);
    
    % Capture and process burst
    data_raw = captureTransmitWaveform(rx, tx, bf);
    data_comb = data_raw * conj(digitalWeights);
    data_pulses = data_comb(sampleidxs);
    
    % Extract complex envelope at target range bin
    pulse_fft = fft(data_pulses .* fast_win, [], 1);
    s1_meas(iA) = mean(pulse_fft(target_bin, :));
end

% --- Transmit 2: SMA Out 2 (x = 4*lambda) ---
fprintf('Executing beam scan for Tx 2 (SMA Out 2)...\n');
bf.EnableOut1 = false;
pause(0.05);

for iA = 1:nAngles
    ang = scanAngles(iA);
    
    % Steering weights for physical 8-element array
    steerweights = exp(-1j * k * x_rx.' * sind(ang));
    analogsteer = analogWeightsCalAdjustment([steerweights(1:4), steerweights(5:8)], calibrationweights.AnalogWeights);
    setAnalogBfWeights(bf, analogsteer);
    
    % Capture and process burst
    data_raw = captureTransmitWaveform(rx, tx, bf);
    data_comb = data_raw * conj(digitalWeights);
    data_pulses = data_comb(sampleidxs);
    
    % Extract complex envelope at target range bin
    pulse_fft = fft(data_pulses .* fast_win, [], 1);
    s2_meas(iA) = mean(pulse_fft(target_bin, :));
end

% Restore hardware to idle
disableTddTrigger(bf_TDD);
fprintf('Hardware acquisition complete.\n\n');

%% 6. Virtual Array Synthesis & Spatial Spectrum
% =========================================================================

% Measured physical 8-element array power spectrum (Tx 1 alone)
P_rx_hw = abs(s1_meas).^2;

% Virtual 16-element array synthesis:
% S_virt(theta) = S1(theta) + S2(theta) * exp(j * 2*pi * dTx * sin(theta) / lambda)
tx2_phase_comp = exp(1j * k * dTx * sind(scanAngles));
s_virt_meas = s1_meas + s2_meas .* tx2_phase_comp;
P_virt_hw = abs(s_virt_meas).^2;

P_rx_hw_db   = norm_db(P_rx_hw);
P_virt_hw_db = norm_db(P_virt_hw);

%% 7. Visualization
% =========================================================================

% Figure 1: Measured Range Profile
figure('Name', 'Range Profile', 'Position', [80, 150, 600, 360]);
plot(range_axis, 20 * log10(range_profile / max(range_profile)), 'LineWidth', 1.5);
hold on;
xline(target_range_meas, '--r', sprintf('Target: %.2f m', target_range_meas), ...
      'LabelVerticalAlignment', 'bottom', 'LineWidth', 1.2);
grid on; xlim([0, 8]); ylim([-40, 5]);
xlabel('Range (m)'); ylabel('Normalized Amplitude (dB)');
title('Measured Range Profile');

% Figure 2: Side-by-Side Spatial Spectrum Comparison
figure('Name', 'Spatial Spectrum Comparison', 'Position', [120, 150, 1150, 480]);

% Left Subplot: Simulated Spatial Spectrum
subplot(1, 2, 1);
plot(scanAngles, P_rx_sim_db,   'LineWidth', 1.8, 'DisplayName', 'Physical Rx (8 elements, 3.5\lambda)'); hold on;
plot(scanAngles, P_virt_sim_db, 'LineWidth', 1.8, 'DisplayName', 'Virtual Array (16 elements, 7.5\lambda)');
xline(target1_azimuth, ':k', sprintf('%.1f^\\circ', target1_azimuth), 'LabelVerticalAlignment', 'top');
xline(target2_azimuth, ':k', sprintf('%.1f^\\circ', target2_azimuth), 'LabelVerticalAlignment', 'top');
grid on; xlim([min(scanAngles), max(scanAngles)]); ylim([-35, 2]);
xlabel('Azimuth Angle (deg)'); ylabel('Normalized Power (dB)');
title('Simulated Spatial Spectrum');
legend('Location', 'south');

% Right Subplot: Measured Spatial Spectrum
subplot(1, 2, 2);
plot(scanAngles, P_rx_hw_db,   'LineWidth', 1.8, 'DisplayName', 'Physical Rx (8 elements, 3.5\lambda)'); hold on;
plot(scanAngles, P_virt_hw_db, 'LineWidth', 1.8, 'DisplayName', 'Virtual Array (16 elements, 7.5\lambda)');
xline(target1_azimuth, ':k', sprintf('%.1f^\\circ', target1_azimuth), 'LabelVerticalAlignment', 'top');
xline(target2_azimuth, ':k', sprintf('%.1f^\\circ', target2_azimuth), 'LabelVerticalAlignment', 'top');
grid on; xlim([min(scanAngles), max(scanAngles)]); ylim([-35, 2]);
xlabel('Azimuth Angle (deg)'); ylabel('Normalized Power (dB)');
title('Measured Spatial Spectrum');
legend('Location', 'south');
