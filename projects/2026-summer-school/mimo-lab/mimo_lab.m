%% MIMO Radar Summer School 2026 - Contiguous Virtual ULA (2x8 MIMO)
%
% Synthesizes a 16-element contiguous virtual uniform linear array (ULA) from 
% an 8-element physical Rx array (ADALM-PHASER) and two electronically switched 
% transmit antennas (SMA Out 1 & Out 2 spaced by 4*lambda).
%
% This script is structured into modular sections designed for easy conversion
% to an interactive MATLAB Live Script (.mlx). All functional, non-interactive
% logic is encapsulated in standalone functions within this folder.
%
% Copyright 2026 Microwave Sensing, Signals and Systems (MS3), TU Delft.

clear; close all; clc;
warning('off','MATLAB:system:ObsoleteSystemObjectMixin');

%% 0. Environment Setup & Path Initialization
% =========================================================================
scriptFolder = fileparts(mfilename('fullpath'));
if isempty(scriptFolder), scriptFolder = pwd; end
addpath(scriptFolder);

% Add repository root shared and demo libraries
repoRoot = fullfile(scriptFolder, '..', '..', '..');
addpath(genpath(fullfile(repoRoot, 'shared')));
addpath(genpath(fullfile(repoRoot, 'demos')));

%% 1. Interactive Radar & Processing Controls
% =========================================================================
% [LIVE SCRIPT CONTROLS TO CONFIGURE]:
%   arrayMode     -> Drop-Down: ["2x8_switched", "2x2_subarray"]
%   spatialWindow -> Drop-Down: ["Uniform", "Hann", "Chebyshev"]
%   sllChebDb     -> Slider / Numeric: [-40, -15] step 1 (default: -25)
%   tx_phase_cal  -> Slider: [-180, 180] step 5 (default: 0.0)
%   calRange      -> Numeric Edit Field (default: 1.6)
%   targetRangeMin-> Numeric Edit Field / Slider (default: 0.5)
%   targetRangeMax-> Numeric Edit Field / Slider (default: 5.0)

% Array Mode: '2x8_switched' (16-element virtual ULA) or '2x2_subarray' (4-channel baseline)
arrayMode = '2x8_switched';

% Spatial tapering / windowing across array elements
spatialWindow = 'Uniform';   % Options: 'Uniform', 'Hann', 'Chebyshev'
sllChebDb = -25;             % Sidelobe level for Chebyshev window (dB)

% Tx phase calibration offset between Out 1 and Out 2 (degrees)
tx_phase_cal = 0.0;

% FMCW range calibration offset and search interval
calRange = 1.6;              % Hardware range offset calibration (m)
targetRangeGate = [0.5, 5.0];% Search interval for reflectors (calibrated m)
portSwitchPause = 0.10;      % Switch settling time (s)

% System & Radar Parameters
fc = 10e9;                   % Carrier frequency (10 GHz)
c = physconst('LightSpeed'); % Speed of light (m/s)
lambda = c / fc;             % Wavelength (0.03 m)
maxRange = 10;               % Maximum instrumented range (m)
rangeResolution = 0.1;       % Desired range resolution (m)
maxSpeed = 5;                % Max speed for PRF selection (m/s)
speedResolution = 1/2;       % Speed resolution for chirp count (m/s)

% Array Geometry Setup
nRxPhysical = 8;
dRx = lambda / 2;
dTx = 4 * lambda;            % 8 * dRx

arrayParams = struct();
arrayParams.x_rx        = (0:(nRxPhysical - 1)) * dRx;
arrayParams.x_tx        = [0, dTx];
arrayParams.x_virt      = (0:(2 * nRxPhysical - 1)) * dRx;
arrayParams.lambda      = lambda;
arrayParams.arrayMode   = arrayMode;

%% 2. Figure 1: Array Geometry & Spatial Convolution
% =========================================================================
% Visualizes physical Rx (8 elements), physical Tx (2 elements), and the
% synthesized contiguous 16-element virtual ULA.
plotMimoArrayGeometry(arrayParams);

%% 3. Radar Hardware Configuration
% =========================================================================
[rx, tx, bf, bf_TDD, fmcwParams] = setupMimoFmcwRadar( ...
    fc, rangeResolution, maxRange, maxSpeed, speedResolution);
cleanupGuard = onCleanup(@() cleanupAntenna(rx, tx, bf, bf_TDD));

%% 4. Multichannel FMCW Hardware Data Acquisition
% =========================================================================
mimoPulseData = captureMimoData( ...
    rx, tx, bf, bf_TDD, fmcwParams, arrayMode, portSwitchPause);

%% 5. Figure 2: Precise Range Measurement & Target Detection
% =========================================================================
% Computes range FFT, coherent pulse averaging, applies calRange offset,
% and automatically identifies the target range bin.
rangeResults = processMimoRange( ...
    mimoPulseData, fmcwParams, calRange, targetRangeGate);

plotMimoRangeProfile(rangeResults);

%% 6. Figure 3: Spatial Spectrum, AOA Detection & Simulated Benchmark
% =========================================================================
% Digital beamforming, automatic peak detection, spatial tapering, and
% data-driven simulation benchmark generation.
doaParams = struct();
doaParams.azimuthGrid   = -60:0.5:60;
doaParams.tx_phase_cal  = tx_phase_cal;
doaParams.spatialWindow = spatialWindow;
doaParams.sllChebDb     = sllChebDb;

spatialResults = calculateMimoSpatialSpectrum( ...
    rangeResults, arrayParams, doaParams);

plotMimoSpatialSpectrum(spatialResults);

%% 7. Figure 4: 2D MIMO Range-Azimuth Map
% =========================================================================
plotMimoRangeAzimuth(rangeResults, spatialResults, doaParams);
