%% MIMO radar summer school 2026 - contiguous virtual ULA (2x8 MIMO)
%
% Synthesises a 16-element contiguous virtual uniform linear array (ULA) from 
% an eight-element physical Rx array (ADALM-PHASER) and two electronically switched 
% transmit antennas (SMA out 1 and out 2 spaced by 4*lambda).
%
% This script is structured into modular sections designed for easy conversion
% to an interactive MATLAB Live Script (.mlx). All functional, non-interactive
% logic is encapsulated in standalone functions within the helpers/ folder.
%
% Copyright 2026 Microwave Sensing, Signals and Systems (MS3), TU Delft.

clear; close all; clc;
warning('off','MATLAB:system:ObsoleteSystemObjectMixin');

%% 0. Environment setup and path initialisation
% =========================================================================
scriptFolder = fileparts(mfilename('fullpath'));
if isempty(scriptFolder), scriptFolder = pwd; end
addpath(scriptFolder);
addpath(fullfile(scriptFolder, 'helpers'));

% Add repository root shared and demo libraries
repoRoot = fullfile(scriptFolder, '..', '..');
addpath(genpath(fullfile(repoRoot, 'shared')));
addpath(genpath(fullfile(repoRoot, 'demos')));

%% 1. Interactive radar and processing controls
% =========================================================================
% [Live script controls to configure]:
%   dTx_lambda    -> Numeric edit field: e.g. 4.0 (1.0 for overlapped, 4.0 for contiguous ULA)
%   targetCount   -> Numeric edit field: integer >= 1 (default: 2)
%   spatialWindow -> Drop-down: ["Uniform", "Hann", "Chebyshev"]
%   sllChebDb     -> Slider / numeric: [-40, -15] step 1 (default: -25)
%   tx_phase_cal  -> Slider: [-180, 180] step 5 (default: 0.0)
%   calRange      -> Numeric edit field (default: 1.6)
%   targetRangeMin-> Numeric edit field / slider (default: 0.5)
%   targetRangeMax-> Numeric edit field / slider (default: 5.0)

% Transmit antenna separation in wavelengths (lambda)
%   1.0 -> Overlapped array bracket
%   4.0 -> Non-overlapped contiguous linear array bracket (default)
dTx_lambda = 4.0;

% Target count for automated peak locking and theoretical simulation (default: 2)
targetCount = 2;

% Spatial tapering / windowing across array elements
spatialWindow = 'Uniform';   % Options: 'Uniform', 'Hann', 'Chebyshev'
sllChebDb = -25;             % Sidelobe level for Chebyshev window (dB)

% Tx phase calibration offset between out 1 and out 2 (degrees)
tx_phase_cal = 0.0;

% FMCW range calibration offset and search interval
calRange = 1.6;              % Hardware range offset calibration (m)
targetRangeGate = [0.5, 5.0];% Search interval for reflectors (calibrated m)
portSwitchPause = 0.10;      % Switch settling time (s)

% System and radar parameters
fc = 10e9;                   % Carrier frequency (10 GHz)
c = physconst('LightSpeed'); % Speed of light (m/s)
lambda = c / fc;             % Wavelength (0.03 m)
maxRange = 10;               % Maximum instrumented range (m)
rangeResolution = 0.1;       % Desired range resolution (m)
maxSpeed = 5;                % Max speed for PRF selection (m/s)
speedResolution = 1/2;       % Speed resolution for chirp count (m/s)

% Array geometry setup
nRxPhysical = 8;
dRx = lambda / 2;
dTx = dTx_lambda * lambda;

arrayParams = struct();
arrayParams.x_rx        = (0:(nRxPhysical - 1)) * dRx;
arrayParams.x_tx        = [0, dTx];
arrayParams.x_virt      = [arrayParams.x_rx, arrayParams.x_rx + dTx];
arrayParams.dTx_lambda  = dTx_lambda;
arrayParams.lambda      = lambda;

%% 2. Figure 1: Array geometry and spatial convolution
% =========================================================================
% Visualises physical Rx (8 elements), physical Tx (2 elements), and the
% synthesised contiguous 16-element virtual ULA.
plotMimoArrayGeometry(arrayParams);

%% 3. Radar hardware configuration
% =========================================================================
[rx, tx, bf, bf_TDD, fmcwParams] = setupMimoFmcwRadar( ...
    fc, rangeResolution, maxRange, maxSpeed, speedResolution);
cleanupGuard = onCleanup(@() cleanupAntenna(rx, tx, bf, bf_TDD));

%% 4. Multichannel FMCW hardware data acquisition
% =========================================================================
mimoPulseData = captureMimoData( ...
    rx, tx, bf, bf_TDD, fmcwParams, portSwitchPause);

%% 5. Figure 2: Precise range measurement and target detection
% =========================================================================
% Computes range FFT and coherent pulse averaging, applies calRange offset,
% and automatically identifies the target range bin.
rangeResults = processMimoRange( ...
    mimoPulseData, fmcwParams, calRange, targetRangeGate);

plotMimoRangeProfile(rangeResults);

%% 6. Figure 3: Spatial spectrum, AOA detection and simulated benchmark
% =========================================================================
% Digital beamforming, automatic peak detection, spatial tapering, and
% data-driven simulation benchmark generation.
doaParams = struct();
doaParams.azimuthGrid   = -60:0.5:60;
doaParams.tx_phase_cal  = tx_phase_cal;
doaParams.spatialWindow = spatialWindow;
doaParams.sllChebDb     = sllChebDb;
doaParams.targetCount   = targetCount;

spatialResults = calculateMimoSpatialSpectrum( ...
    rangeResults, arrayParams, doaParams);

plotMimoSpatialSpectrum(spatialResults);

%% 7. Figure 4: 2D MIMO range-azimuth map
% =========================================================================
plotMimoRangeAzimuth(rangeResults, spatialResults, doaParams);

%% 8. Figure 5: Range profiles by transmit channel
% =========================================================================
% Displays all eight Rx channel range profiles for Tx 1 and Tx 2 side by side.
% Rotating one transmit antenna by 90 degrees enables polarimetric
% observation (co-polarised and cross-polarised returns).
plotMimoTxRangeProfiles(rangeResults);
