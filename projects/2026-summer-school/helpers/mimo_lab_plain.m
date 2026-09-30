%% MIMO radar summer school 2026 - contiguous virtual ULA (2x8 MIMO)
%
% Synthesizes a 16-element contiguous virtual uniform linear array (ULA) from 
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

%% 0. Environment setup and path initialization
% =========================================================================
scriptFolder = fileparts(mfilename('fullpath'));
if isempty(scriptFolder), scriptFolder = pwd; end
addpath(scriptFolder);
if exist(fullfile(scriptFolder, 'helpers'), 'dir')
    addpath(fullfile(scriptFolder, 'helpers'));
end

% Locate repository root containing 'shared' and 'demos'
curr = scriptFolder;
while ~exist(fullfile(curr, 'shared'), 'dir') && ~strcmp(curr, fileparts(curr))
    curr = fileparts(curr);
end
repoRoot = curr;
addpath(genpath(fullfile(repoRoot, 'shared')));
addpath(genpath(fullfile(repoRoot, 'demos')));

%% 1. Interactive radar and processing controls
% =========================================================================
% In MATLAB Live Script (.mlx), configure each variable below using an
% interactive control (slider, numeric edit field, or drop-down menu).
%
% [Live script controls]:
%   --- 1.1 Radar Waveform & Kinematics (Hardware Controls) ---
%   maxRange        -> Slider / Numeric: [2, 30] m, step 1 (default: 10)
%   rangeResolution -> Slider / Numeric: [0.08, 0.50] m, step 0.01 (default: 0.10)
%   maxSpeed        -> Slider / Numeric: [1.0, 15.0] m/s, step 0.5 (default: 5.0)
%   speedResolution -> Drop-down / Slider: [0.2, 0.25, 0.5, 1.0, 2.0] m/s (default: 0.5)
%
%   --- 1.2 Array Geometry & Virtual Synthesis Controls ---
%   dTx_lambda      -> Numeric / Slider: [1.0, 8.0], step 0.5 (default: 4.0)
%
%   --- 1.3 Calibration & Range Gating Controls ---
%   calRange        -> Numeric edit field: [0.0, 5.0] m, step 0.05 (default: 1.6)
%   targetRangeMin  -> Slider / Numeric: [0.1, 10.0] m, step 0.1 (default: 0.5)
%   targetRangeMax  -> Slider / Numeric: [0.5, 15.0] m, step 0.1 (default: 5.0)
%
%   --- 1.4 AOA Detection & Spatial Beamforming Controls ---
%   targetCount     -> Numeric / Slider: integer [1, 5], step 1 (default: 2)
%   spatialWindow   -> Drop-down: ["Uniform", "Hann", "Chebyshev"] (default: 'Uniform')
%   sllChebDb       -> Slider / Numeric: [-45, -15] dB, step 1 (default: -25)
%   tx_phase_cal    -> Slider: [-180, 180] deg, step 5 (default: 0.0)
%
% NOTE ON LIVE SCRIPT WORKFLOW:
%   - Modifying Hardware Controls (maxRange, rangeResolution, maxSpeed,
%     speedResolution) requires re-running from Section 3 (Hardware Config)
%     and Section 4 (Data Acquisition).
%   - Modifying Post-Processing Controls (targetRangeGate, targetCount,
%     spatialWindow, sllChebDb, tx_phase_cal) can be evaluated immediately
%     by re-running Sections 5 through 8 without re-acquiring data!

% 1.1 Radar waveform and kinematics controls (Hardware)
maxRange = 10;                  % Maximum instrumented range (m), bounds: [2, 30]
rangeResolution = 0.10;         % Desired range resolution (m), bounds: [0.08, 0.50]
maxSpeed = 5.0;                 % Maximum unambiguous speed (m/s), bounds: [1.0, 15.0]
speedResolution = 0.5;          % Speed resolution for chirp count (m/s), bounds: [0.2, 2.0]

% 1.2 Transmit antenna separation in wavelengths (lambda)
%   1.0 -> Overlapped array bracket
%   4.0 -> Non-overlapped contiguous linear array bracket (default)
dTx_lambda = 4.0;               % Bounds: [1.0, 8.0]

% 1.3 FMCW range calibration offset and search interval
calRange = 1.6;                 % Hardware range offset calibration (m), bounds: [0.0, 5.0]
targetRangeMin = 0.5;           % Search gate minimum distance (m), bounds: [0.1, 10.0]
targetRangeMax = 5.0;           % Search gate maximum distance (m), bounds: [0.5, 15.0]
targetRangeGate = [targetRangeMin, targetRangeMax];
portSwitchPause = 0.10;         % Switch settling time (s)

% 1.4 Target count for automated peak locking and theoretical simulation
targetCount = 2;                % Expected target count, bounds: [1, 5]

% 1.5 Spatial tapering / windowing across array elements
spatialWindow = 'Uniform';      % Options: 'Uniform', 'Hann', 'Chebyshev'
sllChebDb = -25;                % Sidelobe level for Chebyshev window (dB), bounds: [-45, -15]

% 1.6 Tx phase calibration offset between out 1 and out 2 (degrees)
tx_phase_cal = 0.0;             % Bounds: [-180, 180] deg

% System constants and array geometry setup
fc = 10e9;                      % Carrier frequency (10 GHz)
c = physconst('LightSpeed');    % Speed of light (m/s)
lambda = c / fc;                % Wavelength (0.03 m)

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
% Visualizes physical Rx (8 elements), physical Tx (2 elements), and the
% synthesized contiguous 16-element virtual ULA.
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
% observation (co-polarized and cross-polarized returns).
plotMimoTxRangeProfiles(rangeResults);
