function [rx, tx, bf, bf_TDD, fmcwParams] = setupMimoFmcwRadar(fc, rangeResolution, maxRange, maxSpeed, speedResolution)
% SETUPMIMOFMCWRADAR Configure ADALM-PHASER, PlutoSDR, and TDD engine for FMCW MIMO radar.
%
%   [rx, tx, bf, bf_TDD, fmcwParams] = setupMimoFmcwRadar(fc, rangeResolution, maxRange, maxSpeed, speedResolution)
%
%   Inputs:
%       fc              - Center frequency (Hz, typically 10e9)
%       rangeResolution - Desired range resolution (m, default: 0.1)
%       maxRange        - Maximum instrumented range (m, default: 10)
%       maxSpeed        - Maximum velocity for PRF selection (m/s, default: 5)
%       speedResolution - Velocity resolution for pulse count (m/s, default: 0.5)
%
%   Outputs:
%       rx, tx, bf, bf_TDD - Initialized hardware system objects
%       fmcwParams         - Struct containing derived FMCW waveform and timing parameters

    if nargin < 2, rangeResolution = 0.1; end
    if nargin < 3, maxRange = 10; end
    if nargin < 4, maxSpeed = 5; end
    if nargin < 5, speedResolution = 0.5; end

    c = physconst('LightSpeed');
    lambda = c / fc;

    % Derive FMCW parameters (matching standard AD radar helpers)
    rampBandwidth = ceil(rangeres2bw(rangeResolution) / 1e6) * 1e6;
    fmaxdop = speed2dop(2 * maxSpeed, lambda);
    prf = 2 * fmaxdop;
    nPulses = ceil(2 * maxSpeed / speedResolution);
    tpulse = ceil((1 / prf) * 1e3) * 1e-3;
    tsweep = getFMCWSweepTime(tpulse, tpulse);
    sweepSlope = rampBandwidth / tsweep;
    fmaxbeat = sweepSlope * range2time(maxRange);
    fs = max(ceil(2 * fmaxbeat), 520834);

    fmcwParams = struct();
    fmcwParams.fc = fc;
    fmcwParams.lambda = lambda;
    fmcwParams.rampBandwidth = rampBandwidth;
    fmcwParams.prf = prf;
    fmcwParams.nPulses = nPulses;
    fmcwParams.tpulse = tpulse;
    fmcwParams.tsweep = tsweep;
    fmcwParams.sweepSlope = sweepSlope;
    fmcwParams.fs = fs;

    fprintf('FMCW bandwidth: %.1f MHz, sweep time: %.3f ms, chirps/CPI: %d, fs: %.2f MHz\n', ...
        rampBandwidth / 1e6, tsweep * 1e3, nPulses, fs / 1e6);
    fprintf('Initializing ADALM-PHASER and PlutoSDR...\n');

    % Load calibration weights
    calibrationweights = loadCalibrationWeights();

    % Configure Pluto, Phaser, and TDD engine using verified setupFMCWRadar
    [rx, tx, bf, bf_TDD] = setupFMCWRadar(fc, fs, tpulse, tsweep, nPulses, rampBandwidth);

    % Transmit baseband waveform
    amp = 0.9 * 2^15;
    txWaveform = amp * ones(rx.SamplesPerFrame, 2);
    fmcwParams.txWaveform = txWaveform;

    % Flush initial frame to clear Pluto DMA buffer
    rx();

    % Apply default broadside analog calibration phases to ADAR1000
    phaseshifts = wrapTo360(rad2deg(angle(calibrationweights.AnalogWeights)));
    bf.RxPhase(:) = [phaseshifts(:, 1)', phaseshifts(:, 2)'];
    bf.RxGain(:) = 127;
    bf.LatchRxSettings();
end
