function rangeResults = processMimoRange(mimoPulseData, fmcwParams, calRange, targetRangeGate)
% PROCESSMIMORANGE Perform FMCW Range FFT, pulse integration, and automated target bin identification.
%
%   rangeResults = processMimoRange(mimoPulseData, fmcwParams, calRange, targetRangeGate)
%
%   Outputs:
%       rangeResults - Struct with fields:
%           rangeAxis         - Calibrated range vector in meters
%           meanRangeSpectrum - Complex spectrum after coherent chirp averaging [nRangeBins x nRx x nTx]
%           meanRangePower    - Power spectrum [nRangeBins x nRx x nTx]
%           combinedPower     - Summed power across all channels [nRangeBins x 1]
%           targetBin         - Index of detected target range bin
%           targetRangeMeas   - Distance in meters of detected target

    if nargin < 3, calRange = 1.6; end
    if nargin < 4, targetRangeGate = [0.5, 5.0]; end

    fs = fmcwParams.fs;
    sweepSlope = fmcwParams.sweepSlope;
    c = physconst('LightSpeed');

    nFastTime = size(mimoPulseData, 1);
    nFft = 2^nextpow2(nFastTime);
    window = localHann(nFastTime);
    windowedData = mimoPulseData .* reshape(window, [], 1, 1, 1);

    % Fast-time Range FFT
    rangeSpectrum = fft(windowedData, nFft, 1);
    rangeSpectrum = rangeSpectrum(1:nFft/2, :, :, :);

    % Calibrated Range Axis (accounting for internal delay via calRange)
    rawRangeAxis = (0:nFft/2 - 1).' * fs / nFft * c / (2 * sweepSlope);
    rangeAxis = rawRangeAxis - calRange;

    % Coherently integrate across pulses in CPI
    meanRangeSpectrum = squeeze(mean(rangeSpectrum, 2)); % [nRangeBins, nRx, nTx]
    meanRangePower = abs(meanRangeSpectrum).^2;

    % Search for prominent peak within targetRangeGate
    searchMask = rangeAxis >= targetRangeGate(1) & rangeAxis <= targetRangeGate(2);
    searchBins = find(searchMask);
    assert(~isempty(searchBins), 'Target range gate [%.1f, %.1f] m contains no range bins.', ...
        targetRangeGate(1), targetRangeGate(2));

    combinedPower = sum(sum(meanRangePower, 2), 3);
    [~, maxSearchIdx] = max(combinedPower(searchMask));
    targetBin = searchBins(maxSearchIdx);
    targetRangeMeas = rangeAxis(targetBin);

    fprintf('Detected target range: %.2f m (FFT bin %d)\n\n', targetRangeMeas, targetBin);

    rangeResults = struct();
    rangeResults.rangeAxis = rangeAxis;
    rangeResults.meanRangeSpectrum = meanRangeSpectrum;
    rangeResults.meanRangePower = meanRangePower;
    rangeResults.combinedPower = combinedPower;
    rangeResults.targetBin = targetBin;
    rangeResults.targetRangeMeas = targetRangeMeas;
    rangeResults.targetRangeGate = targetRangeGate;
end
