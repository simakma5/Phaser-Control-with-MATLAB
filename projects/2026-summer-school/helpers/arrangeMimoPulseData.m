function pulseData = arrangeMimoPulseData(rawData, rx, bf, bf_TDD)
% ARRANGEMIMOPULSEDATA Extract and calibrate dual-channel FMCW pulses from Pluto raw data.
%
%   pulseData = arrangeMimoPulseData(rawData, rx, bf, bf_TDD)
%
%   Applies digital calibration weights across the Pluto channels, extracts
%   fast-time chirp samples according to TDD timing, and reshapes into:
%       pulseData: [nSweep x nPulses x nRxChannels]

    calibrationWeights = loadCalibrationWeights().DigitalWeights;
    calibrationWeights = reshape(conj(calibrationWeights), 1, []);
    assert(size(rawData, 2) == numel(calibrationWeights), ...
        'The receive data and digital calibration channel counts differ.');
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
        error('Pluto buffer underflow. Rerun the script; first hardware capture can be short.');
    end

    nSweep = size(sampleIndices, 1);
    pulseData = zeros(nSweep, nPulses, size(calibratedData, 2), 'like', calibratedData);
    for iRx = 1:size(calibratedData, 2)
        pulseData(:, :, iRx) = reshape(calibratedData(sampleIndices, iRx), nSweep, nPulses);
    end
end
