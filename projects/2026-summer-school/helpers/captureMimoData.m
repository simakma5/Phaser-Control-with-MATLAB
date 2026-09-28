function mimoPulseData = captureMimoData(rx, tx, bf, bf_TDD, fmcwParams, portSwitchPause)
% CAPTUREMIMODATA Acquire 2x8 FMCW radar data across switched transmitters.
%
%   mimoPulseData = captureMimoData(rx, tx, bf, bf_TDD, fmcwParams, portSwitchPause)
%
%   Executes Switched Single-Element Mode: cycles 4 element pairs across the
%   two ADAR1000 chips for Tx 1 and Tx 2 (8 rapid bursts total in ~1-2 s).
%
%   Outputs:
%       mimoPulseData - Complex FMCW tensor [nFastTime x nPulses x 8 x 2]

    if nargin < 6, portSwitchPause = 0.10; end

    txWaveform = fmcwParams.txWaveform;
    nPulses = fmcwParams.nPulses;

    fprintf('Executing 2x8 MIMO capture (Switched Single-Element Mode)...\n');
    % 4 element pairs across 2 ADAR1000 chips:
    %   Pair 1: Element 1 (Chip 1) & Element 5 (Chip 2)
    %   Pair 2: Element 2 (Chip 1) & Element 6 (Chip 2)
    %   Pair 3: Element 3 (Chip 1) & Element 7 (Chip 2)
    %   Pair 4: Element 4 (Chip 1) & Element 8 (Chip 2)
    % Pluto Ch 2 = Subarray 1 (Elements 1..4)
    % Pluto Ch 1 = Subarray 2 (Elements 5..8)

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

    fprintf('Data acquisition complete.\n\n');
end
