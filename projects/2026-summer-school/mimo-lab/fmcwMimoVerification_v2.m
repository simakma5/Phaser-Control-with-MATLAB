%% Two-transmitter TDM-MIMO verification with ADALM-PHASER
% This script performs a simple 2-Tx by 2-Rx MIMO verification using the
% two selectable Phaser transmit outputs and the two Pluto receive
% channels. OUT1 and OUT2 do NOT transmit simultaneously. A complete FMCW
% coherent processing interval (CPI) is captured from one output, then the
% RF output switch is changed and another CPI is captured.
%
% Changes in this version:
%   - Range axis is calibrated with a fixed offset (calRange), the same way
%     as in the FMCW Range Lab. Measure calRange once with a reflector at a
%     known distance, then the reported ranges become absolute.
%   - A range-azimuth profile is formed from the four virtual channels and
%     plotted (target assumed stationary).
%
% Hardware setup:
%   1. Connect one X-band transmit antenna to OUT1 and one to OUT2.
%   2. Point both transmit antennas in the same direction as the receive
%      array. Keep their positions and cable routing fixed.
%   3. Place a strong stationary target, such as a corner reflector, in the
%      range gate configured below.
%   4. Remove or turn off other X-band transmitters, including the HB100.

clear; close all;
warning('off','MATLAB:system:ObsoleteSystemObjectMixin');

%% Make the repository helpers available

scriptFolder = fileparts(mfilename('fullpath'));
repositoryFolder = fileparts(fileparts(scriptFolder));
addpath(fullfile(scriptFolder,'helpers'));
addpath(genpath(fullfile(repositoryFolder,'shared')));

%% User-adjustable verification settings

fc = 10e9;                    % Approximate RF carrier frequency (Hz)
maxRange = 10;                % Instrumented range used for setup (m)
rangeResolution = 0.1;        % Requested FMCW range resolution (m)
maxSpeed = 5;                 % Used to select PRF (m/s)
speedResolution = 1/2;        % Used to select number of chirps (m/s)

calRange = 1.6;               % Range calibration offset (m). Subtracted from
                              % the raw range axis, as in FMCW_Range_Lab.
                              % Set to the raw peak range measured with a
                              % reflector at a known true distance:
                              %   calRange = rawPeak - trueDistance.

targetRangeGate = [0.5 5];    % Search only this interval for the reflector (m)
                              % NOTE: interpreted in calibrated range.
nMimoRepeats = 4;             % Repeat Tx1/Tx2 captures to test phase stability
portSwitchPause = 0.10;       % Host-controlled switch settling time (s)

% Range-azimuth imaging settings
azimuthGrid = -60:1:60;       % Azimuth angles to evaluate (deg)
elementSpacing = freq2wavelen(10.39875e9)/2;  % Rx subarray half-lambda spacing
rxPhaseCenterSpacing = 4*elementSpacing;      % 4 elements per subarray -> 2*lambda
txSpacing = rxPhaseCenterSpacing/2;           % Tx separation that fills the
                                              % virtual array (adjust to your
                                              % physical OUT1/OUT2 spacing).

minimumSnrDb = 10;            % Suggested basic verification threshold
maximumPhaseStdDeg = 20;      % Suggested static-scene phase repeatability

assert(targetRangeGate(1) >= -maxRange && targetRangeGate(2) <= maxRange && ...
    targetRangeGate(1) < targetRangeGate(2), ...
    'targetRangeGate must lie inside [-maxRange, maxRange].');
assert(nMimoRepeats >= 2, ...
    'Use at least two repeats to measure phase repeatability.');

%% Derive the FMCW parameters in the same way as fmcwDemo.m

c = physconst('LightSpeed');
lambda = c/fc;
rampBandwidth = ceil(rangeres2bw(rangeResolution)/1e6)*1e6;
fmaxdop = speed2dop(2*maxSpeed,lambda);
prf = 2*fmaxdop;
nPulses = ceil(2*maxSpeed/speedResolution);
tpulse = ceil((1/prf)*1e3)*1e-3;
tsweep = getFMCWSweepTime(tpulse,tpulse);
sweepSlope = rampBandwidth/tsweep;
fmaxbeat = sweepSlope*range2time(maxRange);
fs = max(ceil(2*fmaxbeat),520834);

fprintf('FMCW bandwidth: %.1f MHz, sweep time: %.3f ms, chirps/CPI: %d\n', ...
    rampBandwidth/1e6,tsweep*1e3,nPulses);

%% Configure Phaser, Pluto and the TDD engine

[rx,tx,bf,bf_TDD] = setupFMCWRadar( ...
    fc,fs,tpulse,tsweep,nPulses,rampBandwidth);
cleanupGuard = onCleanup(@() cleanupAntenna(rx,tx,bf,bf_TDD));

amp = 0.9*2^15;
txWaveform = amp*ones(rx.SamplesPerFrame,2);

% Discard one initial frame, matching the workaround used by the existing
% continuous FMCW example.
rx();

%% Capture complete CPIs from OUT1 and OUT2

mimoCaptures = cell(nMimoRepeats,2);

for iRepeat = 1:nMimoRepeats
    fprintf('MIMO capture %d of %d\n',iRepeat,nMimoRepeats);

    for iTx = 1:2
        bf.EnableOut1 = (iTx == 1);
        pause(portSwitchPause);

        rawData = captureTransmitWaveform(rx,tx,bf,txWaveform);
        pulseData = arrangeMimoPulseData(rawData,rx,bf,bf_TDD);

        mimoCaptures{iRepeat,iTx} = pulseData;
    end
end

firstCapture = mimoCaptures{1,1};
nFastTime = size(firstCapture,1);
nRx = size(firstCapture,3);
mimoData = zeros(nFastTime,nPulses,nRx,2,nMimoRepeats, ...
    'like',firstCapture);
for iRepeat = 1:nMimoRepeats
    for iTx = 1:2
        mimoData(:,:,:,iTx,iRepeat) = mimoCaptures{iRepeat,iTx};
    end
end
clear mimoCaptures rawData pulseData firstCapture;

%% Form a range profile while retaining every Tx/Rx combination

nFastTime = size(mimoData,1);
nFft = 2^nextpow2(nFastTime);
window = localHann(nFastTime);
windowedData = mimoData.*reshape(window,[],1,1,1,1);

rangeSpectrum = fft(windowedData,nFft,1);
rangeSpectrum = rangeSpectrum(1:nFft/2,:,:,:,:);

% Raw range axis, then apply the fixed calibration offset (Range Lab style).
rawRangeAxis = (0:nFft/2-1).'*fs/nFft*c/(2*sweepSlope);
rangeAxis = rawRangeAxis - calRange;

% Coherently average the chirps inside each CPI.
rangePerRepeat = mean(rangeSpectrum,2);
rangePerRepeat = reshape(rangePerRepeat, ...
    numel(rangeAxis),size(mimoData,3),2,nMimoRepeats);
meanRangePower = mean(abs(rangePerRepeat).^2,4);

searchMask = rangeAxis >= targetRangeGate(1) & ...
    rangeAxis <= targetRangeGate(2);
searchBins = find(searchMask);
assert(any(searchMask), ...
    'The calibrated target range gate contains no range bins.');
combinedPower = sum(sum(meanRangePower,2),3);
[~,localTargetBin] = max(combinedPower(searchMask));
targetBin = searchBins(localTargetBin);
commonTargetRange = rangeAxis(targetBin);

% H is Rx x Tx x repeat at the common reflector range bin.
H = reshape(rangePerRepeat(targetBin,:,:,:), ...
    size(mimoData,3),2,nMimoRepeats);

%% Calculate per-channel range, SNR and relative phase

nRx = size(H,1);
nTx = size(H,2);
nChannels = nRx*nTx;
channelName = strings(nChannels,1);
peakRangeM = zeros(nChannels,1);
snrDb = zeros(nChannels,1);
amplitudeDb = zeros(nChannels,1);
relativePhaseDeg = zeros(nChannels,1);

meanAmplitude = sqrt(mean(abs(H).^2,3));
meanAmplitudeDb = 20*log10(max(meanAmplitude,eps)/max(meanAmplitude,[],'all'));

phaseReference = reshape(H(1,1,:),1,1,nMimoRepeats);
relativeH = H.*conj(phaseReference);
meanRelativePhase = angle(mean(exp(1j*angle(relativeH)),3));

guardBins = max(2,ceil(rangeResolution/(rangeAxis(2)-rangeAxis(1))));
noiseMask = searchMask;
noiseMask(max(1,targetBin-guardBins): ...
    min(numel(noiseMask),targetBin+guardBins)) = false;
assert(any(noiseMask), ...
    'The target range gate is too narrow to estimate the noise floor.');

iChannel = 0;
for iTx = 1:nTx
    for iRx = 1:nRx
        iChannel = iChannel+1;
        channelName(iChannel) = sprintf('Tx%d-Rx%d',iTx,iRx);

        channelPower = meanRangePower(:,iRx,iTx);
        [peakPower,localPeakBin] = max(channelPower(searchMask));
        peakBin = searchBins(localPeakBin);
        noisePower = median(channelPower(noiseMask));

        peakRangeM(iChannel) = rangeAxis(peakBin);
        snrDb(iChannel) = 10*log10(max(peakPower,eps)/max(noisePower,eps));
        amplitudeDb(iChannel) = meanAmplitudeDb(iRx,iTx);
        relativePhaseDeg(iChannel) = rad2deg(meanRelativePhase(iRx,iTx));
    end
end

channelResults = table(channelName,peakRangeM,snrDb,amplitudeDb, ...
    relativePhaseDeg,VariableNames={ ...
    'Channel','PeakRange_m','SNR_dB','Amplitude_dB','RelativePhase_deg'});

disp(' ');
disp('2-Tx x 2-Rx channel results:');
disp(channelResults);

%% Check Tx-to-Tx phase repeatability

txPhaseDifference = angle(H(:,2,:).*conj(H(:,1,:)));
txPhaseDifference = reshape(txPhaseDifference,nRx,nMimoRepeats);
phaseResultant = abs(mean(exp(1j*txPhaseDifference),2));
txPhaseStdDeg = rad2deg(sqrt(max(0,-2*log(max(phaseResultant,eps)))));
txPhaseMeanDeg = rad2deg(angle(mean(exp(1j*txPhaseDifference),2)));

phaseResults = table((1:nRx).',txPhaseMeanDeg,txPhaseStdDeg, ...
    VariableNames={'RxChannel','Tx2MinusTx1Phase_deg','CircularStd_deg'});
disp('Tx2-to-Tx1 phase repeatability:');
disp(phaseResults);

%% Display basic verification checks

snrPass = all(snrDb >= minimumSnrDb);
rangeSpread = max(peakRangeM)-min(peakRangeM);
rangePass = rangeSpread <= 2*rangeResolution;
phasePass = all(txPhaseStdDeg <= maximumPhaseStdDeg);

fprintf('Range calibration offset: %.3f m\n',calRange);
fprintf('Common target bin: %.3f m (calibrated)\n',commonTargetRange);
fprintf('Peak-range spread: %.3f m (suggested limit %.3f m): %s\n', ...
    rangeSpread,2*rangeResolution,passFail(rangePass));
fprintf('All channel SNRs >= %.1f dB: %s\n', ...
    minimumSnrDb,passFail(snrPass));
fprintf('Tx phase circular std <= %.1f deg: %s\n', ...
    maximumPhaseStdDeg,passFail(phasePass));

if snrPass && rangePass && phasePass
    fprintf('\nBasic slow-TDM MIMO verification: PASS\n');
else
    fprintf(['\nBasic slow-TDM MIMO verification: CHECK RESULTS\n' ...
        'A failure does not necessarily indicate a hardware fault. Check ' ...
        'antenna pointing, target gate, leakage and calibration.\n']);
end

%% Plot the four virtual-channel range profiles

figure('Name','Phaser slow-TDM MIMO verification');
tiledlayout(nTx,nRx,'TileSpacing','compact','Padding','compact');
globalPeak = max(meanRangePower(searchMask,:,:),[],'all');

for iTx = 1:nTx
    for iRx = 1:nRx
        nexttile;
        profileDb = 10*log10(max(meanRangePower(:,iRx,iTx),eps)/globalPeak);
        plot(rangeAxis,profileDb,'LineWidth',1.2);
        xline(commonTargetRange,'--r','Common target bin');
        grid on;
        xlim(targetRangeGate);
        ylim([-60 5]);
        title(sprintf('Tx%d - Rx%d',iTx,iRx));
        xlabel('Range (m)');
        ylabel('Relative power (dB)');
    end
end

%% Form and plot the range-azimuth profile
% Virtual array: for each (Tx,Rx) the virtual phase-center position is
% txPos(iTx) + rxPos(iRx). A conventional (Bartlett) beamformer sums the
% four virtual channels with the appropriate steering phase per angle.

rxPos = ((0:nRx-1) - (nRx-1)/2)*rxPhaseCenterSpacing;   % Rx phase centers (m)
txPos = ((0:nTx-1) - (nTx-1)/2)*txSpacing;              % Tx phase centers (m)

virtualPos = zeros(nChannels,1);
iChannel = 0;
for iTx = 1:nTx
    for iRx = 1:nRx
        iChannel = iChannel+1;
        virtualPos(iChannel) = txPos(iTx) + rxPos(iRx);
    end
end

% Coherently averaged complex range spectrum per channel (over repeats),
% ordered the same way as virtualPos (Tx-major, Rx-minor).
meanRangeComplex = mean(rangePerRepeat,4);              % range x Rx x Tx
channelSpectrum = zeros(numel(rangeAxis),nChannels);
iChannel = 0;
for iTx = 1:nTx
    for iRx = 1:nRx
        iChannel = iChannel+1;
        channelSpectrum(:,iChannel) = meanRangeComplex(:,iRx,iTx);
    end
end

% Steering matrix: nChannels x nAzimuth.
k = 2*pi/lambda;
steering = exp(1j*k*virtualPos*sind(azimuthGrid));

% Range-azimuth map (magnitude), then normalise to dB.
rangeAzimuth = abs(channelSpectrum*steering);
rangeAzimuthDb = 20*log10(max(rangeAzimuth,eps)/max(rangeAzimuth,[],'all'));

plotRangeMask = rangeAxis >= targetRangeGate(1) & ...
    rangeAxis <= targetRangeGate(2);

figure('Name','Phaser MIMO range-azimuth profile');
imagesc(azimuthGrid,rangeAxis(plotRangeMask), ...
    rangeAzimuthDb(plotRangeMask,:));
set(gca,'YDir','normal');
caxis([-30 0]);
colormap(jet);
cb = colorbar; cb.Label.String = 'Relative power (dB)';
xlabel('Azimuth angle (deg)');
ylabel('Range (m)');
title('MIMO range-azimuth profile (stationary target)');

% Report the estimated target azimuth at the common range bin.
[~,peakAzIdx] = max(rangeAzimuthDb(targetBin,:));
estimatedAzimuthDeg = azimuthGrid(peakAzIdx);
hold on;
plot(estimatedAzimuthDeg,commonTargetRange,'w+', ...
    'MarkerSize',12,'LineWidth',1.5);
hold off;
fprintf('Estimated target azimuth at %.2f m: %.1f deg\n', ...
    commonTargetRange,estimatedAzimuthDeg);

%% Release hardware

clear cleanupGuard;

%% Local helpers

function pulseData = arrangeMimoPulseData(rawData,rx,bf,bf_TDD)
% Preserve the two calibrated receive channels when extracting FMCW chirps.

    calibrationWeights = loadCalibrationWeights().DigitalWeights;
    calibrationWeights = reshape(conj(calibrationWeights),1,[]);
    assert(size(rawData,2) == numel(calibrationWeights), ...
        'The receive data and digital calibration channel counts differ.');
    calibratedData = rawData.*calibrationWeights;

    fs = rx.SamplingRate;
    tsweep = double(bf.FrequencyDeviationTime)/1e6;
    sweepStartTime = bf_TDD.Ch0On;
    pulseTime = bf_TDD.FrameLength/1e3;
    nPulses = bf_TDD.BurstCount;

    sweepOffsetSamples = ceil(sweepStartTime*fs);
    sweepSamples = (1:ceil(tsweep*fs))+sweepOffsetSamples;
    pulseEndSample = round(pulseTime*fs);
    pulseStartSamples = (0:nPulses-1)*pulseEndSample;
    sampleIndices = sweepSamples.'+pulseStartSamples;

    nRequiredSamples = max(sampleIndices,[],'all');
    if nRequiredSamples > size(calibratedData,1)
        error(['The Pluto returned %d samples, but %d are required. ' ...
            'Run the script again; the first hardware capture can be short.'], ...
            size(calibratedData,1),nRequiredSamples);
    end

    nSweep = size(sampleIndices,1);
    pulseData = zeros(nSweep,nPulses,size(calibratedData,2), ...
        'like',calibratedData);
    for iRx = 1:size(calibratedData,2)
        pulseData(:,:,iRx) = reshape(calibratedData(sampleIndices,iRx), ...
            nSweep,nPulses);
    end
end

function window = localHann(nSamples)
% Toolbox-independent periodic Hann window.
    if nSamples == 1
        window = 1;
    else
        window = 0.5-0.5*cos(2*pi*(0:nSamples-1).'/nSamples);
    end
end

function text = passFail(condition)
    if condition
        text = 'PASS';
    else
        text = 'CHECK';
    end
end