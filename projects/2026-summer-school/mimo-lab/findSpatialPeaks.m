function [detectedAngles, peakPowers] = findSpatialPeaks(spectrumDb, angleGrid, maxPeaks, minProminence, minSeparation)
% FINDSPATIALPEAKS Identify prominent local maxima in a spatial DOA spectrum.
%
%   [detectedAngles, peakPowers] = findSpatialPeaks(spectrumDb, angleGrid, maxPeaks, minProminence, minSeparation)
%
%   Inputs:
%       spectrumDb    - 1D array of normalized spatial spectrum values in dB (peak at 0 dB)
%       angleGrid     - 1D array of corresponding azimuth angles in degrees
%       maxPeaks      - Maximum number of target peaks to detect (default: 2)
%       minProminence - Minimum height above local floor in dB (default: 3 dB)
%       minSeparation - Minimum angular separation between detected peaks in deg (default: 4.0 deg)
%
%   Outputs:
%       detectedAngles - Detected target azimuth angles in degrees (sorted ascending)
%       peakPowers     - Spectrum power at the detected angles in dB

    if nargin < 3, maxPeaks = 2; end
    if nargin < 4, minProminence = 3; end
    if nargin < 5, minSeparation = 4.0; end

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

    % Sort by power descending
    [~, sortIdx] = sort(peakVals, 'descend');
    peakIdx = peakIdx(sortIdx);

    % Enforce minimum angular separation between targets
    selectedIdx = [];
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
