function fig = plotMimoRangeAzimuth(rangeResults, spatialResults, doaParams)
% PLOTMIMORANGEAZIMUTH Generate 2D Range-Azimuth heatmap across the 16 virtual channels.
%
%   fig = plotMimoRangeAzimuth(rangeResults, spatialResults, doaParams)

    if nargin < 3, doaParams = spatialResults.doaParams; end

    rangeAxis         = rangeResults.rangeAxis;
    targetRangeGate   = rangeResults.targetRangeGate;
    meanRangeSpectrum = rangeResults.meanRangeSpectrum;
    azimuthGrid       = spatialResults.azimuthGrid;
    A_virt_grid       = spatialResults.A_virt_grid;
    tx_phase_cal      = doaParams.tx_phase_cal;

    % Only applicable when all 16 virtual elements are present
    if size(meanRangeSpectrum, 2) < 8
        fig = [];
        return;
    end

    channelRangeSpectrum = zeros(numel(rangeAxis), 16);
    channelRangeSpectrum(:, 1:8)  = squeeze(meanRangeSpectrum(:, :, 1));
    channelRangeSpectrum(:, 9:16) = squeeze(meanRangeSpectrum(:, :, 2)) * exp(-1j * deg2rad(tx_phase_cal));

    rangeAzimuth = abs(channelRangeSpectrum * conj(A_virt_grid)); % [nRange x nAngles]
    rangeAzimuthDb = 20 * log10(max(rangeAzimuth, eps) / max(rangeAzimuth, [], 'all'));

    plotMask = rangeAxis >= targetRangeGate(1) & rangeAxis <= targetRangeGate(2);

    fig = figure('Name', 'Range-Azimuth Profile', 'Position', [160, 150, 700, 480]);
    imagesc(azimuthGrid, rangeAxis(plotMask), rangeAzimuthDb(plotMask, :));
    set(gca, 'YDir', 'normal');
    colormap(jet);
    cb = colorbar; cb.Label.String = 'Relative Power (dB)';
    caxis([-30, 0]);
    xlabel('Azimuth Angle (deg)'); ylabel('Range (m)');
    title('MIMO Range-Azimuth Map (16 Virtual Elements)');
    grid on;
end
