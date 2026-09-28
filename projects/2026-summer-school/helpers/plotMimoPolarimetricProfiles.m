function fig = plotMimoPolarimetricProfiles(rangeResults)
% PLOTMIMOPOLARIMETRICPROFILES Plot all 8 Rx range profiles for each Tx side-by-side.
%
%   fig = plotMimoPolarimetricProfiles(rangeResults)
%
%   Generates a 2-panel comparison figure displaying the 8 Rx channel range
%   profiles for Tx 1 (SMA Out 1) and Tx 2 (SMA Out 2). This facilitates
%   polarimetric observation when one Tx antenna is rotated by 90 degrees
%   (e.g., Tx 1 = Co-Pol, Tx 2 = Cross-Pol). Both panels share a common
%   normalization reference to preserve the relative power difference.

    rangeAxis       = rangeResults.rangeAxis;
    meanRangePower  = rangeResults.meanRangePower;
    targetRangeGate = rangeResults.targetRangeGate;
    targetRangeMeas = rangeResults.targetRangeMeas;

    searchMask = rangeAxis >= targetRangeGate(1) & rangeAxis <= targetRangeGate(2);
    globalPeakPower = max(meanRangePower(searchMask, :, :), [], 'all');

    fig = figure('Name', 'Tx Polarimetric Range Profiles (All 8 Rx Channels)', ...
                 'Position', [80, 100, 1300, 520]);

    % Subplot 1: Tx 1 (SMA Out 1) - Co-Pol / H
    subplot(1, 2, 1);
    hold on;
    for iRx = 1:8
        chPowerDb = 10 * log10(max(meanRangePower(:, iRx, 1), eps) / globalPeakPower);
        plot(rangeAxis, chPowerDb, 'LineWidth', 1.2, 'DisplayName', sprintf('Rx %d', iRx));
    end
    xline(targetRangeMeas, '--r', sprintf('Target: %.2f m', targetRangeMeas), ...
          'HandleVisibility', 'off', 'LineWidth', 1.2);
    grid on;
    xlim(targetRangeGate);
    ylim([-50, 5]);
    xlabel('Range (m)');
    ylabel('Relative Power (dB)');
    title('Tx 1 (SMA Out 1) - All 8 Rx Channels (Co-Pol)');
    legend('Location', 'northeast', 'NumColumns', 2);

    % Subplot 2: Tx 2 (SMA Out 2) - Cross-Pol / V (Rotated 90 deg)
    subplot(1, 2, 2);
    hold on;
    for iRx = 1:8
        chPowerDb = 10 * log10(max(meanRangePower(:, iRx, 2), eps) / globalPeakPower);
        plot(rangeAxis, chPowerDb, 'LineWidth', 1.2, 'DisplayName', sprintf('Rx %d', iRx));
    end
    xline(targetRangeMeas, '--r', sprintf('Target: %.2f m', targetRangeMeas), ...
          'HandleVisibility', 'off', 'LineWidth', 1.2);
    grid on;
    xlim(targetRangeGate);
    ylim([-50, 5]);
    xlabel('Range (m)');
    ylabel('Relative Power (dB)');
    title('Tx 2 (SMA Out 2) - All 8 Rx Channels (Cross-Pol)');
    legend('Location', 'northeast', 'NumColumns', 2);
end
