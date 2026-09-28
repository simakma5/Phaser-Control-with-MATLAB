function fig = plotMimoTxRangeProfiles(rangeResults)
% PLOTMIMOTXRANGEPROFILES Plot all eight receive range profiles for each transmit channel.
%
%   fig = plotMimoTxRangeProfiles(rangeResults)
%
%   Generates a two-panel comparison figure displaying the eight receive channel
%   range profiles for transmit channel 1 (SMA out 1) and transmit channel 2
%   (SMA out 2). Both panels share a common normalization reference to preserve
%   relative power differences between transmit channels, enabling observations
%   such as polarimetric co-pol and cross-pol comparisons when one transmit
%   antenna is rotated by 90 degrees.

    rangeAxis       = rangeResults.rangeAxis;
    meanRangePower  = rangeResults.meanRangePower;
    targetRangeGate = rangeResults.targetRangeGate;
    targetRangeMeas = rangeResults.targetRangeMeas;

    searchMask = rangeAxis >= targetRangeGate(1) & rangeAxis <= targetRangeGate(2);
    globalPeakPower = max(meanRangePower(searchMask, :, :), [], 'all');

    fig = figure('Name', 'Range profiles by transmit channel', ...
                 'Position', [80, 100, 1300, 520]);

    % Subplot 1: Transmit channel 1 (SMA out 1)
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
    ylabel('Relative power (dB)');
    title('Transmit channel 1 (SMA out 1)');
    legend('Location', 'northeast', 'NumColumns', 2);

    % Subplot 2: Transmit channel 2 (SMA out 2)
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
    ylabel('Relative power (dB)');
    title('Transmit channel 2 (SMA out 2)');
    legend('Location', 'northeast', 'NumColumns', 2);
end
