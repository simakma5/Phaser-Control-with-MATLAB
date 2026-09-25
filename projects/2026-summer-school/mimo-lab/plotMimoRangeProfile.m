function fig = plotMimoRangeProfile(rangeResults)
% PLOTMIMORANGEPROFILE Plot calibrated range profile and indicate detected target distance.
%
%   fig = plotMimoRangeProfile(rangeResults)

    rangeAxis       = rangeResults.rangeAxis;
    combinedPower   = rangeResults.combinedPower;
    targetRangeGate = rangeResults.targetRangeGate;
    targetRangeMeas = rangeResults.targetRangeMeas;

    fig = figure('Name', 'Calibrated Range Profile', 'Position', [80, 150, 650, 380]);
    profileSumDb = 10 * log10(max(combinedPower, eps) / max(combinedPower));
    plot(rangeAxis, profileSumDb, 'LineWidth', 1.5);
    hold on;
    xline(targetRangeMeas, '--r', sprintf('Target Range: %.2f m', targetRangeMeas), ...
          'LabelVerticalAlignment', 'bottom', 'LineWidth', 1.2);
    grid on; xlim(targetRangeGate); ylim([-40, 5]);
    xlabel('Range (m)'); ylabel('Normalized Power (dB)');
    title('Calibrated Range Profile');
end
