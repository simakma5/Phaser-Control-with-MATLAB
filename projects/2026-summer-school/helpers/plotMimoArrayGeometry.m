function fig = plotMimoArrayGeometry(arrayParams)
% PLOTMIMOARRAYGEOMETRY Plot 3-panel stem diagram of physical Rx, physical Tx, and virtual array.
%
%   fig = plotMimoArrayGeometry(arrayParams)
%
%   Adapts dynamically to any physical Tx separation dTx_lambda (e.g. 1.0 for
%   overlapped array bracket, 4.0 for contiguous linear array bracket, or > 4.0
%   for sparse co-array / grating lobe demonstrations).

    x_rx   = arrayParams.x_rx;
    x_tx   = arrayParams.x_tx;
    x_virt = arrayParams.x_virt;
    lambda = arrayParams.lambda;
    dTx_lambda = arrayParams.dTx_lambda;

    nRxPhysical = length(x_rx);
    posLambda = x_virt / lambda;
    [uPos, ~, uIdx] = unique(round(posLambda, 4));
    weights = accumarray(uIdx, 1);

    xMax = max([9, ceil(max(posLambda) + 1)]);

    fig = figure('Name', 'MIMO array geometry', 'Position', [80, 100, 950, 500]);

    % Subplot 1: Physical Rx array
    subplot(3, 1, 1);
    stem(x_rx / lambda, ones(1, nRxPhysical), 'filled', 'LineWidth', 1.5, 'Color', [0 0.447 0.741]);
    xlim([-1, xMax]); ylim([0, 1.5]); grid on;
    title('Physical Rx array (8 elements, d = \lambda/2, aperture: 3.5\lambda)');
    xlabel('Position along baseline (\lambda)'); ylabel('Active');
    set(gca, 'YTick', [0, 1]);

    % Subplot 2: Physical Tx positions
    subplot(3, 1, 2);
    stem(x_tx / lambda, ones(1, 2), 'filled', 'LineWidth', 1.5, 'Color', [0.85 0.325 0.098]);
    xlim([-1, xMax]); ylim([0, 1.5]); grid on;
    title(sprintf('Physical Tx positions (2 Vivaldi antennas, d_{Tx} = %.2f\\lambda)', dTx_lambda));
    xlabel('Position along baseline (\lambda)'); ylabel('Active');
    set(gca, 'YTick', [0, 1]);

    % Subplot 3: Synthesized virtual array
    subplot(3, 1, 3);
    yMax = max([1.5, max(weights) + 0.5]);
    stem(uPos, weights, 'filled', 'LineWidth', 1.5, 'Color', [0.466 0.674 0.188]);
    xlim([-1, xMax]); ylim([0, yMax]); grid on;

    if abs(dTx_lambda - 4.0) < 1e-3
        title(sprintf('Synthesized virtual array (16 elements contiguous ULA, d = \\lambda/2, aperture: %.1f\\lambda)', max(posLambda)));
        ylabel('Active');
        set(gca, 'YTick', [0, 1]);
    elseif dTx_lambda < 4.0
        title(sprintf('Synthesized virtual array (overlapped aperture: %.1f\\lambda, %d unique positions)', max(posLambda), length(uPos)));
        ylabel('Co-array weight');
        set(gca, 'YTick', 0:max(weights));
    else
        title(sprintf('Synthesized virtual array (sparse array with grating lobes, aperture: %.1f\\lambda)', max(posLambda)));
        ylabel('Active');
        set(gca, 'YTick', [0, 1]);
    end
    xlabel('Position along baseline (\lambda)');
end
