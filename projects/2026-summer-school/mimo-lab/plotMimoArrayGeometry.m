function fig = plotMimoArrayGeometry(arrayParams)
% PLOTMIMOARRAYGEOMETRY Plot 3-panel stem diagram of physical Rx, physical Tx, and virtual array.
%
%   fig = plotMimoArrayGeometry(arrayParams)

    x_rx   = arrayParams.x_rx;
    x_tx   = arrayParams.x_tx;
    x_virt = arrayParams.x_virt;
    lambda = arrayParams.lambda;

    nRxPhysical = length(x_rx);
    nVirt = length(x_virt);

    fig = figure('Name', 'MIMO Array Geometry', 'Position', [80, 100, 950, 480]);

    subplot(3, 1, 1);
    stem(x_rx / lambda, ones(1, nRxPhysical), 'filled', 'LineWidth', 1.5, 'Color', [0 0.447 0.741]);
    xlim([-1, 9]); ylim([0, 1.5]); grid on;
    title('Physical Rx Array (8 elements, d = \lambda/2)');
    xlabel('Position along baseline (\lambda)'); ylabel('Active');
    set(gca, 'YTick', [0, 1]);

    subplot(3, 1, 2);
    stem(x_tx / lambda, ones(1, 2), 'filled', 'LineWidth', 1.5, 'Color', [0.85 0.325 0.098]);
    xlim([-1, 9]); ylim([0, 1.5]); grid on;
    title('Physical Tx Positions (2 Vivaldi antennas, d_{Tx} = 4\lambda = 8\cdot d_{Rx})');
    xlabel('Position along baseline (\lambda)'); ylabel('Active');
    set(gca, 'YTick', [0, 1]);

    subplot(3, 1, 3);
    stem(x_virt / lambda, ones(1, nVirt), 'filled', 'LineWidth', 1.5, 'Color', [0.466 0.674 0.188]);
    xlim([-1, 9]); ylim([0, 1.5]); grid on;
    title('Synthesized Virtual Array (16 elements contiguous, d = \lambda/2)');
    xlabel('Position along baseline (\lambda)'); ylabel('Active');
    set(gca, 'YTick', [0, 1]);
end
