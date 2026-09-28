function fig = plotMimoSpatialSpectrum(spatialResults)
% PLOTMIMOSPATIALSPECTRUM Side-by-side comparison of simulated and measured spatial spectra.
%
%   fig = plotMimoSpatialSpectrum(spatialResults)

    azimuthGrid    = spatialResults.azimuthGrid;
    P_rx_sim_db    = spatialResults.P_rx_sim_db;
    P_virt_sim_db  = spatialResults.P_virt_sim_db;
    P_rx_meas_db   = spatialResults.P_rx_meas_db;
    P_virt_meas_db = spatialResults.P_virt_meas_db;
    detectedAOAs   = spatialResults.detectedAOAs;

    fig = figure('Name', 'Spatial spectrum comparison', 'Position', [120, 150, 1150, 480]);

    % Left subplot: Simulated spatial spectrum (driven by detected AOAs)
    subplot(1, 2, 1);
    plot(azimuthGrid, P_rx_sim_db,   'LineWidth', 1.8, 'DisplayName', 'Physical Rx (8 elements, 3.5\lambda)'); hold on;
    plot(azimuthGrid, P_virt_sim_db, 'LineWidth', 1.8, 'DisplayName', 'Virtual array (16 elements, 7.5\lambda)');
    for iT = 1:length(detectedAOAs)
        xline(detectedAOAs(iT), ':k', sprintf('%.1f^\\circ', detectedAOAs(iT)), ...
              'LabelVerticalAlignment', 'top', 'HandleVisibility', 'off');
    end
    grid on; xlim([min(azimuthGrid), max(azimuthGrid)]); ylim([-35, 2]);
    xlabel('Azimuth angle (deg)'); ylabel('Normalised power (dB)');
    title(sprintf('Simulated spectrum (AOAs: %s^\\circ)', mat2str(round(detectedAOAs, 1))));
    legend('Location', 'south');

    % Right subplot: Measured spatial spectrum
    subplot(1, 2, 2);
    plot(azimuthGrid, P_rx_meas_db,   'LineWidth', 1.8, 'DisplayName', 'Physical Rx (8 elements, 3.5\lambda)'); hold on;
    plot(azimuthGrid, P_virt_meas_db, 'LineWidth', 1.8, 'DisplayName', 'Virtual array (16 elements, 7.5\lambda)');
    for iT = 1:length(detectedAOAs)
        xline(detectedAOAs(iT), ':k', sprintf('%.1f^\\circ', detectedAOAs(iT)), ...
              'LabelVerticalAlignment', 'top', 'HandleVisibility', 'off');
    end
    grid on; xlim([min(azimuthGrid), max(azimuthGrid)]); ylim([-35, 2]);
    xlabel('Azimuth angle (deg)'); ylabel('Normalised power (dB)');
    title('Measured spatial spectrum');
    legend('Location', 'south');
end
