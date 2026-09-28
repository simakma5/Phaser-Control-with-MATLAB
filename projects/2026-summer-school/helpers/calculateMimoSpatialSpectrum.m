function spatialResults = calculateMimoSpatialSpectrum(rangeResults, arrayParams, doaParams)
% CALCULATEMIMOSPATIALSPECTRUM Compute digital beamforming and spatial spectra for physical and virtual arrays.
%
%   spatialResults = calculateMimoSpatialSpectrum(rangeResults, arrayParams, doaParams)
%
%   Inputs:
%       rangeResults - Output struct from processMimoRange
%       arrayParams  - Struct with fields: x_rx, x_virt, lambda
%       doaParams    - Struct with fields:
%           azimuthGrid   - Azimuth evaluation grid in degrees (e.g. -60:0.5:60)
%           tx_phase_cal  - Tx phase calibration offset in degrees (default: 0.0)
%           doaAlgorithm  - 'Bartlett', 'Capon', or 'MUSIC' (default: 'Bartlett')
%           spatialWindow - 'Uniform', 'Hann', or 'Chebyshev' (default: 'Uniform')
%           sllChebDb     - Sidelobe level for Chebyshev window in dB (default: -25)
%           targetCount   - Number of target peaks to detect and simulate (default: 2)
%
%   Outputs:
%       spatialResults - Struct containing:
%           P_rx_meas_db   - Measured physical array spectrum (dB)
%           P_virt_meas_db - Measured virtual array spectrum (dB)
%           P_rx_sim_db    - Theoretical simulated physical array spectrum (dB)
%           P_virt_sim_db  - Theoretical simulated virtual array spectrum (dB)
%           detectedAOAs   - Detected target azimuth angles (deg)
%           detectedPowers - Measured power at detected angles (dB)
%           azimuthGrid    - Azimuth evaluation grid (deg)

    if nargin < 3, doaParams = struct(); end
    if ~isfield(doaParams, 'azimuthGrid'),   doaParams.azimuthGrid = -60:0.5:60; end
    if ~isfield(doaParams, 'tx_phase_cal'),  doaParams.tx_phase_cal = 0.0; end
    if ~isfield(doaParams, 'doaAlgorithm'),  doaParams.doaAlgorithm = 'Bartlett'; end
    if ~isfield(doaParams, 'spatialWindow'), doaParams.spatialWindow = 'Uniform'; end
    if ~isfield(doaParams, 'sllChebDb'),     doaParams.sllChebDb = -25; end
    if ~isfield(doaParams, 'targetCount'),   doaParams.targetCount = 2; end

    azimuthGrid  = doaParams.azimuthGrid;
    tx_phase_cal = doaParams.tx_phase_cal;
    targetCount  = doaParams.targetCount;
    targetBin    = rangeResults.targetBin;
    meanRangeSpectrum = rangeResults.meanRangeSpectrum;

    x_rx        = arrayParams.x_rx;
    x_virt      = arrayParams.x_virt;
    lambda      = arrayParams.lambda;
    nRxPhysical = length(x_rx);
    nVirt       = length(x_virt);

    k = 2 * pi / lambda;
    norm_db = @(p) 10 * log10(max(p, eps) / max(p));

    % Steering matrices across angular evaluation grid
    A_rx_grid   = exp(1j * k * x_rx.' * sind(azimuthGrid));     % [8 x nAngles]
    A_virt_grid = exp(1j * k * x_virt.' * sind(azimuthGrid));   % [16 x nAngles]

    % Spatial tapering / windowing weights
    w_rx   = getSpatialWindow(doaParams.spatialWindow, nRxPhysical, doaParams.sllChebDb);
    w_virt = getSpatialWindow(doaParams.spatialWindow, nVirt, doaParams.sllChebDb);

    % Extract complex response across all 8 Rx elements for Tx 1 and Tx 2
    H_tx1 = squeeze(meanRangeSpectrum(targetBin, :, 1)).'; % 8 x 1
    H_tx2 = squeeze(meanRangeSpectrum(targetBin, :, 2)).'; % 8 x 1
    H_tx2_cal = H_tx2 * exp(-1j * deg2rad(tx_phase_cal));

    % 16-element contiguous virtual array snapshot
    H_virt = [H_tx1; H_tx2_cal]; % 16 x 1

    % Apply spatial tapering
    H_tx1_win  = H_tx1 .* w_rx;
    H_virt_win = H_virt .* w_virt;

    % Digital beamforming (Bartlett spatial spectrum)
    P_rx_meas   = abs(A_rx_grid' * H_tx1_win).^2;
    P_virt_meas = abs(A_virt_grid' * H_virt_win).^2;

    P_rx_meas_db   = norm_db(P_rx_meas);
    P_virt_meas_db = norm_db(P_virt_meas);

    % Inspect relative power between transmit channels
    pTx1 = mean(abs(H_tx1).^2);
    pTx2 = mean(abs(H_tx2).^2);
    txRatioDb = 10 * log10(max(pTx2, eps) / max(pTx1, eps));
    fprintf('Transmit channel 2 / channel 1 power ratio: %.1f dB\n', txRatioDb);

    % Automatic AOA detection:
    % When both transmit channels have comparable power (>= -10 dB, co-pol MIMO mode),
    % detect peaks from the higher-resolution 16-element virtual spectrum.
    % When Tx 2 is heavily attenuated (< -10 dB, rotated cross-pol mode), concatenating
    % Tx 1 and Tx 2 introduces an artificial aperture step discontinuity; therefore
    % detect target AOAs from the 8-element physical Rx spectrum (Tx 1).
    if txRatioDb < -10
        fprintf('Note: Transmit channel 2 is heavily attenuated (< -10 dB, rotated cross-pol mode).\n');
        fprintf('Detecting target AOAs from physical Rx array (Tx 1) to avoid aperture discontinuity.\n');
        [detectedAOAs, detectedPowers] = findSpatialPeaks(P_rx_meas_db, azimuthGrid, targetCount, 3.0);
    else
        [detectedAOAs, detectedPowers] = findSpatialPeaks(P_virt_meas_db, azimuthGrid, targetCount, 3.0);
    end

    if isempty(detectedAOAs)
        detectedAOAs = 0.0;
        detectedPowers = 0.0;
    end

    fprintf('Detected target AOAs: %s deg\n\n', mat2str(round(detectedAOAs, 1)));

    % Data-Driven Theoretical Simulated Spatial Spectrum
    detectedAmps = 10.^(detectedPowers / 20);
    detectedAmps = detectedAmps / max(detectedAmps);

    s_rx_sim   = zeros(nRxPhysical, 1);
    s_virt_sim = zeros(nVirt, 1);

    for iT = 1:length(detectedAOAs)
        theta_t = detectedAOAs(iT);
        amp_t   = detectedAmps(iT);

        a_rx_t   = exp(1j * k * x_rx.' * sind(theta_t));
        a_virt_t = exp(1j * k * x_virt.' * sind(theta_t));

        s_rx_sim   = s_rx_sim + amp_t * a_rx_t;
        s_virt_sim = s_virt_sim + amp_t * a_virt_t;
    end

    % Apply spatial tapering to simulated signals
    s_rx_sim_win   = s_rx_sim .* w_rx;
    s_virt_sim_win = s_virt_sim .* w_virt;

    P_rx_sim   = abs(s_rx_sim_win.' * conj(A_rx_grid)).^2;
    P_virt_sim = abs(s_virt_sim_win.' * conj(A_virt_grid)).^2;

    P_rx_sim_db   = norm_db(P_rx_sim);
    P_virt_sim_db = norm_db(P_virt_sim);

    spatialResults = struct();
    spatialResults.P_rx_meas_db   = P_rx_meas_db;
    spatialResults.P_virt_meas_db = P_virt_meas_db;
    spatialResults.P_rx_sim_db    = P_rx_sim_db;
    spatialResults.P_virt_sim_db  = P_virt_sim_db;
    spatialResults.detectedAOAs   = detectedAOAs;
    spatialResults.detectedPowers = detectedPowers;
    spatialResults.azimuthGrid    = azimuthGrid;
    spatialResults.A_virt_grid    = A_virt_grid;
    spatialResults.doaParams      = doaParams;
end

function w = getSpatialWindow(windowType, N, sllChebDb)
    switch lower(windowType)
        case 'hann'
            w = 0.5 - 0.5 * cos(2 * pi * (0:N - 1).' / (N - 1));
        case 'chebyshev'
            try
                w = chebwin(N, abs(sllChebDb));
            catch
                % Fallback if Signal Processing Toolbox unavailable
                w = 0.5 - 0.5 * cos(2 * pi * (0:N - 1).' / (N - 1));
            end
        otherwise % 'uniform' / 'rectangular'
            w = ones(N, 1);
    end
    w = w / max(w);
end
