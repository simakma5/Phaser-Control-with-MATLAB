%% Antenna Pattern & Direction of Arrival (DOA) Lab
clear; close all;
warning('off','MATLAB:system:ObsoleteSystemObjectMixin');

% Load calibration weights and frequency
load('CalibrationWeights.mat','calibrationweights');
load('HB100_Fc.mat','fc_hb100');

% Create Antenna Interactor
ai = AntennaInteractor(fc_hb100,calibrationweights);

% Array Parameters
lambda = freq2wavelen(10.7e9);
d = lambda/2;
nElements = 8;
element_idx = 0:(nElements-1);

% Scan Angles for DOA
scanAngles = -90:1:90;
nAngles = length(scanAngles);
spatial_spectrum = nan(1,nAngles);

figure('Name', 'DOA Spatial Spectrum');
ax = axes;
patLine = plot(ax, scanAngles, spatial_spectrum, 'LineWidth', 2);
xlim(ax, [-90 90]);
title(ax, 'Spatial Spectrum (DOA)');
xlabel(ax, 'Azimuth Angle (degrees)');
ylabel(ax, 'Magnitude (dB)');
grid on;

% Collect data across all scan angles
for i = 1:nAngles
    % --- DSP LOGIC (SOLUTION) ---
    % Calculate the phase shift for each element to steer the beam
    phase_shifts = -2 * pi * d * element_idx * sind(scanAngles(i)) / lambda;
    steerweights = exp(1j * phase_shifts).';
    % ----------------------------------------------
    
    if exist('steerweights','var') && length(steerweights) == 8
        steerweights = steerweights(:); % force column vector
        analogsteer = analogWeightsCalAdjustment([steerweights(1:4) steerweights(5:8)], calibrationweights.AnalogWeights);
        data = ai.steerAnalog(analogsteer);
        digitalsteer = digitalWeightsCalAdjustment([1;1], calibrationweights.DigitalWeights);
        data = data * conj(digitalsteer);
        
        % Plot the updated pattern
        spatial_spectrum(i) = mag2db(helperGetAmplitude(data));
        patLine.YData = spatial_spectrum;
        drawnow;
    end
end

cleanup(ai);
