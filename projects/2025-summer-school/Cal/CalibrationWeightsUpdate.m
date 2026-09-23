
% Load the calibration weights. 
folder_cal = 'C:\Users\LocalAdmin\Desktop\Phaser lab\Phaser-Control-with-MATLAB-main\shared\calibration';
file_cal = 'CalibrationWeights.mat'; 
load([folder_cal filesep file_cal]);
calibrationweightsoriginal = calibrationweights; 

% Obtain AnalogWeights and DigitalWeights. 
AnalogWeights = calibrationweights.AnalogWeights; 
DigitalWeights = calibrationweights.DigitalWeights; 

% Phase only. 
AnalogWeightsPhaseonly = exp(1j*angle(AnalogWeights)); 
DigitalWeightsPhaseonly = exp(1j*angle(DigitalWeights)); 

% Amplitude only. 
AnalogWeightsAmplitudeonly = abs(AnalogWeights); 
DigitalWeightsAmplitudeonly = abs(DigitalWeights); 

% Store results phase only. 
calibrationweights = calibrationweightsoriginal; 
calibrationweights.AnalogWeights = AnalogWeightsPhaseonly; 
calibrationweights.DigitalWeights = DigitalWeightsPhaseonly; 
save('CalibrationWeightsPhaseonly.mat', 'calibrationweights'); 

% Store results amplitude only. 
calibrationweights = calibrationweightsoriginal; 
calibrationweights.AnalogWeights = AnalogWeightsAmplitudeonly; 
calibrationweights.DigitalWeights = DigitalWeightsAmplitudeonly; 
save('CalibrationWeightsAmplitudeonly.mat', 'calibrationweights'); 
