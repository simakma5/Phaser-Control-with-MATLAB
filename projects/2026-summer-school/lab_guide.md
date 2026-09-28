# MIMO Radar Systems Summer School 2026
## Lab Guide: Virtual Array Synthesis and Direction-of-Arrival (DOA) Estimation

**Microwave Sensing, Signals and Systems (MS3) Group, TU Delft**  
*Target Script:* [`mimo_lab.mlx`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/mimo-lab/mimo_lab.m) *(referenced as `mimo_lab.m`)*

---

## 1. Laboratory Objectives

This hands-on laboratory session provides practical experience with the core physical and algorithmic principles of Multiple-Input Multiple-Output (MIMO) radar using the **ADALM-PHASER (CN0566)** kit.

By completing this exercise, participants will be able to:
1. **Apply Spatial Convolution:** Synthesize virtual arrays via the spatial convolution of physical transmit ($\mathbf{r}_{\text{Tx}}$) and receive ($\mathbf{r}_{\text{Rx}}$) antenna locations:
   $$\mathbf{r}_{\text{virt}} = \mathbf{r}_{\text{Tx}} \oplus \mathbf{r}_{\text{Rx}} = \{ \mathbf{r}_t + \mathbf{r}_r \mid \mathbf{r}_t \in \mathcal{T},\, \mathbf{r}_r \in \mathcal{R} \}$$
2. **Observe the Rayleigh Resolution Limit:** Contrast the angular resolution of the physical 8-element Rx phased array (aperture $L = 3.5\lambda$, beamwidth $\approx 14.5^\circ$) with that of the synthesized 16-element virtual array ($L = 7.5\lambda$, beamwidth $\approx 6.8^\circ$), demonstrating how closely spaced targets ($\Delta\theta \approx 10^\circ$) transition from an unresolved mainlobe into two cleanly separated peaks.
3. **Control Switched TDM-MIMO Hardware:** Understand time-division multiplexed (TDM) switching using the Phaser's on-board microwave switch (`bf.EnableOut1`) and sequential element capture across the two ADAR1000 beamformer chips.
4. **Evaluate Array Topologies:** Investigate how varying transmit spacing ($d_{\text{Tx}} = 1.0\lambda$ overlapped vs. $4.0\lambda$ contiguous vs. $> 4.0\lambda$ sparse) affects the virtual aperture, co-array redundancy, and grating lobes.
5. **Study Spatial Tapering & Coherence:** Experiment with spatial windowing (Uniform vs. Hann vs. Chebyshev) and analyze how phase discrepancies between transmit paths impact virtual beam synthesis.

---

## 2. Overview of the Processing Pipeline

The script [`mimo_lab.m`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/mimo-lab/mimo_lab.m) executes a fully automated, 6-step pipeline:

```
[1. Array Geometry]      -> Computes and plots physical & virtual antenna baselines
[2. Radar Config]        -> Initializes PlutoSDR, Phaser ADF4159 PLL, and TDD engine
[3. Data Acquisition]    -> Executes 8-burst switched element capture for Tx 1 & Tx 2
[4. Range Processing]    -> Range FFT, coherent pulse averaging, automated target range locking
[5. Spatial Spectrum]    -> Digital beamforming, automated AOA peak finding & data-driven simulation
[6. 2D Heatmap Imaging]  -> Forms 16-channel Range-Azimuth profile
```

1. **Geometry Initialization:** Determines the physical and virtual coordinate vectors based on $d_{\text{Tx}}$ and plots the 3-panel geometry stem figure.
2. **FMCW Radar Setup:** Sets bandwidth, sweep time, PRF, and TDD triggers using the verified `setupFMCWRadar` abstraction, discarding the initial DMA buffer.
3. **Switched Single-Element Data Acquisition:** Cycles through 4 element pairs across the two ADAR1000 chips for Tx 1 and Tx 2 (8 rapid bursts total in ~1–2 seconds), collecting raw snapshots for all 16 virtual channels.
4. **Range FFT & Automated Target Gating:** Performs fast-time FFT windowed by Hann, averages pulses across the coherent processing interval (CPI), applies the hardware calibration offset ($R = R_{\text{raw}} - \text{calRange}$), and locks onto the reflector range bin within `targetRangeGate`.
5. **DOA Beamforming & Data-Driven Simulation:** Evaluates Bartlett spatial spectra for both physical (8 elements) and virtual (16 elements) arrays, automatically detects target peaks, and synthesizes a matched theoretical benchmark for side-by-side display.
6. **2D Range-Azimuth Map:** Generates a 2D intensity heatmap across range and azimuth.

---

## 3. Modular Function Reference

All non-interactive and hardware-control logic is encapsulated in standalone `.m` files residing in the same directory:

| Function File | Primary Role & Internal Operations |
|:---|:---|
| [`setupMimoFmcwRadar.m`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/mimo-lab/setupMimoFmcwRadar.m) | **Radar Configuration:** Derives FMCW sweep slope, PRF, and sampling rate; initializes PlutoSDR and Phaser TDD engine (`Ch2Off = FrameLength * nPulses`); applies broadside phase calibration weights to ADAR1000 chips; flushes initial frame. |
| [`captureMimoData.m`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/mimo-lab/captureMimoData.m) | **Hardware Data Acquisition:** Selects Tx 1/Tx 2 via `bf.EnableOut1`; cycles `bf.RxPowerDown` across 4 element pairs (`[1,5]`, `[2,6]`, `[3,7]`, `[4,8]`); captures bursts and arranges them into an `[nFastTime x nPulses x nRx x 2]` tensor. |
| [`arrangeMimoPulseData.m`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/mimo-lab/arrangeMimoPulseData.m) | **Pulse Slicing & Calibration:** Multiplies raw Pluto channels by digital calibration weights; computes sample offset indices from TDD timing; slices the continuous stream into discrete pulses. |
| [`processMimoRange.m`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/mimo-lab/processMimoRange.m) | **Range Profile & Gating:** Applies fast-time Hann window; computes next-power-of-2 FFT; coherently integrates chirps; applies `calRange = 1.6 m` offset; sums power across all channels to detect the target range bin. |
| [`calculateMimoSpatialSpectrum.m`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/mimo-lab/calculateMimoSpatialSpectrum.m) | **Spatial Processing & AOA Detection:** Extracts complex envelopes at the target range bin; applies spatial tapering (Uniform/Hann/Chebyshev); computes measured Bartlett spectra; detects target AOAs via `findSpatialPeaks`; synthesizes data-driven theoretical response. |
| [`findSpatialPeaks.m`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/mimo-lab/findSpatialPeaks.m) | **Peak Detection Utility:** Identifies local maxima in spatial spectra with specified minimum prominence (3 dB) and angular separation (4°), locking up to `targetCount` targets. |
| [`plotMimoArrayGeometry.m`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/mimo-lab/plotMimoArrayGeometry.m) | **Figure 1:** 3-panel stem plot of Physical Rx ($8\times \lambda/2$), Physical Tx ($2\times d_{\text{Tx}}$), and Virtual Array (including co-array redundancy weights and dynamic aperture scaling). |
| [`plotMimoRangeProfile.m`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/mimo-lab/plotMimoRangeProfile.m) | **Figure 2:** Calibrated range profile with an automated red marker indicating the detected reflector distance. |
| [`plotMimoSpatialSpectrum.m`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/mimo-lab/plotMimoSpatialSpectrum.m) | **Figure 3:** Side-by-side comparison of Simulated vs. Measured spatial spectra, showing physical (8 elements) and virtual (16 elements) responses with detected AOA markers. |
| [`plotMimoRangeAzimuth.m`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/mimo-lab/plotMimoRangeAzimuth.m) | **Figure 4:** 2D Range-Azimuth heatmap across the 16 virtual channels, showing target returns in range and angle. |
| [`localHann.m`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/mimo-lab/localHann.m) | **Window Utility:** Toolbox-independent periodic Hann window generator. |

---

## 4. Interactive Live Script Controls

When running [`mimo_lab.mlx`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/mimo-lab/mimo_lab.m), participants adjust parameters using interactive Live Editor controls in **Section 1**:

```matlab
% Array Mode: '2x8_switched' (16-element virtual ULA) or '2x2_subarray' (4-channel baseline)
arrayMode = '2x8_switched';

% Transmit antenna separation in wavelengths (lambda)
dTx_lambda = 4.0;            % 1.0: Overlapped, 4.0: Contiguous ULA, > 4.0: Sparse

% Number of target peaks to detect and simulate
targetCount = 2;

% Spatial tapering / windowing across array elements
spatialWindow = 'Uniform';   % 'Uniform', 'Hann', 'Chebyshev'
sllChebDb = -25;             % Chebyshev sidelobe level (dB)

% Tx phase calibration offset between Out 1 and Out 2 (degrees)
tx_phase_cal = 0.0;

% FMCW range calibration offset and search interval
calRange = 1.6;              % Hardware offset calibration (m)
targetRangeGate = [0.5, 5.0];% Target search interval (m)
```

### Participant Experiments:
1. **Resolution Doubling:** Run with `targetCount = 2` and `arrayMode = '2x8_switched'`. Observe how the two reflectors at $\approx \pm 14^\circ$ form an unresolved lump on the physical Rx curve (blue) but split into two narrow peaks on the virtual array curve (red).
2. **Topology Variations:** Change `dTx_lambda` to `1.0` to model the overlapped Tx bracket. Inspect Figure 1 to observe co-array redundancy weights, and examine how aperture size impacts beamwidth.
3. **Aperture Tapering:** Switch `spatialWindow` from `'Uniform'` to `'Chebyshev'` and adjust `sllChebDb` from `-15 dB` to `-35 dB`. Note the reduction in sidelobes at the cost of mainlobe widening.
4. **Tx Phase Errors:** Adjust `tx_phase_cal` using the slider. Observe how non-zero phase mismatches between the two transmit paths degrade virtual array coherence, introducing beam splitting and grating lobes.

---

## 5. Study Questions

1. **Aperture and Angular Resolution:**
   - From the measured spatial spectrum (Figure 3), determine the 3 dB beamwidth for both the physical 8-element array and the 16-element virtual array. How closely do they match theoretical values ($\theta_{3\text{dB}} \approx 50.8^\circ \lambda / L$)?
2. **Co-Array Redundancy:**
   - For an overlapped bracket with $d_{\text{Tx}} = 1.0\lambda$, how many unique spatial positions are formed? Why do redundant virtual elements increase SNR at those spatial frequencies?
3. **Grating Lobes in Sparse Arrays:**
   - If the two transmitters are placed at $d_{\text{Tx}} = 5.0\lambda$ (leaving a $1.0\lambda$ hole between the subarrays), at what azimuth angles do grating lobes appear?
4. **TDM Coherence Constraints:**
   - What happens to the synthesized virtual array response if a target moves radially during the 8-burst switching sequence? Calculate the Doppler phase shift $\Delta\phi = 4\pi v \Delta t / \lambda$ for a velocity of $0.5\text{ m/s}$ with a switching delay of $\Delta t = 100\text{ ms}$.
