# MIMO radar systems summer school 2026
## Lab guide: virtual array synthesis and direction-of-arrival (DOA) estimation

**Microwave Sensing, Signals and Systems (MS3) group, TU Delft**  
*Target script:* [`mimo_lab.mlx`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/mimo_lab.m) *(referenced as `mimo_lab.m`)*

---

## 1. Laboratory objectives

This hands-on laboratory session provides practical experience with the core physical and algorithmic principles of Multiple-Input Multiple-Output (MIMO) radar using the **ADALM-PHASER (CN0566)** kit.

By completing this exercise, participants will be able to:
1. **Apply spatial convolution:** Synthesize virtual arrays via the spatial convolution of physical transmit ($\mathbf{r}_{\text{Tx}}$) and receive ($\mathbf{r}_{\text{Rx}}$) antenna locations:
   $$\mathbf{r}_{\text{virt}} = \mathbf{r}_{\text{Tx}} \oplus \mathbf{r}_{\text{Rx}} = \{ \mathbf{r}_t + \mathbf{r}_r \mid \mathbf{r}_t \in \mathcal{T},\, \mathbf{r}_r \in \mathcal{R} \}$$
2. **Observe the Rayleigh resolution limit:** Contrast the angular resolution of the physical eight-element Rx phased array (aperture $L = 3.5\lambda$, beamwidth $\approx 14.5^\circ$) with that of the synthesized 16-element virtual array ($L = 7.5\lambda$, beamwidth $\approx 6.8^\circ$), demonstrating how closely spaced targets ($\Delta\theta \approx 10^\circ$) transition from an unresolved mainlobe into two cleanly separated peaks.
3. **Control switched TDM-MIMO hardware:** Understand time-division multiplexed (TDM) switching using the Phaser's on-board microwave switch (`bf.EnableOut1`) and sequential element capture across the two ADAR1000 beamformer chips.
4. **Evaluate array topologies:** Investigate how varying transmit spacing ($d_{\text{Tx}} = 1.0\lambda$ overlapped vs. $4.0\lambda$ contiguous vs. $> 4.0\lambda$ sparse) affects the virtual aperture, co-array redundancy, and grating lobes.
5. **Study spatial tapering and coherence:** Experiment with spatial windowing (Uniform vs Hann vs Chebyshev) and analyse how phase discrepancies between transmit paths impact virtual beam synthesis.
6. **Observe transmit channel returns (polarimetry):** Inspect returns separated by transmit channel. Rotating one transmit Vivaldi antenna by 90° enables simultaneous capture of co-polarized and cross-polarized returns across all eight receive elements, allowing target scattering matrices (e.g. dihedral vs trihedral corner reflectors) to be analysed.

---

## 2. Overview of the processing pipeline

The script [`mimo_lab.m`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/mimo_lab.m) executes a fully automated, seven-step pipeline:

```
[1. Array geometry]          -> Computes and plots physical and virtual antenna baselines
[2. Radar configuration]     -> Initializes PlutoSDR, Phaser ADF4159 PLL, and TDD engine
[3. Data acquisition]        -> Executes eight-burst switched element capture for Tx 1 and Tx 2
[4. Range processing]        -> Range FFT, coherent pulse averaging, automated target range locking
[5. Spatial spectrum]        -> Digital beamforming, automated AOA peak finding and data-driven simulation
[6. 2D heatmap imaging]      -> Forms 16-channel range-azimuth profile
[7. Transmit range profiles] -> Overlays all eight Rx channel range profiles for Tx 1 vs Tx 2
```

1. **Geometry initialization:** Determines the physical and virtual coordinate vectors based on $d_{\text{Tx}}$ and plots the three-panel geometry stem figure.
2. **FMCW radar setup:** Sets bandwidth, sweep time, PRF, and TDD triggers using the verified `setupFMCWRadar` abstraction, discarding the initial DMA buffer.
3. **Switched single-element data acquisition:** Cycles through four element pairs across the two ADAR1000 chips for Tx 1 and Tx 2 (eight rapid bursts total in ~1–2 seconds), collecting raw snapshots for all 16 virtual channels.
4. **Range FFT and automated target gating:** Performs fast-time FFT windowed by Hann, averages pulses across the coherent processing interval (CPI), applies the hardware calibration offset ($R = R_{\text{raw}} - \text{calRange}$), and locks onto the reflector range bin within `targetRangeGate`.
5. **DOA beamforming and data-driven simulation:** Evaluates Bartlett spatial spectra for both physical (8 elements) and virtual (16 elements) arrays, automatically detects target peaks, and synthesizes a matched theoretical benchmark for side-by-side display.
6. **2D range-azimuth map:** Generates a 2D intensity heatmap across range and azimuth.
7. **Range profiles by transmit channel:** Displays all eight Rx range profiles for Tx 1 and Tx 2 side by side with a shared normalization scale (enabling co-pol vs cross-pol observation when Tx 2 is rotated 90°).

---

## 3. Modular function reference

All non-interactive and hardware-control logic is encapsulated in standalone `.m` files residing in the [`helpers/`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/helpers/) folder:

| Function file | Primary role and internal operations |
|:---|:---|
| [`setupMimoFmcwRadar.m`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/helpers/setupMimoFmcwRadar.m) | **Radar configuration:** Derives FMCW sweep slope, PRF, and sampling rate; initializes PlutoSDR and Phaser TDD engine (`Ch2Off = FrameLength * nPulses`); applies broadside phase calibration weights to ADAR1000 chips; flushes initial frame. |
| [`captureMimoData.m`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/helpers/captureMimoData.m) | **Hardware data acquisition:** Selects Tx 1/Tx 2 via `bf.EnableOut1`; cycles `bf.RxPowerDown` across four element pairs (`[1,5]`, `[2,6]`, `[3,7]`, `[4,8]`); captures bursts and arranges them into an `[nFastTime x nPulses x 8 x 2]` tensor. |
| [`arrangeMimoPulseData.m`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/helpers/arrangeMimoPulseData.m) | **Pulse slicing and calibration:** Multiplies raw Pluto channels by digital calibration weights; computes sample offset indices from TDD timing; slices the continuous stream into discrete pulses. |
| [`processMimoRange.m`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/helpers/processMimoRange.m) | **Range profile and gating:** Applies fast-time Hann window; computes next-power-of-2 FFT; coherently integrates chirps; applies `calRange = 1.6 m` offset; sums power across all channels to detect the target range bin. |
| [`calculateMimoSpatialSpectrum.m`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/helpers/calculateMimoSpatialSpectrum.m) | **Spatial processing and AOA detection:** Extracts complex envelopes at the target range bin; applies spatial tapering (Uniform/Hann/Chebyshev); computes measured Bartlett spectra; detects target AOAs via `findSpatialPeaks`; synthesizes data-driven theoretical response. |
| [`findSpatialPeaks.m`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/helpers/findSpatialPeaks.m) | **Peak detection utility:** Identifies local maxima in spatial spectra with specified minimum prominence (3 dB) and angular separation (4°), locking up to `targetCount` targets. |
| [`plotMimoArrayGeometry.m`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/helpers/plotMimoArrayGeometry.m) | **Figure 1:** Three-panel stem plot of physical Rx ($8\times \lambda/2$), physical Tx ($2\times d_{\text{Tx}}$), and virtual array (including co-array redundancy weights and dynamic aperture scaling). |
| [`plotMimoRangeProfile.m`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/helpers/plotMimoRangeProfile.m) | **Figure 2:** Calibrated range profile with an automated red marker indicating the detected reflector distance. |
| [`plotMimoSpatialSpectrum.m`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/helpers/plotMimoSpatialSpectrum.m) | **Figure 3:** Side-by-side comparison of simulated vs measured spatial spectra, showing physical (8 elements) and virtual (16 elements) responses with detected AOA markers. |
| [`plotMimoRangeAzimuth.m`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/helpers/plotMimoRangeAzimuth.m) | **Figure 4:** 2D range-azimuth heatmap across the 16 virtual channels, showing target returns in range and angle. |
| [`plotMimoTxRangeProfiles.m`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/helpers/plotMimoTxRangeProfiles.m) | **Figure 5:** Two-panel plot displaying all eight Rx returns for Tx 1 and Tx 2 on a common dB scale (facilitating polarimetric co-pol vs cross-pol observation when Tx 2 is rotated 90°). |
| [`localHann.m`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/helpers/localHann.m) | **Window utility:** Toolbox-independent periodic Hann window generator. |

---

## 4. Interactive live script controls

When running [`mimo_lab.mlx`](file:///home/martin/Repositories/Phaser-control-with-MATLAB/projects/2026-summer-school/mimo_lab.m), participants adjust parameters using interactive Live Editor controls in **Section 1**:

```matlab
% Transmit antenna separation in wavelengths (lambda)
dTx_lambda = 4.0;            % 1.0: Overlapped, 4.0: Contiguous ULA, > 4.0: Sparse

% Number of target peaks to detect and simulate
targetCount = 2;

% Spatial tapering / windowing across array elements
spatialWindow = 'Uniform';   % 'Uniform', 'Hann', 'Chebyshev'
sllChebDb = -25;             % Chebyshev sidelobe level (dB)

% Tx phase calibration offset between out 1 and out 2 (degrees)
tx_phase_cal = 0.0;

% FMCW range calibration offset and search interval
calRange = 1.6;              % Hardware offset calibration (m)
targetRangeGate = [0.5, 5.0];% Target search interval (m)
```

### Participant experiments:
1. **Resolution doubling:** Run with `targetCount = 2`. Observe how two reflectors at $\approx \pm 14^\circ$ form an unresolved lump on the physical Rx curve (blue) but split into two narrow peaks on the virtual array curve (red).
2. **Topology variations:** Change `dTx_lambda` to `1.0` to model the overlapped Tx bracket. Inspect Figure 1 to observe co-array redundancy weights, and examine how aperture size impacts beamwidth.
3. **Aperture tapering:** Switch `spatialWindow` from `'Uniform'` to `'Chebyshev'` and adjust `sllChebDb` from `-15 dB` to `-35 dB`. Note the reduction in sidelobes at the cost of mainlobe widening.
4. **Tx phase errors:** Adjust `tx_phase_cal` using the slider. Observe how non-zero phase mismatches between the two transmit paths degrade virtual array coherence, introducing beam splitting and grating lobes.
5. **Transmit channel separation and polarimetry:** Rotate the Tx 2 Vivaldi antenna by 90° relative to Tx 1. Observe Figure 5:
   - For a trihedral corner reflector (co-pol preserving), Tx 1 produces strong returns while Tx 2 is attenuated.
   - For a dihedral reflector rotated at 45° (polarization rotating), Tx 2 exhibits strong cross-pol conversion.

---

## 5. Study questions

1. **Aperture and angular resolution:**
   - From the measured spatial spectrum (Figure 3), determine the 3 dB beamwidth for both the physical eight-element array and the 16-element virtual array. How closely do they match theoretical values ($\theta_{3\text{dB}} \approx 50.8^\circ \lambda / L$)?
2. **Co-array redundancy:**
   - For an overlapped bracket with $d_{\text{Tx}} = 1.0\lambda$, how many unique spatial positions are formed? Why do redundant virtual elements increase SNR at those spatial frequencies?
3. **Grating lobes in sparse arrays:**
   - If the two transmitters are placed at $d_{\text{Tx}} = 5.0\lambda$ (leaving a $1.0\lambda$ hole between the subarrays), at what azimuth angles do grating lobes appear?
4. **TDM coherence constraints:**
   - What happens to the synthesized virtual array response if a target moves radially during the eight-burst switching sequence? Calculate the Doppler phase shift $\Delta\phi = 4\pi v \Delta t / \lambda$ for a velocity of $0.5\text{ m/s}$ with a switching delay of $\Delta t = 100\text{ ms}$.
5. **Polarimetric scattering:**
   - In Figure 5, compare the signal level at the target range bin between Tx 1 and Tx 2. What is the measured cross-polarization ratio (XPR in dB)? How does this align with the theoretical scattering matrix of the target reflector?
