# MIMO Radar Summer School 2026

## Exercise 1: Contiguous Virtual ULA Synthesis

---

## 1. Objective
Synthesize a 16-element contiguous virtual uniform linear array (ULA) from an 8-element physical phased array receiver and two electronically switched transmit antennas.

Key goals:
1. Apply the principle of spatial convolution ($\mathbf{r}_{\text{virt}} = \mathbf{r}_{\text{Tx}} \ast \mathbf{r}_{\text{Rx}}$) in MIMO radar.
2. Control electronic TDM-MIMO switching between SMA Out 1 and Out 2 via software (`bf.EnableOut1`).
3. Evaluate aperture expansion from $3.5\lambda$ to $7.5\lambda$ and the corresponding 3 dB beamwidth reduction from $\approx 14.5^\circ$ to $\approx 6.8^\circ$.
4. Compare simulated and measured spatial spectra for targets positioned at specified azimuth angles and range.

---

## 2. Theoretical Context

### 2.1 Spatial Convolution and Virtual Arrays
For a radar system with transmit antenna positions $\mathbf{r}_{\text{Tx}}$ and receive antenna positions $\mathbf{r}_{\text{Rx}}$, the two-way phase delay to a far-field target at azimuth angle $\theta$ is:
$$d_{\text{total}}(\theta) = (\mathbf{r}_{\text{Tx}} + \mathbf{r}_{\text{Rx}}) \sin\theta$$

The effective spatial sampling coordinates correspond to the spatial convolution of the transmit and receive coordinates:
$$\mathbf{r}_{\text{virt}} = \mathbf{r}_{\text{Tx}} \oplus \mathbf{r}_{\text{Rx}} = \{ \mathbf{r}_t + \mathbf{r}_r \mid \mathbf{r}_t \in \mathcal{T},\, \mathbf{r}_r \in \mathcal{R} \}$$

### 2.2 Contiguous Array Geometry
- **Physical Rx Array:** $N_{\text{Rx}} = 8$ patch elements with inter-element spacing $d_{\text{Rx}} = \lambda/2$:
  $$\mathbf{x}_{\text{Rx}} = [0, 1, 2, 3, 4, 5, 6, 7] \cdot \frac{\lambda}{2} \quad (L_{\text{Rx}} = 3.5\lambda)$$
- **Physical Tx Antennas:** $N_{\text{Tx}} = 2$ Vivaldi antennas spaced by $d_{\text{Tx}} = 8 \cdot d_{\text{Rx}} = 4\lambda$:
  $$\mathbf{x}_{\text{Tx}} = [0, 4\lambda]$$

When Tx 1 ($x_1 = 0$) transmits, it samples spatial positions $[0\dots 3.5\lambda]$.
When Tx 2 ($x_2 = 4\lambda$) transmits, it samples spatial positions $[4.0\lambda\dots 7.5\lambda]$.

```
Physical Rx Array (8 elements, d = λ/2):
[ R1 ][ R2 ][ R3 ][ R4 ][ R5 ][ R6 ][ R7 ][ R8 ]   (Aperture: 3.5 λ)

Tx Positions:
[ T1 ] -----------------------------------------> [ T2 ] (Separation: 4 λ)

Synthesized Virtual Array (16 elements contiguous, d = λ/2):
[ V01 V02 V03 V04 V05 V06 V07 V08 ][ V09 V10 V11 V12 V13 V14 V15 V16 ]  (Aperture: 7.5 λ)
```

The resulting virtual array is a contiguous 16-element ULA with $d = \lambda/2$ and total aperture $L_{\text{virt}} = 7.5\lambda$.

### 2.3 Beamwidth and Resolution Limits
The nominal 3 dB angular beamwidth of an unweighted linear array of aperture $L$ is:
$$\theta_{3\text{dB}} \approx 0.886 \frac{\lambda}{L} \text{ rad} \approx 50.8^\circ \frac{\lambda}{L}$$

- **Physical Rx Array ($L = 3.5\lambda$):** $\theta_{3\text{dB}} \approx \frac{50.8^\circ}{3.5} \approx 14.5^\circ$.
- **Synthesized Virtual Array ($L = 7.5\lambda$):** $\theta_{3\text{dB}} \approx \frac{50.8^\circ}{7.5} \approx 6.8^\circ$.

---

## 3. Hardware Architecture & Control

### 3.1 Electronic Transmit Switching (`bf.EnableOut1`)
On the ADALM-PHASER (CN0566), the 10 GHz FMCW signal from the ADF4159 synthesizer feeds an on-board SPDT microwave switch routed to **SMA Out 1** and **SMA Out 2**:
```matlab
% Select Out 1 (Tx 1 at x = 0):
bf.EnableOut1 = true;

% Select Out 2 (Tx 2 at x = 4*lambda):
bf.EnableOut1 = false;
```

### 3.2 Switched Single-Element Mode (2x8 MIMO)
To recover all 8 physical Rx elements from the PlutoSDR's dual-channel receiver, the ADAR1000 beamformers operate in switched single-element mode:
- Chip 1 (Subarray 1, elements 1–4) feeds Pluto Channel 2.
- Chip 2 (Subarray 2, elements 5–8) feeds Pluto Channel 1.

By cycling through 4 element pairs across the two chips (`bf.RxPowerDown`), all 8 physical Rx elements are captured in 4 FMCW bursts per Tx (8 bursts total in ~1–2 seconds):
```matlab
% Pair 1: Elements [1, 5], Pair 2: [2, 6], Pair 3: [3, 7], Pair 4: [4, 8]
bf.RxPowerDown(:) = 1;
bf.RxPowerDown(nPair)     = 0; % Chip 1 (Subarray 1)
bf.RxPowerDown(nPair + 4) = 0; % Chip 2 (Subarray 2)
bf.LatchRxSettings();
```

For each Tx:
1. Tx 1 is selected (`bf.EnableOut1 = true`); the 8 physical Rx element returns $\mathbf{y}_1 \in \mathbb{C}^{8 \times 1}$ at the target range bin are recorded.
2. Tx 2 is selected (`bf.EnableOut1 = false`); the 8 physical Rx element returns $\mathbf{y}_2 \in \mathbb{C}^{8 \times 1}$ at the target range bin are recorded.
3. The 16-element virtual array snapshot is concatenated:
   $$\mathbf{y}_{\text{virt}} = \begin{bmatrix} \mathbf{y}_1 \\ \mathbf{y}_2 \cdot e^{-j \Delta\phi_{\text{cal}}} \end{bmatrix} \in \mathbb{C}^{16 \times 1}$$
4. Spatial spectra are computed digitally in post-processing across azimuth angles $\theta$:
   $$P_{\text{rx}}(\theta) = |\mathbf{a}_{\text{rx}}^H(\theta) \mathbf{y}_1|^2, \quad P_{\text{virt}}(\theta) = |\mathbf{a}_{\text{virt}}^H(\theta) \mathbf{y}_{\text{virt}}|^2$$

---

## 4. Laboratory Workflow

### Step 1: Target Setup
1. Position two radar targets (e.g., corner reflectors) at distance $R \approx 1.4\text{ m}$ (azimuth $\approx \pm 14^\circ$). No manual entry of target angles is required; the script automatically detects the target range and AOAs.
2. Verify settings in `mimo_lab.m`:
   ```matlab
   arrayMode       = '2x8_switched'; % Full 16-element contiguous virtual ULA
   calRange        = 1.6;            % Range calibration offset in meters
   targetRangeGate = [0.5, 5.0];     % Target search interval in meters
   ```

### Step 2: Hardware Execution
1. Connect Tx 1 to SMA Out 1 and Tx 2 to SMA Out 2.
2. Run `mimo_lab.m`.
3. The script will:
   - Display **Figure 1**: MIMO Array Geometry & Virtual Synthesis stem plots.
   - Configure radar via `setupFMCWRadar` with proper TDD gating and buffer flush.
   - Execute 4-pair switched element acquisition for Tx 1 and Tx 2 (8 bursts total).
   - Display **Figure 2**: Calibrated Range Profile with automated target distance identification.
   - Extract the 16-element virtual array snapshot $\mathbf{y}_{\text{virt}}$ and detect target AOAs from the measured spectrum.
   - Compute data-driven theoretical response using detected AOAs and display **Figure 3**: Side-by-side Simulated vs. Measured spatial spectra.
   - Display **Figure 4**: 2D Range-Azimuth heatmap (16 virtual elements).

---

## 5. Assignment Questions

1. **Aperture & Resolution:** From the measured spatial spectrum, determine the 3 dB beamwidth for the physical 8-element array and the 16-element virtual array. Compare with theoretical values.
2. **Phase Compensation:** Explain why the factor $e^{j \frac{2\pi d_{\text{Tx}}}{\lambda} \sin\theta}$ synthesizes the remaining 8 elements of the 16-element virtual array.
3. **Array Spacing:** If the two Tx antennas were placed $5\lambda$ apart instead of $4\lambda$, what changes in the spatial sampling grid and the resulting array factor?
