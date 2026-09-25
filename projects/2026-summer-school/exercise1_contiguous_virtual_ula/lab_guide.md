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

### 3.2 Hybrid Beamsteering & Coherent Synthesis
The physical 8-element Rx array employs hybrid beamforming: each 4-element subarray (ADAR1000 Chip 1 and Chip 2) is steered in analog, and the two subarrays (separated by $4 d_{\text{Rx}} = 2\lambda$) are combined digitally via Pluto's two receiver channels:
```matlab
% Steer 4-element analog subarrays:
analog_1sub = exp(-1j * (2*pi/lambda) * (0:3)' * dRx * sind(ang));
analogsteer = analogWeightsCalAdjustment([analog_1sub, analog_1sub], calibrationweights.AnalogWeights);
setAnalogBfWeights(bf, analogsteer);

% Steer 2-channel digital combiner:
digitalWeights = [1; exp(-1j * (2*pi/lambda) * (4*dRx) * sind(ang))];
digitalsteer = digitalWeightsCalAdjustment(digitalWeights, calibrationweights.DigitalWeights);
data_comb = data_raw * conj(digitalsteer);
```

For each scan angle $\theta$:
1. Tx 1 is selected; the complex return at the target range bin $S_1(\theta)$ is recorded.
2. Tx 2 is selected; the complex return at the target range bin $S_2(\theta)$ is recorded.
3. The virtual 16-element array response is synthesized by applying the geometric phase compensation for Tx 2:
   $$S_{\text{virt}}(\theta) = S_1(\theta) + S_2(\theta) \cdot e^{j \left(\frac{2\pi d_{\text{Tx}}}{\lambda} \sin\theta - \phi_{\text{cal}}\right)}$$
4. Power spectra are computed as $P_{\text{rx}}(\theta) = |S_1(\theta)|^2$ and $P_{\text{virt}}(\theta) = |S_{\text{virt}}(\theta)|^2$.

---

## 4. Laboratory Workflow

### Step 1: Target Setup
1. Position two radar targets (e.g., corner reflectors) at distance $R \approx 1.4\text{ m}$.
2. Set target parameters in `mimo_lab.m`:
   ```matlab
   target_range    = 1.4;         % Target range in meters
   target1_azimuth = -14.0;       % Target 1 azimuth in degrees
   target2_azimuth = 14.0;        % Target 2 azimuth in degrees
   tx_phase_cal    = 0.0;         % Tx phase calibration in degrees
   ```

### Step 2: Hardware Execution
1. Connect Tx 1 to SMA Out 1 and Tx 2 to SMA Out 2.
2. Run `mimo_lab.m`.
3. The script will:
   - Calculate theoretical simulated spatial spectra for the physical and virtual arrays.
   - Capture a boresight range profile to locate the target range gate.
   - Electronically sweep azimuth angles for Tx 1 and Tx 2.
   - Synthesize the 16-element virtual array response.
   - Display the measured range profile and side-by-side simulated vs. measured spatial spectra.

---

## 5. Assignment Questions

1. **Aperture & Resolution:** From the measured spatial spectrum, determine the 3 dB beamwidth for the physical 8-element array and the 16-element virtual array. Compare with theoretical values.
2. **Phase Compensation:** Explain why the factor $e^{j \frac{2\pi d_{\text{Tx}}}{\lambda} \sin\theta}$ synthesizes the remaining 8 elements of the 16-element virtual array.
3. **Array Spacing:** If the two Tx antennas were placed $5\lambda$ apart instead of $4\lambda$, what changes in the spatial sampling grid and the resulting array factor?
