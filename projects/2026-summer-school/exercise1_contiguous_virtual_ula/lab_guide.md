# MIMO Radar Summer School 2026

## Exercise 1: Contiguous Virtual ULA Synthesis (Aperture Doubling)

---

## 1. Objective
In this laboratory exercise, you will synthesize a **16-element contiguous virtual uniform linear array (ULA)** from an **8-element physical phased array receiver** and **two physical transmit antennas**.

By the end of this exercise, you will:
1. Master the theoretical foundation of **spatial convolution** ($\mathbf{r}_{\text{virt}} = \mathbf{r}_{\text{Tx}} \ast \mathbf{r}_{\text{Rx}}$) in MIMO radar.
2. Configure Time-Division Multiplexed (TDM) MIMO transmission using either **digital electronic RF switching** (`bf.EnableOut1`) or **manual single-Tx repositioning on an indexed rail**.
3. Implement sequential element-level readout on the ADAR1000 beamformer chips to recover the full $8 \times 1$ digital receive snapshot.
4. Double the physical radar aperture from $3.5\lambda$ to $7.5\lambda$, cutting the 3 dB beamwidth in half from $\approx 14.5^\circ$ to $\approx 6.8^\circ$.
5. Resolve two closely-spaced targets separated by $9^\circ$ that are completely unresolved by the physical receiver alone, comparing **Bartlett**, **Capon (MVDR)**, and **MUSIC** direction-of-arrival (DOA) estimation algorithms.

---

## 2. Theoretical Context

### 2.1 Spatial Convolution and Virtual Arrays
For a radar system with transmit antenna positions $\mathbf{r}_{\text{Tx}}$ and receive antenna positions $\mathbf{r}_{\text{Rx}}$, the two-way phase delay to a far-field target at angle $\theta$ is determined by the total round-trip distance:
$$d_{\text{total}}(\theta) = (\mathbf{r}_{\text{Tx}} + \mathbf{r}_{\text{Rx}}) \sin\theta$$

Consequently, the effective spatial sampling points of the system correspond to the **spatial convolution** (or Minkowski sum) of the transmit and receive coordinates:
$$\mathbf{r}_{\text{virt}} = \mathbf{r}_{\text{Tx}} \oplus \mathbf{r}_{\text{Rx}} = \{ \mathbf{r}_t + \mathbf{r}_r \mid \mathbf{r}_t \in \mathcal{T},\, \mathbf{r}_r \in \mathcal{R} \}$$

The composite two-way steering vector is the Kronecker product of the transmit and receive steering vectors:
$$\mathbf{a}_{\text{virt}}(\theta) = \mathbf{a}_{\text{Tx}}(\theta) \otimes \mathbf{a}_{\text{Rx}}(\theta)$$

### 2.2 Contiguous Array Geometry Design
In our laboratory setup:
- **Physical Rx Array:** $N_{\text{Rx}} = 8$ patch antennas with inter-element spacing $d_{\text{Rx}} = \lambda/2$:
  $$\mathbf{x}_{\text{Rx}} = [0, 1, 2, 3, 4, 5, 6, 7] \cdot \frac{\lambda}{2} \quad (\text{Aperture } L_{\text{Rx}} = 3.5\lambda)$$
- **Physical Tx Antennas:** $N_{\text{Tx}} = 2$ Vivaldi antennas spaced by $d_{\text{Tx}} = 8 \cdot d_{\text{Rx}} = 4\lambda$:
  $$\mathbf{x}_{\text{Tx}} = [0, 4\lambda]$$

When Tx 1 ($x_1 = 0$) fires, it illuminates the 8 physical Rx elements, synthesizing virtual elements $1\dots 8$ at locations $[0\dots 3.5\lambda]$.
When Tx 2 ($x_2 = 4\lambda$) fires, the same 8 physical Rx elements are illuminated from an offset position, synthesizing virtual elements $9\dots 16$ at locations $[4.0\lambda\dots 7.5\lambda]$.

```
Physical Rx Array (8 elements, d = λ/2):
[ R1 ][ R2 ][ R3 ][ R4 ][ R5 ][ R6 ][ R7 ][ R8 ]   (Aperture ~ 3.5 λ)

Tx Positions:
[ T1 ] -----------------------------------------> [ T2 ] (Separation = 8d = 4 λ)

Synthesized Virtual Array (16 elements contiguous, d = λ/2):
[ V01 V02 V03 V04 V05 V06 V07 V08 ][ V09 V10 V11 V12 V13 V14 V15 V16 ]  (Aperture ~ 7.5 λ)
```

The resulting virtual array is a contiguous **16-element uniform linear array** with uniform spacing $d = \lambda/2$ and total aperture $L_{\text{virt}} = 7.5\lambda$.

### 2.3 Angular Resolution and Rayleigh Criterion
The 3 dB angular beamwidth of an unweighted linear array of aperture length $L$ is approximately:
$$\theta_{3\text{dB}} \approx 0.886 \frac{\lambda}{L} \text{ radians} \approx 50.8^\circ \frac{\lambda}{L}$$

- **Physical Rx Array ($L = 3.5\lambda$):** $\theta_{3\text{dB}} \approx \frac{50.8^\circ}{3.5} \approx 14.5^\circ$.
- **Synthesized Virtual Array ($L = 7.5\lambda$):** $\theta_{3\text{dB}} \approx \frac{50.8^\circ}{7.5} \approx 6.8^\circ$.

If two targets are placed with an angular separation of $\Delta\theta = 9^\circ$:
- They fall inside the physical mainlobe ($\Delta\theta < 14.5^\circ$), appearing as an **unresolved single peak**.
- They exceed the virtual mainlobe ($\Delta\theta > 6.8^\circ$), appearing as **two sharply resolved peaks**.

---

## 3. Hardware Architecture & Switching Mechanics

### 3.1 Transmit RF Switching (`bf.EnableOut1`)
On the ADALM-PHASER (CN0566), the 10 GHz FMCW chirp is produced by the ADF4159 synthesizer followed by an active multiplier chain. The signal passes through an on-board SPDT microwave switch connected to **SMA Out 1** and **SMA Out 2**:
```matlab
% Select Out 1 (Tx 1):
bf.EnableOut1 = true;

% Select Out 2 (Tx 2):
bf.EnableOut1 = false;
```
> [!NOTE]
> Setting `bf.EnableOut1` sends a digital SPI command over Ethernet/USB to the Phaser controller. Between transmit frames, this electronically toggles which Vivaldi antenna radiates.

### 3.2 Sequential 8-Element Rx Readout on ADAR1000
The PlutoSDR contains 2 analog receive channels (`Rx1` and `Rx2`), while the Phaser board has 8 antenna elements. Channels 1–4 are routed through ADAR1000 Beamformer Chip 1 to Pluto `Rx1`, and Channels 5–8 are routed through ADAR1000 Chip 2 to Pluto `Rx2`.

To extract the uncombined, full 8-element digital array vector $\mathbf{y}_{\text{Rx}} \in \mathbb{C}^{8 \times 1}$, the script sequences through 4 sub-frames, powering down 3 out of 4 channels per chip:
```matlab
for ch = 1:4
    bf.RxPowerDown(:) = 1;         % Power down all elements
    bf.RxPowerDown(ch) = 0;         % Enable channel ch on Chip 1
    bf.RxPowerDown(ch + 4) = 0;     % Enable channel ch+4 on Chip 2
    bf.LatchRxSettings();

    data = captureTransmitWaveform(rx, tx, bf);
    % data(:, :, 2) is Pluto Rx1 -> Element ch
    % data(:, :, 1) is Pluto Rx2 -> Element ch+4
end
```

---

## 4. Laboratory Workflow

### Step 1: Run Simulation Warm-Up
Open [`mimo_virtual_ula_contiguous.m`](mimo_virtual_ula_contiguous.m) and verify that `mode = 'simulated';` is active.
Run the script to observe:
1. **Figure 1:** The physical Rx, physical Tx, and synthesized 16-element virtual geometry.
2. **Figure 2:** The comparative spatial spectrum plots. Notice how Bartlett, Capon, and MUSIC on the 8-element physical array fail to resolve the two targets at $\pm 4.5^\circ$, while all three algorithms on the 16-element virtual array clearly display two distinct peaks.

### Step 2: Physical Hardware Setup
1. Mount two Vivaldi antennas on a rigid baseline separated by $d_{\text{Tx}} = 12\text{ cm}$ ($4\lambda$).
2. Connect Tx 1 to **SMA Out 1** and Tx 2 to **SMA Out 2** using phase-stable SMA cables.
3. Position two trihedral corner reflectors at range $R \approx 2.0\text{--}2.5\text{ m}$ from the radar, separated in azimuth by $\approx 9^\circ$ (e.g., at $-4.5^\circ$ and $+4.5^\circ$, or approximately $35\text{ cm}$ lateral spacing at $2.2\text{ m}$ range).
4. Connect the Phaser and PlutoSDR to the host PC via USB and Ethernet.

### Step 3: Hardware Acquisition
Set `mode = 'electronic';` in [`mimo_virtual_ula_contiguous.m`](mimo_virtual_ula_contiguous.m).
- If only one Vivaldi antenna is available, mount it on the 3D-printed indexed rail and set `mode = 'manual';`. The script will pause and prompt you to move the antenna from $x = 0$ to $x = 12\text{ cm}$ between bursts.
- Run the script and inspect the reconstructed virtual array spectrum.

---

## 5. Assignment Questions

1. **Aperture & Resolution Scaling:** Calculate the theoretical Rayleigh 3 dB beamwidth for:
   - An 8-element ULA with $d = \lambda/2$.
   - A 16-element ULA with $d = \lambda/2$.
   Compare your calculations with the half-power beamwidths measured from your Bartlett spectrum plot.
2. **Tx Separation Limits:** What would happen to the virtual array if the two physical Tx antennas were spaced by $d_{\text{Tx}} = 5\lambda$ instead of $4\lambda$?
   *(Hint: Consider the spatial convolution $\mathbf{r}_{\text{virt}} = \mathbf{r}_{\text{Tx}} \oplus \mathbf{r}_{\text{Rx}}$. Will there be missing virtual elements / holes? What effect do holes have on the spatial spectrum and grating lobes?)*
3. **Algorithm Comparison:** Compare the Bartlett, Capon (MVDR), and MUSIC spatial spectra for the 16-element virtual array:
   - Which algorithm provides the narrowest peak width?
   - How does Capon (MVDR) handle correlated or coherent radar returns compared to forward-backward smoothed MUSIC?
4. **Target Motion Impact:** Suppose one of the targets was moving with a radial velocity of $v = 0.5\text{ m/s}$ during the measurement. How would target Doppler phase shift affect the coherent concatenation between Tx 1 and Tx 2 snapshots?
