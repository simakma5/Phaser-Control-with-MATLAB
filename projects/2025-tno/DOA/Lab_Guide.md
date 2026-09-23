# Practicum Lab: Direction of Arrival (DOA)

## Objective
In this lab, you will perform electronic beam steering on the receiver to identify the Direction of Arrival (DOA) of incoming signals. You will run the measurement multiple times by placing the physical transmitter at different locations, extracting the spatial spectrum to explain array performance limits.

## Theoretical Context
- **Uniform Linear Array (ULA):** An array of evenly spaced antenna elements. When a wavefront arrives from an angle $\theta$, there is a path length difference between adjacent elements, leading to a phase shift.
- **Phase Shift:** For elements spaced by distance $d$, the progressive phase shift $\Delta\phi$ required to electronically steer the main beam towards angle $\theta$ is:
  $\Delta\phi = -\frac{2\pi d \sin(\theta)}{\lambda}$
- **Steering Vector:** By applying these progressive phase shifts across the array elements, the signals constructively interfere at the desired look direction $\theta$, allowing you to sweep across all angles and build a spatial spectrum.

## Workflow
1. **DSP Implementation:** Open `DOA.m`.
   - The script computes the spatial steering vector using the phase shift formula above.
2. **Execute Scan:** The script will electronically sweep the beam from -90 to +90 degrees and plot the received power (spatial spectrum).
3. **Run Experiments:** Move the physical transmitter to the locations specified in the assignment questions. Rerun the script, save the resulting spatial spectrum plot, and analyze the DOA performance (e.g. beam broadening, ambiguity).

## Assignment Questions
1. Place the Tx at a relatively short distance (e.g., 1 meter) at an angle (e.g., around 45 deg.) in azimuth (zero elevation). Apply beam steering on Rx to identify the Direction of Arrival (DoA).
2. At a similar distance, move the Tx to wider azimuth angles. Apply beam steering to find the DoA, see and explain the change in performance.
3. At 1m distance and 45 deg. azimuth angle, move the Tx to wider elevation angles. Apply beam steering to find the DoA, see and explain the change in performance.
4. At the 45 deg. azimuth angle and zero elevation, move the Tx to further distances. Apply beam steering to find the DoA and explain the change in performance.
5. Hold two Tx's at a relatively short distance. One will stay fixed on broadside. Move the other Tx at a similar distance to further azimuth angles. Apply beam steering and see if/when you can resolve the Tx angles.
6. Hold one Tx at 45 deg in azimuth, and place a plate reflector at –25 deg & -50 deg in azimuth. Apply beam steering and see if you can resolve the Tx and the plate.
