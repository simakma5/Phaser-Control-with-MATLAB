# Practicum Lab: Micro-Doppler

## Objective
In this lab, you will use a rotating fan as a target to observe and analyze micro-Doppler signatures with the FMCW radar system. By running the measurement multiple times with different physical configurations, you will extract the radar output and explain the underlying physics.

## Theoretical Context
- **Micro-Doppler Effect:** While bulk target motion creates a primary Doppler shift, vibrating or rotating structures (like fan blades) create secondary, periodic Doppler modulations called micro-Doppler signatures.
- **Rotating Blades:** For a fan rotating with frequency $f_{rot}$ (in Hz) and blade length $L$, the maximum tip velocity is $v_{max} = 2\pi f_{rot} L$. This continuous range of velocities from the hub to the tip creates a distinct "flash" or streak in the Doppler spectrum.
- **Flash Spacing:** The time between blade flashes depends on both the rotation speed and the number of blades. The observed flash rate is $f_{flash} = N_{blades} \times f_{rot}$.

## Workflow
1. **Radar Parameters & DSP Pipeline:** Open `Micro_Doppler.m`.
   - The script sets the PRF to 2000 Hz and the number of pulses to 128 to capture the fast-moving fan blades without aliasing.
   - It computes the Range FFT across fast time and Doppler FFT across slow time to generate the real-time Range-Doppler response.
2. **Run Experiments:** You will run the script multiple times, physically changing the fan configuration each time. Save your Doppler-Time plots for analysis.

## Assignment Questions
1. Place the fan at a fixed distance from the radar, making sure all four blades are visible. Run the measurement with the fan turned on (one measurement with low rotation speed, one measurement with high rotation speed).
2. Repeat the measurement with 3 blades, 2 blades, and 1 blade.
3. **Analysis:** Try to explain the relation between the spacing of these bright spots (in the Doppler-Time/Range-Doppler plot) and the rotation speed of the fan. 
4. **Analysis:** Try to explain what happened after changing the number of blades.
5. **Analysis:** Explain why you observe so many bright spots at different Doppler velocities during a single blade flash.
