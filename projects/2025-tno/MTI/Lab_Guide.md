# Practicum Lab: MTI Filtering

## Objective
Moving Target Indicator (MTI) filters are used to distinguish moving targets from static clutter. In this lab, you will analyze a slow-time MTI filter (delay-line canceler) and evaluate its theoretical and practical performance.

## Theoretical Context
- **Doppler Shift:** A target moving with radial velocity $v$ produces a Doppler frequency shift $f_d = \frac{2v}{\lambda}$.
- **Delay-Line Cancelers:** The simplest MTI filters subtract consecutive pulses to remove static targets ($f_d = 0$). 
  - A 2-pulse canceler has the transfer function $H(z) = 1 - z^{-1}$.
  - A 3-pulse canceler has the transfer function $H(z) = 1 - 2z^{-1} + z^{-2}$.
- **FFT as a Filter Bank:** Computing an $N$-point FFT across slow-time is mathematically equivalent to applying a bank of $N$ narrow bandpass filters, each tuned to a different Doppler frequency.

## Workflow
1. **Analyze Theoretical Filters:** Open `MTI.m` to see the theoretical frequency responses of the 2-pulse and 3-pulse cancelers.
2. **Design a Custom Filter:** Use MATLAB's filter design functions to create a custom filter tuned to a specific target speed (1 m/s).
3. **DSP Loop:** Review the radar processing loop to see how the chosen MTI filter is applied across the slow-time dimension (pulses) prior to computing the Range and Doppler FFTs.
4. **Visualize Responses:** Toggle the `applyFilter` variable and change the `mti_coeff` variable to see the difference between raw data, simple cancelers, and your custom bandpass filter.

## Assignment Questions
1. In the script, an MTI filter is defined with 2 or 3 coefficients. Plot the theoretical frequency response of these two MTI filters using MATLAB (e.g., `freqz`). 
2. What is the effect of defining a longer MTI filter? Also show this in the theoretical frequency response.
3. How can the MTI filter be moved to a different frequency? For example from high-pass to low-pass, such that only clutter is passed. *(Hint: Consider multiplying the filter impulse response by a complex exponential $e^{j 2 \pi f_{shift} n}$)*
4. Design an MTI filter that passes the velocity 1 m/s, with 50 dB suppression for other velocities. Show the frequency response of this filter, and the effect on the FMCW script. Also try different suppression levels. What is the effect? *(Hint: Use a higher-order FIR filter, e.g., order 39 with a Taylor window)*
5. Can you explain the operation of using the FFT to do Doppler filtering, in terms of MTI filtering? How does the FFT compare in terms of computational complexity? 
