Simulation examples

This folder contains two minimal examples to help tune and validate a basic altitude controller before hardware testing.

Python example
- `quad_sim.py`: simple vertical dynamics, sensor noise models (IMU accel + barometer), a complementary-style estimator, and a PID altitude controller. Generates plots and saves them as PNG files.
- Run (from project root):

```bash
python3 sim/quad_sim.py
```

Requirements: `numpy`, `matplotlib`

MATLAB example
- `quad_sim.m`: minimal script showing the same vertical dynamics and PID loop for quick tests in MATLAB/Octave.
