# Vector-field acoustic trapping

This folder is an isolated research prototype. It does not replace `cli/New`.

The method designs the Gor'kov radiation-force field directly over a 3D volume. A smooth squared-distance potential defines the requested point, nearest-of-many, or path basin. A phase-only optimizer then matches all three components of `-grad(U)` at volumetric collocation points using analytic phase derivatives. Four rapidly multiplexed phase frames are optimized jointly so wavelength-scale force ripple can cancel in the bead's time-averaged dynamics.

The primary viable method is `dynamic_vector_policy.py`: camera position is converted to the nearest-target/path vector, a strong local trap is placed 2 mm ahead of the bead, and the phase command is refreshed as the bead moves. For multiple beads, commands are time-interleaved. Its vector plot is a *policy map*: each arrow is the force generated for a bead observed there, not one simultaneous static field.

## Run

```powershell
cd D:\vtatp_proper\vdatp\cli\Vector
powershell -NoProfile -ExecutionPolicy Bypass -File .\run_vector_experiments.ps1
```

The script logs progress, limits verification time, computes quantitative verification, and produces PyVista PNGs under `results`. The rejected static/global research route is available with `-RunStaticResearch`; it is off by default because current independent-grid evidence does not support it.

Run one case directly:

```powershell
$py = "C:\Users\Bored\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
& $py .\vector_optimizer.py --case point --iterations 450 --max-seconds 180
& $py .\verify_and_visualize.py --case point --screenshot
```

Compile current camera observations into one 512-level, 200-channel frame per bead:

```powershell
& $py .\compile_phase_frames.py --case two-points --positions='-20,0,4;22,3,-5' --output results\live_frames.csv
```

The frames must be refreshed from new tracked positions and interleaved faster than bead motion. This prototype intentionally does not guess the camera API or FPGA-safe update rate.

## Scientific status

The outputs are simulation evidence, not an experimental proof. Absolute force depends on measured per-emitter pressure, phase offsets, reflections, temperature, and the true EPS density/compressibility. The default 3000 Pa reference is metadata only; normalized direction and basin metrics do not depend on it. A 3 mm sphere at 40 kHz has `ka` near 1.1, so Gor'kov theory is a useful first design model but is not deeply inside the Rayleigh limit. Hardware certification requires acoustic calibration, bead tracking, and preferably a scattering or boundary-element model.
