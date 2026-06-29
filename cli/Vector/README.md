# Vector-field acoustic trapping

This folder is an isolated research prototype. It does not replace `cli/New`.

The method designs the Gor'kov radiation-force field directly over a 3D volume. A smooth squared-distance potential defines the requested point, nearest-of-many, or path basin. A phase-only optimizer then matches all three components of `-grad(U)` at volumetric collocation points using analytic phase derivatives. Four rapidly multiplexed phase frames are optimized jointly so wavelength-scale force ripple can cancel in the bead's time-averaged dynamics.

The primary viable method is `dynamic_vector_policy.py`: camera position is converted to the nearest-target/path vector and a tri-axial local trap ensemble is placed 1.0 mm ahead of the bead. X-, Y-, and Z-oriented twin-trap frames use direction-adaptive dwell weights `[6|dx|+1, |dy|+1, 6|dz|+1]` to compensate for opposed-array axial bias without wasting lateral dwell where it is unnecessary. For multiple beads, ensembles are time-interleaved. Its vector plot is a *policy map*: each arrow is the averaged force generated for a bead observed there, not one simultaneous static field.

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

Compile current camera observations into three weighted 512-level, 200-channel axis frames per bead:

```powershell
& $py .\compile_phase_frames.py --case two-points --positions='-20,0,4;22,3,-5' --output results\live_frames.csv
```

The frames must be refreshed from new tracked positions and interleaved faster than bead motion. This prototype intentionally does not guess the camera API or FPGA-safe update rate.

## C phase sender

`laptop_phase_sender_vector.c` implements the corrected policy directly against `phase_protocol.h`. It has exactly two operating modes: `one` and `two`. Two-particle mode assigns the two beads one-to-one to the minimum-total-distance pair of goals, then interleaves each particle's X/Y/Z ensemble.

```powershell
gcc -O2 -std=c11 -Wall -Wextra laptop_phase_sender_vector.c -I.. -lws2_32 -lm -o laptop_phase_sender_vector.exe
.\laptop_phase_sender_vector.exe --self-test one vector_state_one.txt
.\laptop_phase_sender_vector.exe 192.168.1.50 one --port 5656
.\laptop_phase_sender_vector.exe 192.168.1.50 two --port 5656
```

The commands above need no text file. They remain connected and accept live keyboard movement: `1/2` selects a particle, `x/s` moves X, `c/d` moves Y, `z/a` moves Z, `h` homes the selected particle, `0` homes all, and `q` quits.

For camera-tracked closed-loop routing, add the state file explicitly:

```powershell
.\laptop_phase_sender_vector.exe 192.168.1.50 one --port 5656 --state vector_state_one.txt
.\laptop_phase_sender_vector.exe 192.168.1.50 two --port 5656 --state vector_state_two.txt
```

The one-particle state format is `one bx by bz gx gy gz`. The two-particle format is `two b1x b1y b1z b2x b2y b2z g1x g1y g1z g2x g2y g2z`. Coordinates are millimetres. A tracking process should write a temporary file and atomically rename it over the state file; malformed or partially written updates are rejected while the last valid state remains active.

The port is always selected with `--port`; it defaults to 5656 if omitted. Other options are `--board-mm`, `--ensemble-ms`, `--lookahead-mm`, and `--step-mm`. The Pi still runs `cli --phase-bridge 5656`. Absolute acoustic output and safe update timing must be established on the hardware.

## Scientific status

The outputs are simulation evidence, not an experimental proof. Absolute force depends on measured per-emitter pressure, phase offsets, reflections, temperature, and the true EPS density/compressibility. The default 3000 Pa reference is metadata only; normalized direction and basin metrics do not depend on it. A 3 mm sphere at 40 kHz has `ka` near 1.1, so Gor'kov theory is a useful first design model but is not deeply inside the Rayleigh limit. Hardware certification requires acoustic calibration, bead tracking, and preferably a scattering or boundary-element model.
