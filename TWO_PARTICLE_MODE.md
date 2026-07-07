# Static two-particle levitation

This path replaces the unsuccessful global vector-field objective with a
single, continuously active two-trap hologram. It does not time-slice particles.
The current compiler also optimizes the weakest trap across one nominal and four
fixed error scenarios containing 5-degree RMS phase error and 5% RMS amplitude
error per emitter.

The compiler minimises the normalized Gor'kov potential locally at two target
points. Its maximin objective maximises the smaller of the two traps' weakest
Hessian eigenvalues, so a solution is rejected if either trap is a saddle in
any rotated direction. A force-bias penalty keeps the equilibrium close to the
requested coordinates. The final result is quantized to the FPGA's 512 phase
levels before verification.

The recommended automatic staged mode starts with one particle at the origin.
Move it to any commanded position at least 12 mm from the origin and press `2`.
The existing single-focus frame remains active while the laptop compiles a new
robust hologram for the particle's current position plus `(0,0,0)`. The FPGA is
updated only if optimization succeeds and the resulting 200 phases validate.

```powershell
python cli\two_particle_optimizer.py `
  --targets="-16,0,0;0,0,0" `
  --board-mm=135 `
  --iterations=1600 `
  --output=cli\staged_two_particle_phases.txt

python cli\verify_two_particle.py `
  --phases=cli\staged_two_particle_phases.txt `
  --trials=500 `
  --output=cli\staged_two_particle_verification.json
```

Build the normal sender, then activate the compiled two-trap frame:

```powershell
gcc -std=gnu99 -O3 -Wall -Wextra cli\laptop_phase_sender.c `
  -o cli\laptop_phase_sender.exe -lm -lws2_32

cd cli
$env:PYTHON = "C:\path\to\python.exe"
.\laptop_phase_sender.exe 169.254.3.160 --staged-auto 5656 135
```

The sender initially activates the ordinary single-particle focus at `(0,0,0)`.
Use the normal movement keys (press Enter after each command) to move particle 1.
Once it is at least 12 mm from the origin, press `2`. Optimization takes a few
seconds; particle 1 remains held by the unchanged single focus during this time.
After the sender reports that the two-particle field is active, introduce
particle 2 at the origin. Press `q` to stop.

Run the executable from the `cli` directory so it can find
`two_particle_optimizer.py` and its `Vector` model. `PYTHON` is optional when
`python` is already on `PATH`; when set, it must be the path to the Python
executable. Python must have NumPy available.

The fixed `--staged` mode remains available for experiments that require a
pre-certified phase frame and no optimization delay.

The FPGA retains each frame, so continuous retransmission is unnecessary. The
phase file contains exactly 200 integer ticks; malformed or out-of-range files
are rejected.

## Verification status

- Both quantized traps have positive Hessian eigenvalues at finite-difference
  steps from 0.2 to 0.7 mm.
- The weaker normalized stiffness eigenvalue is approximately 0.249.
- The midpoint potential is much higher than either trap potential, rather
  than forming a competitive central well.
- In 500 Monte Carlo trials at each error level, both staged traps stayed locally
  positive-definite through the harshest tested case: independent 20-degree
  RMS phase error and 15% RMS amplitude error per emitter.
- Five independent optimizer seeds all produced two positive-definite traps.

These are model tests, not proof of physical levitation. Absolute gravity
margin depends on measured acoustic pressure, EPS density, transducer phase and
amplitude calibration, reflections, and temperature. A 3 mm bead at 40 kHz has
`ka` near 1.1, so the Gor'kov model is useful but only marginally Rayleigh.

The staged equilibria are predicted near `(-16.155,-0.003,-0.001)` mm and
`(0.195,-0.005,-0.010)` mm. Under the conditional 3000 Pa reference, the weaker
stiffness is about 0.022 N/m and linear gravity sag is about 0.157 mm.

Bring-up should start at reduced voltage. Capture particle 1 at the origin, move
it to -16 mm, activate the dual field with `2`, and only then introduce particle
2 near the origin. Verify both remain captured before attempting closer spacing.
