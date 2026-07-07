# Static two-particle levitation

This path replaces the unsuccessful global vector-field objective with a
single, continuously active two-trap hologram. It does not time-slice particles.

The compiler minimises the normalized Gor'kov potential locally at two target
points. Its maximin objective maximises the smaller of the two traps' weakest
Hessian eigenvalues, so a solution is rejected if either trap is a saddle in
any rotated direction. A force-bias penalty keeps the equilibrium close to the
requested coordinates. The final result is quantized to the FPGA's 512 phase
levels before verification.

The checked-in frame targets `(-8, 0, 0)` and `(8, 0, 0)` mm with 135 mm board
separation. Regenerate it whenever the target positions or physical board
separation change:

```powershell
python cli\two_particle_optimizer.py `
  --targets="-8,0,0;8,0,0" `
  --board-mm=135 `
  --iterations=1200 `
  --output=cli\two_particle_phases.txt

python cli\verify_two_particle.py `
  --phases=cli\two_particle_phases.txt `
  --output=cli\two_particle_verification.json
```

Build the normal sender, then activate the compiled two-trap frame:

```powershell
gcc -std=gnu99 -O3 -Wall -Wextra cli\laptop_phase_sender.c `
  -o cli\laptop_phase_sender.exe -lm -lws2_32

cli\laptop_phase_sender.exe 169.254.3.160 `
  --two cli\two_particle_phases.txt 5656
```

The FPGA retains the frame, so continuous retransmission is unnecessary. Press
Enter to resend it or `q` to disconnect. The phase file contains exactly 200
integer ticks; malformed or out-of-range files are rejected.

## Verification status

- Both quantized traps have positive Hessian eigenvalues at finite-difference
  steps from 0.2 to 0.7 mm.
- The weaker normalized stiffness eigenvalue is approximately 0.249.
- The midpoint potential is much higher than either trap potential, rather
  than forming a competitive central well.
- In 250 Monte Carlo trials at each error level, both traps stayed locally
  positive-definite through the harshest tested case: independent 20-degree
  RMS phase error and 15% RMS amplitude error per emitter.
- Five independent optimizer seeds all produced two positive-definite traps.

These are model tests, not proof of physical levitation. Absolute gravity
margin depends on measured acoustic pressure, EPS density, transducer phase and
amplitude calibration, reflections, and temperature. A 3 mm bead at 40 kHz has
`ka` near 1.1, so the Gor'kov model is useful but only marginally Rayleigh.

Bring-up should begin with the checked-in symmetric targets. Place one particle
in each well sequentially, starting with reduced voltage, and verify that both
remain captured before attempting motion or smaller target spacing.
