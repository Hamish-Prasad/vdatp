# Methods Tried: Original Improvement

## Working name

Shadow-nullspace cage holography, upgraded with curvature-boost cage synthesis.

## Idea

The usual quick multi-focus approach superposes phase conjugates for several
foci. That creates traps, but each focus also becomes a disturbance for every
other focus. The new method treats the particle centers as "acoustic shadows":
pressure should be cancelled exactly there, while a surrounding shell receives
energy to build restoring gradients.

This borrows the null-space projection idea from antenna arrays, beamforming,
and optical holography, but applies it to Gor'kov trap construction for the
existing 200-channel airborne levitator.

## Implemented method

`NEW_TRAP_SHADOW_NULLSPACE` is non-iterative:

1. Put a zero-pressure control point at each particle center.
2. Put a six-point pressure cage around each particle, with staggered complex
   phases so neighbouring cages are less coherent.
3. Back-propagate the cage field to the transducers.
4. Project the transducer vector away from the center-null rows.
5. Convert the result to phase-only commands and quantize to 512 FPGA ticks.

The projection step is the important difference: it explicitly removes any
component of the drive vector that would illuminate the particle centers. The
remaining acoustic field is forced to spend its power in the cage/shell rather
than at the bead center.

`NEW_TRAP_CURVATURE_BOOST` is the stronger default variant:

1. Use a tighter 1.6 mm shell around each bead instead of the first 2.6 mm shell.
2. Add four diagonal x/z shell points, increasing transverse stiffness.
3. Give the center-null projection higher weight.
4. Apply six projection passes with lower regularization.
5. Quantize to the same 512 phase ticks, so the FPGA protocol is unchanged.

## Why it may improve stability

- Multi-particle cross-talk is handled as a linear constraint instead of being
  left as an interference accident.
- The six-point cage pushes the design toward positive 3D curvature of the
  Gor'kov-like potential, not just toward high pressure at a target point.
- The non-iterative form is suitable for real-time updates on the laptop.

## Results so far

- Added offline validator in `validate_traps.c`.
- Validator writes `shadow_nullspace_metrics.csv` and
  `shadow_nullspace_gorkov_xz.ppm`.
- The same three-particle geometry is evaluated for the paper-replication and
  original methods, making the metrics directly comparable.
- 2026-06-25 three-particle validation at x = -12, 0, +12 mm:
  - trap 0 curvature score: 0.863428
  - trap 1 curvature score: 0.640360
  - trap 2 curvature score: 0.352194
- Interpretation: all three target sites have positive local curvature in the
  simplified Gor'kov screen, while the literature-style baseline had two
  negative-curvature sites in the same geometry. This is the first candidate to
  try on real beads.
- 2026-06-26 strengthened shadow-nullspace after using null weights:
  - trap 0 curvature score: 1.04964
  - trap 1 curvature score: 0.813231
  - trap 2 curvature score: 0.628240
- 2026-06-26 curvature-boost validation at x = -12, 0, +12 mm:
  - trap 0 curvature score: 1.396080
  - trap 1 curvature score: 0.874339
  - trap 2 curvature score: 0.855982
  - weakest-trap improvement versus the 2026-06-25 shadow-nullspace result:
    about 2.43x.
- 2026-06-26 3D validation artifacts:
  - `shadow_nullspace_3d.html`
  - `curvature_boost_3d.html`
  - `shadow_nullspace_3d.3d_metrics.json`
  - `curvature_boost_3d.3d_metrics.json`

## Caveats

- The current field model is monopole/cosine-directivity with a small-angle
  piston correction. It is useful for comparing phase laws, but final force
  values need calibration against bead tracking.
- Gor'kov theory assumes particles smaller than wavelength and slow dynamics.
  Your 2-3.5 mm beads are below the 40 kHz wavelength in air, but not tiny, so
  the validator should be read as a design screen, not proof of perfection.
