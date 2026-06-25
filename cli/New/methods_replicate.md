# Methods Tried: Literature Replication

## Sources read

- Marzo et al., "Holographic acoustic elements for manipulation of levitated
  objects", Nature Communications, 2015. Key idea used here: single-sided
  twin/vortex/bottle traps are produced by adding phase structure around a focus.
- Marzo and Drinkwater, "Holographic acoustic tweezers", PNAS, 2019. Key idea:
  phased arrays can create full 3D manipulation and multiple millimetric
  particles by shaping holographic traps.
- Marrara et al., "Optical Calibration of Holographic Acoustic Tweezers", 2024.
  Useful details: 40 kHz arrays, circular-piston pressure model, styrofoam bead
  density around 36 kg/m^3, and Gor'kov-potential-based force/stiffness checks.
- Stone et al., "Experimental and Numerical Study of Acoustic Streaming in
  Mid-Air Phased Arrays", 2025. Design warning: multi-focus fields can create
  streaming and lateral jets, so validation should inspect sidelobes rather than
  only the target pressure.
- Williams, "Finite-Inertia Corrections and Breakdown of Gor'kov Theory in
  Acoustic Levitation of Droplets", 2026 preprint. Design warning: Gor'kov is a
  slow-time approximation, so practical tests should still check bead motion and
  not rely on a single static scalar plot.

## Implemented method

`NEW_TRAP_REPLICATE_WGS` implements a conservative twin-trap reproduction:

1. For every desired particle position, create two opposed pressure lobes along
   the local x direction.
2. Add a lower-weight pressure-null control point at the bead center.
3. Generate phase-only transducer drives by repeated weighted matched-field
   back-propagation.
4. Quantize to the FPGA's 512 phase ticks.

This is deliberately close to the holographic-acoustic-tweezers family: it
uses pressure shaping around each particle rather than a single path-length
focus. It should be much closer to the published twin trap than the old sender.

## Results so far

- Added offline validator in `validate_traps.c`.
- Validator reports a curvature-like score at each requested particle site and
  writes `replicate_wgs_metrics.csv` plus `replicate_wgs_gorkov_xz.ppm`.
- The score is dimensionless because the hardware transducer pressure constant
  is not calibrated yet. Positive relative curvature means the local potential
  rises around the center in the sampled stencil.
- 2026-06-25 three-particle validation at x = -12, 0, +12 mm:
  - trap 0 curvature score: -0.797153
  - trap 1 curvature score: 0.062989
  - trap 2 curvature score: -0.797153
- Interpretation: this reproduction forms the central null well, but the two
  outer traps are locally unstable in the simplified Gor'kov screen. It is still
  useful as a baseline because it resembles the usual phase-superposition trap.
- 2026-06-26 3D validation added:
  - `replicate_wgs_3d.html`
  - `replicate_wgs_3d.3d_metrics.json`
  - low-potential volume cutoff: -0.00184649
  - conclusion unchanged: useful reference, not a reliable multi-particle trap.

## Next experimental checks

- Run the sender with one bead first, then two beads at 12-16 mm spacing.
- Check whether bead oscillations are lower than the old sender at the same
  drive voltage.
- If two traps interact, increase spacing or use the shadow-nullspace method.
