# Vector Gor'kov Field Matching: methods, evidence, and research log

## Research vision

The objective is not merely a low Gor'kov value at a target. It is a *basin of attraction*: throughout a prescribed working volume, the radiation-force vector should guide a released 3 mm EPS bead toward the nearest allowed target set. This formulation supports one point, multiple Voronoi-like basins, and continuous paths with the same mathematics.

The working name is **Conservative Voronoi Gor'kov Field Matching (CV-GFM)**. This is a new synthesis assembled here; novelty in the publication sense has not yet been established by an exhaustive prior-art review or experiment.

After falsifying the global-static hypothesis, the primary method became **Dynamic Vector-Field Hologram Compilation (DVF-HC)**. DVF-HC treats the desired arrows as a feedback policy. At each camera observation, it selects the nearest target/path point and places a tri-axial twin-trap ensemble 1.0 mm ahead. The three phase-only frames split the aperture along X, Y, and Z; direction-adaptive dwell weights `[6|d_x|+1, |d_y|+1, 6|d_z|+1]` compensate for the much stronger opposed-array Y response while retaining useful force magnitude. Multiple beads receive interleaved ensembles. This preserves the requested global routing behavior without claiming that mutually incompatible arrows coexist in one static Helmholtz field.

## Physics

For a small sphere in an inviscid host fluid, the time-averaged force is

`F_rad = -grad(U_G)`

with

`U_G = V [f1 |p|^2/(4 rho_0 c_0^2) - 3 f2 |grad(p)|^2/(8 rho_0 omega^2)]`

for peak-amplitude complex pressure. The implementation divides out the positive pressure coefficient and optimizes the equivalent normalized potential

`U_hat = |p|^2 - beta |grad(p)|^2`,

`beta = 3 f2 c_0^2/(2 f1 omega^2)`.

The pressure is the linear superposition `p(x)=sum_n h_n(x) exp(i phi_n)`. Spatial central differences obtain `grad(p)` and `grad(U)`. The phase derivative of every quadratic field term is analytic, so no automatic-differentiation package is required.

The model uses 40 kHz, 343 m/s, 200 emitters in the exact four-board geometry used by `cli/New`, 135 mm board separation, 5 mm piston radius, 3 mm EPS diameter, density 25 kg/m^3, and nominal compressibility 3e-10 Pa^-1. EPS density varies substantially and must be measured for calibrated force prediction.

### Applicability warning

At 40 kHz, wavelength is about 8.575 mm and a 3 mm bead gives `ka` about 1.10. Gor'kov theory assumes a Rayleigh particle; this bead is only marginally within that modeling regime. The code therefore treats force magnitudes as normalized until the transducers and bead are calibrated. This limitation cannot responsibly be hidden behind a strong-looking rendering.

## Mathematics of the requested vectors

An arbitrary arrow drawing is generally not realizable because a Gor'kov force is conservative: away from singularities, `curl(F)=0`. Hard nearest-target arrows are also discontinuous on Voronoi boundaries.

CV-GFM starts from a realizable scalar navigation potential. For targets or sampled path points `t_j`,

`V_tau(x) = -tau log sum_j exp(-||x-t_j||^2/(2 tau))`.

Its negative gradient is a soft nearest-set vector field. As `tau` decreases it approaches the nearest point rule, while remaining differentiable enough for inverse design. A circle is represented by 96 target samples, creating a vector-defined path basin rather than a single focus.

The phase-only objective maximizes cosine alignment of all three force components at volumetric collocation points and separately enforces a low-tail force floor. Four phase holograms are optimized jointly; their forces are averaged under the explicit assumption that frame switching is fast compared with bead motion. This temporal diversity is intended to cancel wavelength-scale force reversals that a single static hologram cannot remove. Optimization uses mini-batch Adam, a cosine learning-rate schedule, focused/twin seeds, 512-level phase quantization, deterministic random seed, progress logs, and a wall-clock timeout.

## Verification protocol

Each case is evaluated on a denser grid not used for training. Reported quantities are:

- mean and median cosine between realized and requested vectors;
- fraction of points with inward force;
- fraction with cosine greater than 0.8;
- low-tail and median normalized force magnitude;
- capture fraction for eight trajectories released around the working volume;
- eigenvalues of the symmetric local stiffness tensor at target samples;
- performance after 512-level phase quantization.

PyVista renders emitters in white, targets/path in cyan, normalized force arrows with Viridis strength coloring, and colored bead trajectories. These images are diagnostic; the numerical metrics carry the evidential weight.

## Primary academic sources

1. L. P. Gor'kov, “On the forces acting on a small particle in an acoustical field in an ideal fluid,” *Soviet Physics Doklady* 6, 773 (1962; original 1961). Foundation of the small-sphere potential model.
2. T. Hoshi et al., “Three-Dimensional Mid-Air Acoustic Manipulation by Ultrasonic Phased Arrays,” *PLOS ONE* 9, e97590 (2014), DOI: 10.1371/journal.pone.0097590. Opposed PAT manipulation, Gor'kov force, gravity comparison, and 40 kHz measurements.
3. A. Marzo et al., “Holographic acoustic elements for manipulation of levitated objects,” *Nature Communications* 6, 8661 (2015), DOI: 10.1038/ncomms9661. Twin, bottle, and vortex traps from phased arrays.
4. A. Marzo and B. W. Drinkwater, “Holographic acoustic tweezers,” *PNAS* 116, 84-89 (2019), DOI: 10.1073/pnas.1813047115. Iterative back-propagation for multiple acoustic traps.
5. G. Memoli et al., “Acoustic levitation with optimized reflective metamaterials,” *Scientific Reports* 10, 3034 (2020), DOI: 10.1038/s41598-020-60978-4. Uses positive local Hessian/stiffness to rate trap quality and documents infeasible multi-trap configurations.
6. T. Fushimi, K. Yamamoto, and Y. Ochiai, “Acoustic hologram optimisation using automatic differentiation,” *Scientific Reports* 11, 12678 (2021), DOI: 10.1038/s41598-021-91880-2. Establishes differentiable phase-only hologram optimization and Adam in this domain.
7. T. Fushimi et al., “A digital twin approach for experimental acoustic hologram optimization,” *Communications Engineering* 3, 39 (2024), DOI: 10.1038/s44172-024-00160-0. Shows simulation-to-hardware offsets and experimental optimization; gives a contemporary Gor'kov formulation.
8. S. Zehnter et al., “Semidefinite programming for manipulating acoustic traps in real time (SMART),” *Scientific Reports* 15, 17523 (2025), DOI: 10.1038/s41598-025-93153-8. Frames inverse force prediction as ambiguous and introduces real-time optimization for acoustic traps.
9. T. Tang et al., “Acoustic levitation of axisymmetric Mie objects above a transducer array by engineering the acoustic radiation force and torque,” *Physical Review E* 106, 045108 (2022), DOI: 10.1103/PhysRevE.106.045108. Relevant route beyond Gor'kov for finite-sized objects.
10. E. Rimon and D. E. Koditschek, “Exact robot navigation using artificial potential functions,” *IEEE Transactions on Robotics and Automation* 8, 501-518 (1992), DOI: 10.1109/70.163777. Mathematical precedent for goal-directed navigation potentials and basin guarantees.
11. J. Sha et al., “Multi frame holograms batched optimization for binary phase spatial light modulators,” *Scientific Reports* 14, 19380 (2024), DOI: 10.1038/s41598-024-70428-0. Optical precedent for jointly optimized temporal hologram ensembles; CV-GFM transfers temporal diversity to acoustic force-vector matching.
12. “Finite-Inertia Corrections and Breakdown of Gor'kov Theory in Acoustic Levitation of Droplets,” arXiv:2601.00043 (2026 preprint). Recent warning that time-averaged Gor'kov dynamics needs a finite-inertia validity check; included as emerging, non-peer-reviewed work rather than settled evidence.

Direct source links:

- https://doi.org/10.1371/journal.pone.0097590
- https://doi.org/10.1038/ncomms9661
- https://doi.org/10.1073/pnas.1813047115
- https://doi.org/10.1038/s41598-020-60978-4
- https://doi.org/10.1038/s41598-021-91880-2
- https://doi.org/10.1038/s44172-024-00160-0
- https://doi.org/10.1038/s41598-025-93153-8
- https://doi.org/10.1103/PhysRevE.106.045108
- https://doi.org/10.1109/70.163777
- https://doi.org/10.1038/s41598-024-70428-0
- https://arxiv.org/abs/2601.00043

## Iteration log

### 2026-06-27: formulation and implementation

- Preserved `cli/New`; created an isolated `cli/Vector` research line.
- Replaced hard nearest arrows with a smooth conservative nearest-set potential.
- Derived and implemented analytic phase gradients of pressure and velocity quadratic terms.
- Added point, two-point, and circle-path target generators.
- Added independent-grid vector alignment, trajectory capture, local stiffness, quantization, timeout, logs, CSV/JSON/NPZ outputs, and PyVista views.
- A first static/coarse-grid smoke test reached 0.862 training alignment but only 0.063 on an offset dense grid and captured 0/8 trajectories. It was rejected as collocation aliasing rather than reported as success.
- Revised to sub-half-wavelength collocation and four-frame time-averaged force matching to attack the observed spatial ripple.
- The four-frame point case reached only 0.393 dense training alignment and -0.157 offset-grid alignment with 0/8 captures. This rejects the global-static hypothesis for the tested aperture/objective.
- A local-policy smoke test at 100 random positions produced inward force at 100/100 positions (mean cosine 0.602, median 0.546) using a 2 mm look-ahead twin trap and the same array propagator.
- Added DVF-HC policy evaluation, phase quantization, trajectory simulation, pressure-dependent force/weight estimates, and PyVista policy maps for point, two-point, and circle cases.
- Full quantized-model DVF-HC result: inward fraction 1.000 and 8/8 quasi-static trajectory captures for point, two-point, and circle cases. Mean direction cosines were 0.632, 0.671, and 0.775; minimum cosines were 0.461, 0.472, and 0.460.
- Under a 3000 Pa RMS focus calibration, estimated median force/weight ratios were 79.4, 85.8, and 82.2. These are conditional estimates, not measurements; force scales with pressure squared.
- Added a compiler that converts multiple tracked bead positions into interleaved 512-level phase frames.
- Pending: camera/latency integration, measured per-transducer phase/amplitude calibration, dynamic simulation with drag and gravity, scattering-aware finite-size validation, and hardware experiments.

### 2026-06-29: lateral-restoring correction

- Visual review exposed axial/Y dominance in the original one-frame policy. Its point-case mean direction cosine was 0.632, X/Z-dominant alignment was 0.619, and only 19.6% of voxels exceeded cosine 0.8.
- A continuously rotated binary aperture split improved the mean to about 0.80 but introduced four fully reversed symmetry-corner voxels, so it was rejected.
- The accepted tri-axial ensemble averages independent X/Y/Z twin traps. A fixed `4:1:4` dwell at 0.75 mm look-ahead achieved mean cosine 0.924 and minimum 0.823 but reduced the conditional median force to 17.3 uN.
- Direction-adaptive dwell `[6|d_x|+1, |d_y|+1, 6|d_z|+1]` at 1.0 mm retained mean cosine 0.923 and minimum 0.809 while increasing the conditional median to 26.3 uN and the 10th percentile to 9.3 uN in the point sweep. Every point remained above cosine 0.8.
- Revalidated the final quantized policy: one-point mean/minimum cosine 0.923/0.809 and two-point 0.921/0.803; both had inward fraction 1.000, every voxel above cosine 0.8, and 8/8 quasi-static captures. Conditional median force/weight margins were 7.59 and 9.11.
- Added `laptop_phase_sender_vector.c`, a standard-C implementation with only one- and two-particle modes, safe coordinate/range checks, CRC-checked protocol frames, adaptive tri-axial dwell, unique two-goal assignment, state-file tracking input, periodic logs, and a network-free self-test.
- Revised the sender interface so keyboard movement requires no state file and the Pi port is the explicit `--port` option. `--state FILE` now clearly selects camera-tracked closed-loop routing. Local mock-Pi tests accepted valid CRC/range-checked packets for one-particle manual mode, one-particle tracked mode, and all six frames of two-particle tracked mode.
