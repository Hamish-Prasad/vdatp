# New acoustic trap sender

This folder is intentionally separate from the existing `cli/laptop_phase_sender.c`.
It contains a C99-only solver and validator for multi-particle acoustic levitation
with the existing 200-channel phase-frame protocol.

## Files

- `acoustic_solver.h` / `acoustic_solver.c`: geometry, field model, phase synthesis,
  and Gor'kov-like validation helpers.
- `laptop_phase_sender_new.c`: TCP sender compatible with `phase_protocol.h`.
- `validate_traps.c`: offline validation that writes CSV metrics and PPM heatmaps.
- `quick_validate.py`: stdlib-only validation fallback used when no C compiler is
  available.
- `ppm_to_svg.py`: converts validator PPM heatmaps into browser-friendly SVGs.
- `visualize_3d.py`: creates rotatable, self-contained 3D HTML potential views.
- `pyvista_visualize_vdatp.py`: logged PyVista/VTK volume renderer styled after
  `D:/Acoustics29/Main`.
- `methods_replicate.md`: paper-replication method log.
- `methods_original.md`: original method log.

## Build

Windows MinGW:

```sh
gcc -O2 -std=c99 -Wall -Wextra acoustic_solver.c validate_traps.c -lm -o validate_traps.exe
gcc -O2 -std=c99 -Wall -Wextra acoustic_solver.c laptop_phase_sender_new.c -lws2_32 -lm -o laptop_phase_sender_new.exe
```

Linux/macOS:

```sh
gcc -O2 -std=c99 -Wall -Wextra acoustic_solver.c validate_traps.c -lm -o validate_traps
gcc -O2 -std=c99 -Wall -Wextra acoustic_solver.c laptop_phase_sender_new.c -lm -o laptop_phase_sender_new
```
gcc -O2 -std=c99 -Wall -Wextra acoustic_solver.c laptop_phase_sender_new.c -lws2_32 -lm -o laptop_phase_sender_new.exe
## Run

```sh
./validate_traps
python quick_validate.py
python pyvista_visualize_vdatp.py --method all --grid 49,31,41 --max-seconds 120 --screenshot
./laptop_phase_sender_new 169.254.181.37 5656 135
```

Inside the sender, `1` selects the literature-style twin-trap solver, `2`
selects shadow-nullspace, and `3` selects the stronger curvature-boost solver.
`+` and `-` change inter-particle spacing. `x/s`, `c/d`, and `z/a` translate
the full trap set.

The sender defaults to one curvature-boost trap at the origin. Pass `2` or `3`
as the final argument only after the single-particle trap is experimentally
stable.

## Current validation artifacts

- `replicate_wgs_3d.html`
- `shadow_nullspace_3d.html`
- `curvature_boost_3d.html`
- `*_metrics.csv`
- `*_gorkov_xz.svg`
- `replicate_wgs_pyvista_volume.png`
- `shadow_nullspace_pyvista_volume.png`
- `curvature_boost_pyvista_volume.png`
- `*_pyvista_volume.npz`
- `pyvista_screenshot_sparse.log`
