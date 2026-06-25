# 3D Validation Results

Generated on 2026-06-26 with `quick_validate.py`, `visualize_3d.py`, and
`pyvista_visualize_vdatp.py`.

The 3D HTML files are self-contained and require no external libraries. They
show the lowest 18% of the sampled Gor'kov-like volume as a rotatable point
cloud. Gold points mark requested particle centers.

## Three-particle case

Target positions: x = -12, 0, +12 mm; y = 0 mm; z = 0 mm.

| method | trap 0 | trap 1 | trap 2 | weakest |
| --- | ---: | ---: | ---: | ---: |
| replicate-wgs | -0.797153 | 0.062989 | -0.797153 | -0.797153 |
| shadow-nullspace | 1.049640 | 0.813231 | 0.628240 | 0.628240 |
| curvature-boost | 1.396080 | 0.874339 | 0.855982 | 0.855982 |

## Interpretation

The literature-style twin baseline is not acceptable for simultaneous
three-particle levitation in this geometry because two local curvatures are
negative. Shadow-nullspace fixes the sign and creates real local wells.
Curvature-boost is currently the strongest method: the weakest well is positive
and about 2.43x stronger than the first shadow-nullspace result from 2026-06-25.

## One-particle setup

The sender now defaults to one curvature-boost particle at the origin. The
single-particle validation at x = 0, y = 0, z = 0 mm gives:

| method | curvature score |
| --- | ---: |
| replicate-wgs | 0.589963 |
| shadow-nullspace | 3.047576 |
| curvature-boost | 3.876013 |

This is the safest starting setup for real hardware testing.

## Artifacts

- `replicate_wgs_3d.html`
- `shadow_nullspace_3d.html`
- `curvature_boost_3d.html`
- `replicate_wgs_gorkov_xz.svg`
- `shadow_nullspace_gorkov_xz.svg`
- `curvature_boost_gorkov_xz.svg`
- `replicate_wgs_pyvista_volume.png`
- `shadow_nullspace_pyvista_volume.png`
- `curvature_boost_pyvista_volume.png`
- `replicate_wgs_pyvista_volume.npz`
- `shadow_nullspace_pyvista_volume.npz`
- `curvature_boost_pyvista_volume.npz`

## PyVista pass

`pyvista_visualize_vdatp.py` follows the render style used in
`D:/Acoustics29/Main`: `pv.ImageData`, `add_volume(..., cmap="viridis")`, white
emitter glyphs, cyan target markers, red nearest-minimum markers, axes, and a
white bounding box.

The first pure-Python attempt took too long. The replacement renderer evaluates
the field in NumPy blocks and logs progress. The successful screenshot run used:

```sh
python pyvista_visualize_vdatp.py --method all --grid 49,31,41 --max-seconds 120 --block 2048 --screenshot --display-percentile 88 --log-file pyvista_screenshot_sparse.log
```

Runtime was 23.1 seconds for all three methods on a 62,279-point grid. The
display volume keeps only the top 12% of well-score values so the traps are not
hidden inside an opaque block; the full score and Gor'kov-like scalar are still
stored in each `.npz`.
