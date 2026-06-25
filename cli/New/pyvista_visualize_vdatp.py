#!/usr/bin/env python3
"""Logged PyVista-style VDATP volume renderer.

The render style mirrors D:/Acoustics29/Main: pv.ImageData volume, viridis
score, white emitters, cyan targets, red nearest minima, axes, and bounding box.

Use --headless to generate .npz volumes and CSV diagnostics without PyVista.
Use --screenshot after installing PyVista to write PNG renders.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

import numpy as np

try:
    import pyvista as pv
except ImportError:
    pv = None

import quick_validate as qv

METHODS = ("replicate-wgs", "shadow-nullspace", "curvature-boost")
SCALAR_NAME = "gorkov_well_score"


def log(msg: str, log_file: Path | None = None) -> None:
    line = f"[{time.strftime('%H:%M:%S')}] {msg}"
    print(line, flush=True)
    if log_file is not None:
        with log_file.open("a", encoding="utf-8") as f:
            f.write(line + "\n")


def parse_vec3_list(text: str) -> np.ndarray:
    out = []
    for item in text.split(";"):
        item = item.strip()
        if not item:
            continue
        vals = [float(v.strip()) for v in item.split(",")]
        if len(vals) != 3:
            raise ValueError(f"expected x,y,z target, got {item!r}")
        out.append(vals)
    if not out:
        raise ValueError("no targets supplied")
    return np.asarray(out, dtype=np.float64)


def parse_int3(text: str) -> tuple[int, int, int]:
    vals = tuple(int(v.strip()) for v in text.split(","))
    if len(vals) != 3:
        raise ValueError("expected three comma-separated integers")
    return vals


def parse_float3(text: str) -> tuple[float, float, float]:
    vals = tuple(float(v.strip()) for v in text.split(","))
    if len(vals) != 3:
        raise ValueError("expected three comma-separated floats")
    return vals


def emitters(board_distance_mm: float = 135.0) -> np.ndarray:
    return np.asarray([qv.tpos(i, board_distance_mm) for i in range(qv.N)], dtype=np.float64)


def make_grid(extent: tuple[float, float, float], shape: tuple[int, int, int]):
    xs = np.linspace(-0.5 * extent[0], 0.5 * extent[0], shape[0])
    ys = np.linspace(-0.5 * extent[1], 0.5 * extent[1], shape[1])
    zs = np.linspace(-0.5 * extent[2], 0.5 * extent[2], shape[2])
    X, Y, Z = np.meshgrid(xs, ys, zs, indexing="ij")
    coords = np.column_stack((X.ravel(order="F"), Y.ravel(order="F"), Z.ravel(order="F")))
    spacing = (xs[1] - xs[0], ys[1] - ys[0], zs[1] - zs[0])
    origin = (xs[0], ys[0], zs[0])
    return coords, spacing, origin


def propagator_block(points: np.ndarray, emitter_xyz: np.ndarray) -> np.ndarray:
    diff = points[:, None, :] - emitter_xyz[None, :, :]
    r = np.linalg.norm(diff, axis=2)
    r = np.maximum(r, 1e-9)
    k = 2.0 * np.pi * 40000.0 / 343.0 / 1000.0
    board = np.arange(qv.N) // 50
    normal = np.where((board == 2) | (board == 3), 1.0, -1.0)
    cos_theta = np.maximum(0.0, normal[None, :] * diff[:, :, 1] / r)
    return cos_theta * np.exp(1j * k * r) / r


def pressure_block(points: np.ndarray, emitter_xyz: np.ndarray, drive: np.ndarray) -> np.ndarray:
    return propagator_block(points, emitter_xyz) @ drive


def gorkov_block(points: np.ndarray, emitter_xyz: np.ndarray, drive: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    h = 0.35
    p0 = pressure_block(points, emitter_xyz, drive)
    offsets = np.asarray([[h, 0, 0], [-h, 0, 0], [0, h, 0], [0, -h, 0], [0, 0, h], [0, 0, -h]], dtype=np.float64)
    ps = [pressure_block(points + off[None, :], emitter_xyz, drive) for off in offsets]
    gx = (ps[0] - ps[1]) / (2.0 * h)
    gy = (ps[2] - ps[3]) / (2.0 * h)
    gz = (ps[4] - ps[5]) / (2.0 * h)
    pressure2 = np.abs(p0) ** 2
    grad2 = np.abs(gx) ** 2 + np.abs(gy) ** 2 + np.abs(gz) ** 2
    U = pressure2 - 0.58 * grad2
    force_proxy = pressure2 + grad2
    return U.real, force_proxy.real


def normalize(a: np.ndarray) -> np.ndarray:
    mn = float(np.min(a))
    mx = float(np.max(a))
    return (a - mn) / (mx - mn + 1e-30)


def phase_drive(method: str, targets: np.ndarray) -> tuple[list[int], np.ndarray]:
    phases = qv.make_phases(method, [tuple(t) for t in targets.tolist()])
    drive = np.exp(-2j * np.pi * np.asarray(phases, dtype=np.float64) / qv.PHASE_MAX)
    return phases, drive.astype(np.complex128)


def target_metrics(method: str, phases: list[int], targets: np.ndarray) -> list[dict[str, float | int | str]]:
    rows = []
    for i, t in enumerate(targets):
        trap = tuple(float(v) for v in t)
        pressure2, grad2, U = qv.sample(phases, trap)
        rows.append({
            "method": method,
            "trap": i,
            "x_mm": trap[0],
            "y_mm": trap[1],
            "z_mm": trap[2],
            "pressure2": pressure2,
            "grad2": grad2,
            "gorkov_like": U,
            "curvature_score": qv.score(phases, trap),
        })
    return rows


def nearest_minima(targets: np.ndarray, coords: np.ndarray, U: np.ndarray, radius: float = 8.0) -> np.ndarray:
    mins = []
    for target in targets:
        d = np.linalg.norm(coords - target[None, :], axis=1)
        cand = np.where(d <= radius)[0]
        idx = int(cand[np.argmin(U[cand])]) if len(cand) else int(np.argmin(d))
        mins.append(coords[idx])
    return np.asarray(mins, dtype=np.float64)


def write_metrics(path: Path, rows: list[dict[str, float | int | str]]) -> None:
    with path.open("w", newline="", encoding="ascii") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def compute_method(method: str, targets: np.ndarray, coords: np.ndarray, shape: tuple[int, int, int],
                   spacing, origin, out_dir: Path, block: int, max_seconds: float,
                   start_time: float, log_file: Path | None, display_percentile: float) -> None:
    log(f"{method}: synthesizing phases", log_file)
    phases, drive = phase_drive(method, targets)
    emitter_xyz = emitters()
    n = coords.shape[0]
    U = np.empty(n, dtype=np.float64)
    F = np.empty(n, dtype=np.float64)
    log(f"{method}: evaluating {n:,} grid points in blocks of {block}", log_file)
    for off in range(0, n, block):
        if max_seconds > 0 and time.monotonic() - start_time > max_seconds:
            raise TimeoutError(f"max runtime {max_seconds:.1f}s exceeded during {method} at point {off}/{n}")
        end = min(off + block, n)
        U[off:end], F[off:end] = gorkov_block(coords[off:end], emitter_xyz, drive)
        if off == 0 or end == n or (off // block) % 5 == 0:
            elapsed = time.monotonic() - start_time
            log(f"{method}: {end:,}/{n:,} points ({100*end/n:.1f}%), elapsed {elapsed:.1f}s", log_file)
    U_grid = U.reshape(shape, order="F")
    F_grid = F.reshape(shape, order="F")
    score_full = np.clip((1.0 - normalize(U_grid)) * (1.0 - 0.25 * normalize(F_grid)), 0.0, 1.0)
    cutoff = float(np.percentile(score_full, display_percentile))
    score = np.where(score_full >= cutoff, score_full, 0.0).astype(np.float32)
    minima = nearest_minima(targets, coords, U)
    rows = target_metrics(method, phases, targets)
    stem = method.replace("-", "_")
    write_metrics(out_dir / f"{stem}_pyvista_metrics.csv", rows)
    np.savez_compressed(
        out_dir / f"{stem}_pyvista_volume.npz",
        score=score,
        score_full=score_full.astype(np.float32),
        U=U_grid.astype(np.float32),
        targets=targets,
        minima=minima,
        emitters=emitter_xyz,
        spacing=np.asarray(spacing),
        origin=np.asarray(origin),
        display_percentile=np.asarray([display_percentile]),
        display_cutoff=np.asarray([cutoff]),
    )
    log(f"{method}: display percentile {display_percentile:.1f}, cutoff {cutoff:.5g}", log_file)
    log(f"{method}: wrote {stem}_pyvista_volume.npz and metrics", log_file)


def make_pv_grid(shape, spacing, origin, score):
    grid = pv.ImageData(dimensions=shape, spacing=spacing, origin=origin)
    grid.point_data[SCALAR_NAME] = score.ravel(order="F")
    return grid


def render_npz(path: Path, screenshot: bool, interactive: bool) -> None:
    if pv is None:
        raise RuntimeError("PyVista is not installed. Use --headless or install pyvista.")
    data = np.load(path)
    grid = make_pv_grid(tuple(data["score"].shape), tuple(data["spacing"]), tuple(data["origin"]), data["score"])
    plotter = pv.Plotter(title=path.stem, off_screen=screenshot and not interactive, window_size=(1500, 1000))
    plotter.set_background("black")
    plotter.add_axes()
    plotter.add_bounding_box(color="white")
    plotter.add_volume(grid, scalars=SCALAR_NAME, cmap="viridis",
                       opacity=[0, 0, 0.02, 0.05, 0.10, 0.18, 0.30, 0.45, 0.65, 0.85, 1.0],
                       shade=True)
    emitter_mesh = pv.Sphere(radius=1.0)
    plotter.add_mesh(pv.PolyData(data["emitters"]).glyph(geom=emitter_mesh), color="white")
    for i, t in enumerate(data["targets"]):
        plotter.add_mesh(pv.Sphere(radius=2.2, center=t), color="cyan", name=f"target_{i}")
    for i, m in enumerate(data["minima"]):
        plotter.add_mesh(pv.Sphere(radius=1.4, center=m), color="red", name=f"minimum_{i}")
    plotter.add_text(f"{path.stem} | cyan=targets red=nearest minima", position="upper_left", font_size=10)
    plotter.camera_position = "iso"
    plotter.camera.zoom(1.18)
    if screenshot:
        plotter.show(screenshot=str(path.with_suffix(".png")), auto_close=not interactive)
    if interactive:
        plotter.show()
    elif not screenshot:
        plotter.close()


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--targets", default="-12,0,0;0,0,0;12,0,0")
    p.add_argument("--method", choices=METHODS + ("all",), default="all")
    p.add_argument("--grid", default="49,31,41", help="nx,ny,nz; use 73,45,65 for slower high-res")
    p.add_argument("--extent", default="60,30,50", help="mm extent ex,ey,ez")
    p.add_argument("--block", type=int, default=2048)
    p.add_argument("--max-seconds", type=float, default=180.0, help="0 disables timeout")
    p.add_argument("--out-dir", default=".")
    p.add_argument("--log-file", default="pyvista_visualize_vdatp.log")
    p.add_argument("--display-percentile", type=float, default=88.0, help="zero display score below this percentile")
    p.add_argument("--headless", action="store_true", help="only write .npz/.csv volumes")
    p.add_argument("--screenshot", action="store_true", help="render PNG from generated .npz using PyVista")
    p.add_argument("--interactive", action="store_true", help="open interactive PyVista window")
    args = p.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    log_file = out_dir / args.log_file if args.log_file else None
    if log_file:
        log_file.write_text("", encoding="utf-8")
    targets = parse_vec3_list(args.targets)
    shape = parse_int3(args.grid)
    extent = parse_float3(args.extent)
    coords, spacing, origin = make_grid(extent, shape)
    methods = METHODS if args.method == "all" else (args.method,)
    start = time.monotonic()
    log(f"start: methods={methods}, grid={shape}, points={coords.shape[0]:,}, block={args.block}, max_seconds={args.max_seconds}", log_file)
    try:
        for method in methods:
            compute_method(method, targets, coords, shape, spacing, origin, out_dir, args.block,
                           args.max_seconds, start, log_file, args.display_percentile)
            if not args.headless and (args.screenshot or args.interactive):
                render_npz(out_dir / f"{method.replace('-', '_')}_pyvista_volume.npz", args.screenshot, args.interactive)
    except TimeoutError as e:
        log(f"TIMEOUT: {e}", log_file)
        return 124
    elapsed = time.monotonic() - start
    log(f"done in {elapsed:.1f}s", log_file)
    if not args.headless and not args.screenshot and not args.interactive:
        log("no render requested; pass --screenshot or --interactive after installing PyVista", log_file)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
