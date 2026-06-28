#!/usr/bin/env python3
"""Quantitative verification and Acoustics29-style PyVista rendering."""

from __future__ import annotations

import argparse
import csv
import json
import time
from pathlib import Path

import numpy as np
import pyvista as pv

from vector_optimizer import CASES, GorkovModel, Physics, soft_nearest_vectors, target_set, vector_alignment


def evaluation_grid() -> np.ndarray:
    # Offset from the training lattice to test interpolation, not memorization.
    axes = (np.linspace(-33.0, 33.0, 14), np.linspace(-21.0, 21.0, 10), np.linspace(-33.0, 33.0, 14))
    return np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 3)


def simulate_trajectories(model: GorkovModel, phases: np.ndarray, case: str,
                          starts: np.ndarray, steps: int = 240) -> list[np.ndarray]:
    targets = target_set(case)
    paths = []
    for start in starts:
        p = start.copy()
        path = [p.copy()]
        for _ in range(steps):
            force = model.compile(p.reshape(1, 3)).force_and_jacobian(phases, jacobian=False)[0]
            norm = np.linalg.norm(force)
            if norm < 1.0e-12:
                break
            p += 0.75 * force / norm
            p = np.clip(p, (-38.0, -28.0, -38.0), (38.0, 28.0, 38.0))
            path.append(p.copy())
            _, distance = soft_nearest_vectors(p.reshape(1, 3), targets)
            if distance[0] < 1.8:
                break
        paths.append(np.asarray(path))
    return paths


def stiffness_eigenvalues(model: GorkovModel, phases: np.ndarray, target: np.ndarray,
                          h: float = 0.7) -> np.ndarray:
    offsets = np.vstack((np.eye(3), -np.eye(3))) * h
    force = model.compile(target[None, :] + offsets).force_and_jacobian(phases, jacobian=False)
    jac = np.column_stack(((force[0] - force[3]) / (2.0 * h),
                           (force[1] - force[4]) / (2.0 * h),
                           (force[2] - force[5]) / (2.0 * h)))
    stiffness = -0.5 * (jac + jac.T)
    return np.linalg.eigvalsh(stiffness)


def evaluate(case: str, results: Path) -> tuple[dict, np.ndarray, np.ndarray, list[np.ndarray]]:
    solution = np.load(results / f"{case}_solution.npz")
    phases = solution["phases"]
    model = GorkovModel(Physics())
    points = evaluation_grid()
    desired, distance = soft_nearest_vectors(points, target_set(case))
    keep = distance > 3.0
    force = np.empty_like(points)
    block = 180
    for begin in range(0, len(points), block):
        end = min(begin + block, len(points))
        force[begin:end] = model.compile(points[begin:end]).force_and_jacobian(phases, jacobian=False)
    fn = np.linalg.norm(force[keep], axis=1)
    cosine = np.sum(force[keep] * desired[keep], axis=1) / np.maximum(
        fn * np.linalg.norm(desired[keep], axis=1), 1.0e-12)
    starts = np.array([[-30, -18, -30], [-30, 18, 28], [30, -18, 30], [30, 18, -28],
                       [0, -20, 30], [0, 20, -30], [-28, 0, 0], [28, 0, 0]], dtype=float)
    trajectories = simulate_trajectories(model, phases, case, starts)
    final_distances = [float(soft_nearest_vectors(path[-1:].copy(), target_set(case))[1][0]) for path in trajectories]
    eigs = [stiffness_eigenvalues(model, phases, t).tolist() for t in target_set(case)[::max(1, len(target_set(case)) // 8)]]
    metrics = {
        "case": case,
        "mean_direction_cosine": float(np.mean(cosine)),
        "median_direction_cosine": float(np.median(cosine)),
        "inward_fraction": float(np.mean(cosine > 0.0)),
        "strong_alignment_fraction_cos_gt_0_8": float(np.mean(cosine > 0.8)),
        "force_magnitude_p10": float(np.percentile(fn, 10)),
        "force_magnitude_median": float(np.median(fn)),
        "trajectory_capture_fraction_1p8mm": float(np.mean(np.asarray(final_distances) < 1.8)),
        "trajectory_final_distances_mm": final_distances,
        "local_stiffness_eigenvalues_normalized": eigs,
        "warning": "Force and stiffness magnitudes are normalized until per-emitter pressure is calibrated.",
    }
    (results / f"{case}_verification.json").write_text(json.dumps(metrics, indent=2), encoding="ascii")
    return metrics, points, force, trajectories


def render(case: str, results: Path, points: np.ndarray, force: np.ndarray,
           trajectories: list[np.ndarray], screenshot: bool) -> None:
    physics = Physics()
    emitters, _ = GorkovModel(physics).emitters, None
    norm = np.linalg.norm(force, axis=1)
    vectors = force / np.maximum(norm[:, None], 1.0e-12)
    cloud = pv.PolyData(points)
    cloud["force"] = vectors
    cloud["strength"] = np.clip(norm / max(float(np.percentile(norm, 90)), 1.0e-12), 0.0, 1.2)
    arrows = cloud.glyph(orient="force", scale="strength", factor=4.0, geom=pv.Arrow())

    plotter = pv.Plotter(off_screen=screenshot, window_size=(1500, 1000))
    plotter.set_background("black")
    plotter.add_mesh(arrows, scalars="strength", cmap="viridis", clim=(0.0, 1.2), opacity=0.72)
    emitter_cloud = pv.PolyData(emitters)
    plotter.add_mesh(emitter_cloud.glyph(scale=False, orient=False, geom=pv.Sphere(radius=0.85)), color="white")
    targets = target_set(case)
    if case == "circle":
        loop = np.vstack((targets, targets[0]))
        plotter.add_mesh(pv.lines_from_points(loop, close=False), color="cyan", line_width=7)
    else:
        for target in targets:
            plotter.add_mesh(pv.Sphere(radius=2.0, center=target), color="cyan")
    colors = ("#ff4d4d", "#ffb000", "#00e5ff", "#ff66cc", "#7fff00", "#ff7f50", "#ab82ff", "#f5f5f5")
    for i, path in enumerate(trajectories):
        if len(path) > 1:
            plotter.add_mesh(pv.lines_from_points(path), color=colors[i % len(colors)], line_width=4)
        plotter.add_mesh(pv.Sphere(radius=1.5, center=path[0]), color=colors[i % len(colors)])
    plotter.add_axes()
    plotter.show_bounds(color="white", grid="back", location="outer", xtitle="X (mm)", ytitle="Y (mm)", ztitle="Z (mm)")
    plotter.camera_position = [(115, 95, 105), (0, 0, 0), (0, 1, 0)]
    if screenshot:
        plotter.show(screenshot=str(results / f"{case}_vector_field.png"), auto_close=True)
    else:
        plotter.show()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=CASES + ("all",), default="all")
    parser.add_argument("--results", type=Path, default=Path("results"))
    parser.add_argument("--screenshot", action="store_true")
    parser.add_argument("--max-seconds", type=float, default=240.0)
    args = parser.parse_args()
    started = time.monotonic()
    cases = CASES if args.case == "all" else (args.case,)
    rows = []
    for case in cases:
        if time.monotonic() - started > args.max_seconds:
            raise TimeoutError(f"verification exceeded {args.max_seconds:.0f} s")
        print(f"[{case}] evaluating vector field", flush=True)
        metrics, points, force, trajectories = evaluate(case, args.results)
        rows.append(metrics)
        render(case, args.results, points, force, trajectories, args.screenshot)
        print(f"[{case}] alignment={metrics['mean_direction_cosine']:.3f} capture={metrics['trajectory_capture_fraction_1p8mm']:.3f}", flush=True)
    with (args.results / "verification_summary.csv").open("w", newline="", encoding="ascii") as f:
        writer = csv.DictWriter(f, fieldnames=("case", "mean_direction_cosine", "median_direction_cosine",
            "inward_fraction", "strong_alignment_fraction_cos_gt_0_8", "force_magnitude_p10",
            "force_magnitude_median", "trajectory_capture_fraction_1p8mm"), extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    main()
