#!/usr/bin/env python3
"""Closed-loop compiler from nearest-set vectors to local phase holograms.

Each displayed vector is the time-averaged force from an X/Y/Z twin-trap
ensemble placed one short look-ahead step along the desired route. Multiple
beads use interleaved ensembles, one tracked bead per group of three frames.
"""

from __future__ import annotations

import argparse
import csv
import json
import time
from pathlib import Path

import numpy as np
import pyvista as pv

from vector_optimizer import CASES, GorkovModel, Physics, quantize_phases, target_set, transfer


def nearest_policy(points: np.ndarray, targets: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    delta = targets[None, :, :] - points[:, None, :]
    distances = np.linalg.norm(delta, axis=2)
    nearest = np.argmin(distances, axis=1)
    chosen = delta[np.arange(len(points)), nearest]
    distance = distances[np.arange(len(points)), nearest]
    direction = chosen / np.maximum(distance[:, None], 1.0e-12)
    return direction, distance, nearest


class DynamicVectorController:
    AXIS_NAMES = ("x", "y", "z")
    LATERAL_DWELL_GAIN = 6.0
    BASE_DWELL = 1.0

    def __init__(self, lookahead_mm: float = 1.0):
        self.physics = Physics()
        self.model = GorkovModel(self.physics)
        self.lookahead_mm = lookahead_mm
        center_row = transfer(np.zeros((1, 3)), self.physics, self.model.emitters, self.model.normals)[0]
        self.pressure_scale_pa = self.physics.reference_focus_rms_pa / np.sum(np.abs(center_row))
        radius_m = 0.5e-3 * self.physics.particle_diameter_mm
        volume_m3 = 4.0 * np.pi * radius_m**3 / 3.0
        self.force_scale_n_per_normalized = (volume_m3 * self.physics.f1 /
            (2.0 * self.physics.air_density_kg_m3 * self.physics.sound_speed_m_s**2)
            * self.pressure_scale_pa**2 * 1000.0)
        self.weight_n = volume_m3 * self.physics.particle_density_kg_m3 * 9.80665

    def command_frames(self, position_mm: np.ndarray, targets: np.ndarray):
        direction, distance, nearest = nearest_policy(position_mm.reshape(1, 3), targets)
        step = min(self.lookahead_mm, float(distance[0]))
        waypoint = position_mm + step * direction[0]
        row = transfer(waypoint.reshape(1, 3), self.physics,
                       self.model.emitters, self.model.normals)[0]
        focus = np.exp(-1j * np.angle(row))
        tick_frames, phase_frames = [], []
        for axis in range(3):
            aperture = self.model.emitters[:, axis] - waypoint[axis]
            drive = focus * np.where(aperture >= 0.0, 1.0, -1.0)
            ticks, phases = quantize_phases(np.angle(drive), self.physics.phase_levels)
            tick_frames.append(ticks)
            phase_frames.append(phases)
        compiled = self.model.compile(position_mm.reshape(1, 3))
        frame_forces = np.asarray([
            compiled.force_and_jacobian(phases, jacobian=False)[0]
            for phases in phase_frames
        ])
        dwell_weights = np.array([
            self.LATERAL_DWELL_GAIN * abs(direction[0, 0]) + self.BASE_DWELL,
            abs(direction[0, 1]) + self.BASE_DWELL,
            self.LATERAL_DWELL_GAIN * abs(direction[0, 2]) + self.BASE_DWELL,
        ])
        force = np.average(frame_forces, axis=0, weights=dwell_weights)
        return (force, np.asarray(tick_frames), dwell_weights, waypoint,
                int(nearest[0]), frame_forces)

    def command(self, position_mm: np.ndarray, targets: np.ndarray):
        force, ticks, dwell, waypoint, nearest, _frame_forces = self.command_frames(position_mm, targets)
        return force, ticks, dwell, waypoint, nearest

    def field(self, points: np.ndarray, targets: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        forces, waypoints = [], []
        for point in points:
            force, _ticks, _dwell, waypoint, _nearest = self.command(point, targets)
            forces.append(force)
            waypoints.append(waypoint)
        return np.asarray(forces), np.asarray(waypoints)


def grid() -> np.ndarray:
    axes = (np.linspace(-34.0, 34.0, 11), np.linspace(-24.0, 24.0, 7), np.linspace(-34.0, 34.0, 11))
    return np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 3)


def trajectories(controller: DynamicVectorController, case: str, steps: int = 180) -> list[np.ndarray]:
    targets = target_set(case, 180)
    starts = np.array([[-30, -18, -30], [-30, 18, 28], [30, -18, 30], [30, 18, -28],
                       [0, -20, 30], [0, 20, -30], [-28, 0, 0], [28, 0, 0]], dtype=float)
    paths = []
    for start in starts:
        point = start.copy()
        path = [point.copy()]
        for _ in range(steps):
            direction, distance, _ = nearest_policy(point.reshape(1, 3), targets)
            if distance[0] < 1.2:
                break
            force, _ticks, _dwell, _waypoint, _nearest = controller.command(point, targets)
            norm = np.linalg.norm(force)
            if norm < 1.0e-12:
                break
            # Quasi-static trajectory: direction comes from the modeled force;
            # speed is capped because drag and update rate are not calibrated.
            point += min(0.7, float(distance[0])) * force / norm
            path.append(point.copy())
        paths.append(np.asarray(path))
    return paths


def evaluate(case: str, out_dir: Path) -> tuple[dict, np.ndarray, np.ndarray, list[np.ndarray]]:
    controller = DynamicVectorController()
    targets = target_set(case, 180)
    points = grid()
    desired, distance, nearest = nearest_policy(points, targets)
    keep = distance > 2.5
    forces, waypoints = controller.field(points, targets)
    magnitude = np.linalg.norm(forces, axis=1)
    cosine = np.sum(forces * desired, axis=1) / np.maximum(magnitude, 1.0e-12)
    paths = trajectories(controller, case)
    final_distance = [float(nearest_policy(path[-1:], targets)[1][0]) for path in paths]
    force_n = magnitude * controller.force_scale_n_per_normalized
    metrics = {
        "case": case,
        "interpretation": "closed-loop vector-to-local-hologram policy; arrows are not simultaneous",
        "lookahead_mm": controller.lookahead_mm,
        "axis_dwell_rule": "[6*abs(dx)+1, abs(dy)+1, 6*abs(dz)+1]",
        "mean_direction_cosine": float(np.mean(cosine[keep])),
        "minimum_direction_cosine": float(np.min(cosine[keep])),
        "inward_fraction": float(np.mean(cosine[keep] > 0.0)),
        "strong_alignment_fraction_cos_gt_0_8": float(np.mean(cosine[keep] > 0.8)),
        "trajectory_capture_fraction_1p2mm": float(np.mean(np.asarray(final_distance) < 1.2)),
        "trajectory_final_distances_mm": final_distance,
        "estimated_force_median_uN": float(np.median(force_n[keep]) * 1.0e6),
        "estimated_force_p10_uN": float(np.percentile(force_n[keep], 10) * 1.0e6),
        "estimated_weight_uN": float(controller.weight_n * 1.0e6),
        "estimated_median_force_to_weight": float(np.median(force_n[keep]) / controller.weight_n),
        "pressure_calibration_assumption_rms_pa": controller.physics.reference_focus_rms_pa,
        "calibration_warning": "Absolute force scales with pressure squared and requires measurement on this hardware.",
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out_dir / f"{case}_dynamic_policy.npz", points=points, forces=forces,
                        desired=desired, waypoints=waypoints, targets=targets)
    (out_dir / f"{case}_dynamic_verification.json").write_text(json.dumps(metrics, indent=2), encoding="ascii")
    return metrics, points, forces, paths


def render(case: str, out_dir: Path, points: np.ndarray, forces: np.ndarray,
           paths: list[np.ndarray], screenshot: bool) -> None:
    controller = DynamicVectorController()
    norm = np.linalg.norm(forces, axis=1)
    cloud = pv.PolyData(points)
    cloud["policy_force"] = forces / np.maximum(norm[:, None], 1.0e-12)
    cloud["strength"] = np.clip(norm / max(float(np.percentile(norm, 90)), 1.0e-12), 0.0, 1.2)
    arrows = cloud.glyph(orient="policy_force", scale="strength", factor=4.5, geom=pv.Arrow())
    plotter = pv.Plotter(off_screen=screenshot, window_size=(1500, 1000))
    plotter.set_background("black")
    plotter.add_mesh(arrows, scalars="strength", cmap="viridis", clim=(0.0, 1.2), opacity=0.82)
    emitters = pv.PolyData(controller.model.emitters)
    plotter.add_mesh(emitters.glyph(scale=False, orient=False, geom=pv.Sphere(radius=0.85)), color="white")
    targets = target_set(case, 180)
    if case == "circle":
        plotter.add_mesh(pv.lines_from_points(np.vstack((targets, targets[0]))), color="cyan", line_width=7)
    else:
        for target in targets:
            plotter.add_mesh(pv.Sphere(radius=2.0, center=target), color="cyan")
    colors = ("#ff4d4d", "#ffb000", "#00e5ff", "#ff66cc", "#7fff00", "#ff7f50", "#ab82ff", "#f5f5f5")
    for i, path in enumerate(paths):
        plotter.add_mesh(pv.lines_from_points(path), color=colors[i], line_width=4)
        plotter.add_mesh(pv.Sphere(radius=1.5, center=path[0]), color=colors[i])
    plotter.add_axes()
    plotter.show_bounds(color="white", grid="back", location="outer", xtitle="X (mm)", ytitle="Y (mm)", ztitle="Z (mm)")
    plotter.camera_position = [(115, 95, 105), (0, 0, 0), (0, 1, 0)]
    if screenshot:
        plotter.show(screenshot=str(out_dir / f"{case}_dynamic_vector_policy.png"), auto_close=True)
    else:
        plotter.show()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=CASES + ("all",), default="all")
    parser.add_argument("--out-dir", type=Path, default=Path("results"))
    parser.add_argument("--screenshot", action="store_true")
    parser.add_argument("--max-seconds", type=float, default=180.0)
    args = parser.parse_args()
    cases = CASES if args.case == "all" else (args.case,)
    started = time.monotonic()
    rows = []
    for case in cases:
        if time.monotonic() - started > args.max_seconds:
            raise TimeoutError(f"dynamic verification exceeded {args.max_seconds:.0f} s")
        print(f"[{case}] compiling closed-loop vector policy", flush=True)
        metrics, points, forces, paths = evaluate(case, args.out_dir)
        render(case, args.out_dir, points, forces, paths, args.screenshot)
        rows.append(metrics)
        print(f"[{case}] inward={metrics['inward_fraction']:.3f} capture={metrics['trajectory_capture_fraction_1p2mm']:.3f}", flush=True)
    with (args.out_dir / "dynamic_verification_summary.csv").open("w", newline="", encoding="ascii") as f:
        fields = ("case", "mean_direction_cosine", "minimum_direction_cosine", "inward_fraction",
                  "strong_alignment_fraction_cos_gt_0_8", "trajectory_capture_fraction_1p2mm",
                  "estimated_force_median_uN", "estimated_force_p10_uN", "estimated_weight_uN",
                  "estimated_median_force_to_weight")
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    main()
