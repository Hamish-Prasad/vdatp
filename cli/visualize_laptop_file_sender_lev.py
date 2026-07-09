#!/usr/bin/env python3
"""Generate phase data and a quick field visualization for laptop_file_sender_lev.c."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np


PHASE_MAX = 512
X_COLS_MM = np.array([45.0, 35.0, 25.0, 15.0, 5.0])
Z_ROWS_MM = np.array([-45.0, -35.0, -25.0, -15.0, -5.0, 5.0, 15.0, 25.0, 35.0, 45.0])
TRANSDUCER_RADIUS_MM = 5.0
SOUND_SPEED_MM_S = 343000.0
FREQUENCY_HZ = 40000.0
BETA_MM2 = 3.0 * 0.957 * SOUND_SPEED_MM_S**2 / (2.0 * 0.965 * (2.0 * math.pi * FREQUENCY_HZ)**2)


def emitter_geometry(board_mm: float) -> tuple[np.ndarray, np.ndarray]:
    points = []
    normals = []
    for board in range(4):
        bottom = board in (2, 3)
        for channel in range(50):
            col = channel // 10
            if board == 0:
                x0 = X_COLS_MM[col]
            elif board == 1:
                x0 = -X_COLS_MM[4 - col]
            elif board == 2:
                x0 = X_COLS_MM[4 - col]
            else:
                x0 = -X_COLS_MM[col]
            points.append([
                x0,
                -0.5 * board_mm if bottom else 0.5 * board_mm,
                Z_ROWS_MM[channel % 10],
            ])
            normals.append([0.0, 1.0 if bottom else -1.0, 0.0])
    return np.asarray(points), np.asarray(normals)


def j0_series(x: np.ndarray) -> np.ndarray:
    term = np.ones_like(x, dtype=float)
    total = np.ones_like(x, dtype=float)
    xx = 0.25 * x * x
    for m in range(1, 24):
        term *= -xx / float(m * m)
        total += term
    return total


def transfer(points: np.ndarray, emitters: np.ndarray, normals: np.ndarray) -> np.ndarray:
    k = 2.0 * math.pi * FREQUENCY_HZ / SOUND_SPEED_MM_S
    delta = points[:, None, :] - emitters[None, :, :]
    radius = np.maximum(np.linalg.norm(delta, axis=2), 1.0e-9)
    cos_theta = np.clip(np.einsum("pij,ij->pi", delta, normals) / radius, 0.0, 1.0)
    sin_theta = np.sqrt(np.maximum(0.0, 1.0 - cos_theta * cos_theta))
    directivity = j0_series(k * TRANSDUCER_RADIUS_MM * sin_theta)
    return directivity * np.exp(1j * k * radius) / radius


def phase_ticks(target: np.ndarray, emitters: np.ndarray, normals: np.ndarray) -> np.ndarray:
    k = 2.0 * math.pi * FREQUENCY_HZ / SOUND_SPEED_MM_S
    delta = target[None, :] - emitters
    radius = np.maximum(np.linalg.norm(delta, axis=1), 1.0e-9)
    cos_theta = np.clip(np.einsum("ij,ij->i", delta, normals) / radius, 0.0, 1.0)
    sin_theta = np.sqrt(np.maximum(0.0, 1.0 - cos_theta * cos_theta))
    directivity = j0_series(k * TRANSDUCER_RADIUS_MM * sin_theta) / radius
    phase = -k * radius
    phase[directivity < 0.0] -= math.pi
    phase[100:] += math.pi
    ticks = np.rint(np.mod(phase, 2.0 * math.pi) * PHASE_MAX / (2.0 * math.pi)).astype(int)
    return np.mod(ticks, PHASE_MAX)


def potential(points: np.ndarray, drive: np.ndarray, emitters: np.ndarray, normals: np.ndarray,
              step_mm: float = 0.12) -> np.ndarray:
    offsets = np.vstack((np.zeros((1, 3)), np.eye(3), -np.eye(3))) * step_mm
    rows = transfer((points[:, None, :] + offsets[None, :, :]).reshape(-1, 3), emitters, normals)
    rows = rows.reshape(len(points), 7, -1)
    pressure = rows[:, 0] @ drive
    grad = np.stack([
        (rows[:, 1] - rows[:, 4]) @ drive,
        (rows[:, 2] - rows[:, 5]) @ drive,
        (rows[:, 3] - rows[:, 6]) @ drive,
    ], axis=1) / (2.0 * step_mm)
    return np.abs(pressure) ** 2 - BETA_MM2 * np.sum(np.abs(grad) ** 2, axis=1)


def make_plot(out_png: Path, grid_x: np.ndarray, grid_z: np.ndarray, u: np.ndarray,
              targets: np.ndarray, minima: np.ndarray) -> bool:
    try:
        import matplotlib
    except ModuleNotFoundError:
        return False

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7, 6), dpi=150)
    im = ax.imshow(u.reshape(len(grid_z), len(grid_x)), origin="lower",
                   extent=[grid_x[0], grid_x[-1], grid_z[0], grid_z[-1]],
                   cmap="viridis")
    ax.plot(targets[:, 0], targets[:, 2], "w.", markersize=5, label="commanded circle")
    ax.plot(minima[:, 0], minima[:, 2], "rx", markersize=6, label="nearest potential minima")
    ax.set_xlabel("x mm")
    ax.set_ylabel("z mm")
    ax.set_title("AcousticLev-style X-Z circle potential, y=0 slice")
    ax.set_aspect("equal")
    ax.legend(loc="upper right")
    fig.colorbar(im, ax=ax, label="normalized Gor'kov potential")
    fig.tight_layout()
    fig.savefig(out_png)
    plt.close(fig)
    return True


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--board-mm", type=float, default=135.0)
    parser.add_argument("--radius-mm", type=float, default=3.0)
    parser.add_argument("--frames", type=int, default=64)
    parser.add_argument("--out-dir", type=Path, default=Path("cli/lev_visualization"))
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    emitters, normals = emitter_geometry(args.board_mm)
    theta = np.linspace(0.0, 2.0 * math.pi, args.frames, endpoint=False)
    targets = np.column_stack((args.radius_mm * np.cos(theta), np.zeros_like(theta), args.radius_mm * np.sin(theta)))
    ticks = np.asarray([phase_ticks(target, emitters, normals) for target in targets], dtype=int)
    np.savetxt(args.out_dir / "circle_phase_ticks.csv", ticks, fmt="%d", delimiter=",")

    grid_x = np.linspace(-args.radius_mm - 3.0, args.radius_mm + 3.0, 121)
    grid_z = np.linspace(-args.radius_mm - 3.0, args.radius_mm + 3.0, 121)
    grid = np.stack(np.meshgrid(grid_x, grid_z, indexing="xy"), axis=-1).reshape(-1, 2)
    points = np.column_stack((grid[:, 0], np.zeros(len(grid)), grid[:, 1]))
    minima = []
    offsets = []
    for target, frame_ticks in zip(targets, ticks):
        drive = np.exp(1j * frame_ticks * 2.0 * math.pi / PHASE_MAX)
        local = points[np.linalg.norm(points[:, [0, 2]] - target[[0, 2]], axis=1) <= 2.0]
        u = potential(local, drive, emitters, normals)
        pmin = local[int(np.argmin(u))]
        minima.append(pmin)
        offsets.append(float(np.linalg.norm(pmin - target)))
    minima = np.asarray(minima)
    offsets = np.asarray(offsets)

    preview_index = 0
    preview_drive = np.exp(1j * ticks[preview_index] * 2.0 * math.pi / PHASE_MAX)
    u_preview = potential(points, preview_drive, emitters, normals)
    u_preview = (u_preview - np.min(u_preview)) / max(float(np.ptp(u_preview)), 1.0e-12)
    preview_png = args.out_dir / "circle_potential_preview.png"
    preview_written = make_plot(preview_png, grid_x, grid_z, u_preview, targets, minima)

    report = {
        "board_mm": args.board_mm,
        "radius_mm": args.radius_mm,
        "frames": args.frames,
        "phase_file": str(args.out_dir / "circle_phase_ticks.csv"),
        "preview_png": str(preview_png) if preview_written else None,
        "nearest_minimum_offset_mm": {
            "max": float(np.max(offsets)),
            "mean": float(np.mean(offsets)),
            "p95": float(np.percentile(offsets, 95)),
        },
        "first_frame_ticks": ticks[0].tolist(),
    }
    (args.out_dir / "circle_report.json").write_text(json.dumps(report, indent=2), encoding="ascii")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
