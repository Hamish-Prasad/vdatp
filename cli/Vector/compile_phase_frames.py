#!/usr/bin/env python3
"""Compile tracked bead positions into one quantized phase frame per bead."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np

from dynamic_vector_policy import DynamicVectorController
from vector_optimizer import CASES, target_set


def parse_positions(text: str) -> np.ndarray:
    points = []
    for item in text.split(";"):
        values = [float(value.strip()) for value in item.split(",")]
        if len(values) != 3:
            raise argparse.ArgumentTypeError("positions must be x,y,z;x,y,z in millimetres")
        points.append(values)
    return np.asarray(points, dtype=float)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=CASES, required=True)
    parser.add_argument("--positions", type=parse_positions, required=True,
                        help='tracked bead positions, e.g. "-20,0,4;22,3,-5"')
    parser.add_argument("--output", type=Path, default=Path("vector_phase_frames.csv"))
    args = parser.parse_args()

    controller = DynamicVectorController()
    targets = target_set(args.case, 180)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="ascii") as f:
        writer = csv.writer(f)
        writer.writerow(("particle", "axis_frame", "dwell_weight", "bead_x_mm", "bead_y_mm",
                         "bead_z_mm", "assigned_target", "waypoint_x_mm", "waypoint_y_mm",
                         "waypoint_z_mm", "channel", "phase_tick"))
        for particle, position in enumerate(args.positions):
            _force, tick_frames, dwell_weights, waypoint, nearest = controller.command(position, targets)
            for axis, ticks in enumerate(tick_frames):
                for channel, tick in enumerate(ticks):
                    writer.writerow((particle, controller.AXIS_NAMES[axis],
                        dwell_weights[axis] / np.sum(dwell_weights), *position, nearest, *waypoint,
                        channel, int(tick)))
    print(f"wrote {len(args.positions)} particles x 3 axis frames x 200 channels to {args.output}")


if __name__ == "__main__":
    main()
