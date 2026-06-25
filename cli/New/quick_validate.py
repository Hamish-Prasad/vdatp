#!/usr/bin/env python3
"""Stdlib-only validator matching the C solver closely enough for review plots."""

from __future__ import annotations

import cmath
import csv
import math
from pathlib import Path

N = 200
PHASE_MAX = 512
X_COLS = [45.0, 35.0, 25.0, 15.0, 5.0]
Z_ROWS = [-45.0, -35.0, -25.0, -15.0, -5.0, 5.0, 15.0, 25.0, 35.0, 45.0]


def tpos(idx: int, board_distance: float = 135.0) -> tuple[float, float, float]:
    board, ch = divmod(idx, 50)
    x0 = X_COLS[ch // 10]
    x = x0 if board in (0, 2) else -x0
    y = -0.5 * board_distance if board in (2, 3) else 0.5 * board_distance
    return x, y, Z_ROWS[ch % 10]


def normal_sign(idx: int) -> float:
    return 1.0 if idx // 50 in (2, 3) else -1.0


def prop(idx: int, p: tuple[float, float, float]) -> complex:
    tx, ty, tz = tpos(idx)
    dx, dy, dz = p[0] - tx, p[1] - ty, p[2] - tz
    r = max(math.sqrt(dx * dx + dy * dy + dz * dz), 1e-6)
    k = 2.0 * math.pi * 40000.0 / 343.0 / 1000.0
    cos_theta = max(0.0, normal_sign(idx) * dy / r)
    return cos_theta * cmath.exp(1j * k * r) / r


def phase_only(s: list[complex]) -> list[complex]:
    return [cmath.exp(1j * math.atan2(z.imag, z.real)) for z in s]


def controls(method: str, traps: list[tuple[float, float, float]]):
    cps = []
    if method == "replicate-wgs":
        for x, y, z in traps:
            d = 2.3
            cps.append(((x - d, y, z), 1 + 0j, 1.0, False))
            cps.append(((x + d, y, z), -1 + 0j, 1.0, False))
            cps.append(((x, y, z), 0j, 0.45, True))
    elif method == "shadow-nullspace":
        dirs = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
        for i, (x, y, z) in enumerate(traps):
            cps.append(((x, y, z), 0j, 2.2, True))
            for d, (dx, dy, dz) in enumerate(dirs):
                a = 2.0 * math.pi * d / 6.0 + 0.73 * i
                cps.append(((x + 2.6 * dx, y + 2.6 * dy, z + 2.6 * dz), cmath.exp(1j * a), 0.85, False))
    else:
        dirs = [
            (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1),
            (1, 0, 1), (-1, 0, 1), (1, 0, -1), (-1, 0, -1),
        ]
        for i, (x, y, z) in enumerate(traps):
            cps.append(((x, y, z), 0j, 3.0, True))
            for d, (dx, dy, dz) in enumerate(dirs):
                norm = math.sqrt(dx * dx + dy * dy + dz * dz) or 1.0
                scale = 1.6 / norm
                a = math.pi * d / 3.0 + 0.73 * i
                weight = 1.4 if dy else 1.0
                cps.append(((x + scale * dx, y + scale * dy, z + scale * dz), cmath.exp(1j * a), weight, False))
    return cps


def make_phases(method: str, traps: list[tuple[float, float, float]]) -> list[int]:
    cps = controls(method, traps)
    iters = 32 if method == "replicate-wgs" else 1
    s = [1 + 0j] * N
    for _ in range(iters):
        s = [0j] * N
        for point, target, weight, is_null in cps:
            if is_null:
                continue
            for j in range(N):
                s[j] += prop(j, point).conjugate() * target * weight
        if method in ("shadow-nullspace", "curvature-boost"):
            passes = 6 if method == "curvature-boost" else 3
            reg = 0.02 if method == "curvature-boost" else 0.04
            for _pass in range(passes):
                for point, _target, _weight, is_null in cps:
                    if not is_null:
                        continue
                    rows = [prop(j, point) for j in range(N)]
                    pressure = sum(rows[j] * s[j] for j in range(N))
                    norm = reg + sum(abs(a) ** 2 for a in rows)
                    corr = pressure / norm
                    s = [s[j] - rows[j].conjugate() * corr * _weight for j in range(N)]
        s = phase_only(s)
    return [round((-math.atan2(z.imag, z.real)) * PHASE_MAX / (2.0 * math.pi)) % PHASE_MAX for z in s]


def pressure(phases: list[int], p: tuple[float, float, float]) -> complex:
    return sum(prop(j, p) * cmath.exp(-2j * math.pi * phases[j] / PHASE_MAX) for j in range(N))


def sample(phases: list[int], p: tuple[float, float, float]) -> tuple[float, float, float]:
    h = 0.35
    p0 = pressure(phases, p)
    gx = (pressure(phases, (p[0] + h, p[1], p[2])) - pressure(phases, (p[0] - h, p[1], p[2]))) / (2 * h)
    gy = (pressure(phases, (p[0], p[1] + h, p[2])) - pressure(phases, (p[0], p[1] - h, p[2]))) / (2 * h)
    gz = (pressure(phases, (p[0], p[1], p[2] + h)) - pressure(phases, (p[0], p[1], p[2] - h))) / (2 * h)
    pressure2 = abs(p0) ** 2
    grad2 = abs(gx) ** 2 + abs(gy) ** 2 + abs(gz) ** 2
    return pressure2, grad2, pressure2 - 0.58 * grad2


def score(phases: list[int], trap: tuple[float, float, float]) -> float:
    h = 0.8
    c = sample(phases, trap)[2]
    pts = [
        (trap[0] + h, trap[1], trap[2]), (trap[0] - h, trap[1], trap[2]),
        (trap[0], trap[1] + h, trap[2]), (trap[0], trap[1] - h, trap[2]),
        (trap[0], trap[1], trap[2] + h), (trap[0], trap[1], trap[2] - h),
    ]
    return (sum(sample(phases, p)[2] for p in pts) - 6.0 * c) / (h * h)


def write_ppm(path: Path, phases: list[int]) -> None:
    w = h = 181
    vals = []
    for iy in range(h):
        row = []
        z = -45.0 + 90.0 * iy / (h - 1)
        for ix in range(w):
            x = -45.0 + 90.0 * ix / (w - 1)
            row.append(sample(phases, (x, 0.0, z))[2])
        vals.append(row)
    flat = [v for row in vals for v in row]
    mn, mx = min(flat), max(flat)
    with path.open("w", encoding="ascii") as f:
        f.write(f"P3\n{w} {h}\n255\n")
        for row in vals:
            for v in row:
                t = (v - mn) / (mx - mn + 1e-30)
                r = int(255 * t)
                b = int(255 * (1 - t))
                g = int(120 * (1 - abs(2 * t - 1)))
                f.write(f"{r} {g} {b} ")
            f.write("\n")


def main() -> None:
    traps = [(-12.0, 0.0, 0.0), (0.0, 0.0, 0.0), (12.0, 0.0, 0.0)]
    cases = [
        ("replicate-wgs", "replicate_wgs"),
        ("shadow-nullspace", "shadow_nullspace"),
        ("curvature-boost", "curvature_boost"),
    ]
    for method, tag in cases:
        phases = make_phases(method, traps)
        with Path(f"{tag}_metrics.csv").open("w", newline="", encoding="ascii") as f:
            wr = csv.writer(f)
            wr.writerow(["method", "trap", "x_mm", "y_mm", "z_mm", "pressure2", "grad2", "gorkov_like", "curvature_score"])
            for i, trap in enumerate(traps):
                wr.writerow([method, i, *trap, *sample(phases, trap), score(phases, trap)])
        write_ppm(Path(f"{tag}_gorkov_xz.ppm"), phases)
        print(tag, "curvatures", [f"{score(phases, t):.6g}" for t in traps])


if __name__ == "__main__":
    main()
