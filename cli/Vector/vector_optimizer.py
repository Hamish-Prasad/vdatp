#!/usr/bin/env python3
"""Phase-only inverse design of volumetric Gor'kov force-vector fields.

The optimizer matches -grad(U) to a conservative nearest-target-set field.
Only NumPy is required. Coordinates exposed by the CLI are in millimetres.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class Physics:
    frequency_hz: float = 40_000.0
    sound_speed_m_s: float = 343.0
    air_density_kg_m3: float = 1.204
    particle_density_kg_m3: float = 25.0
    particle_compressibility_pa_inv: float = 3.0e-10
    particle_diameter_mm: float = 3.0
    transducer_radius_mm: float = 5.0
    board_distance_mm: float = 135.0
    phase_levels: int = 512
    reference_focus_rms_pa: float = 3000.0

    @property
    def wavelength_mm(self) -> float:
        return 1000.0 * self.sound_speed_m_s / self.frequency_hz

    @property
    def k_mm(self) -> float:
        return 2.0 * math.pi / self.wavelength_mm

    @property
    def f1(self) -> float:
        kappa_air = 1.0 / (self.air_density_kg_m3 * self.sound_speed_m_s**2)
        return 1.0 - self.particle_compressibility_pa_inv / kappa_air

    @property
    def f2(self) -> float:
        ratio = self.particle_density_kg_m3 / self.air_density_kg_m3
        return 2.0 * (ratio - 1.0) / (2.0 * ratio + 1.0)

    @property
    def beta_mm2(self) -> float:
        omega = 2.0 * math.pi * self.frequency_hz
        beta_m2 = 3.0 * self.f2 * self.sound_speed_m_s**2 / (2.0 * self.f1 * omega**2)
        return beta_m2 * 1.0e6

    @property
    def rayleigh_ka(self) -> float:
        return self.k_mm * 0.5 * self.particle_diameter_mm


CASES = ("point", "two-points", "circle")


def emitter_geometry(physics: Physics) -> tuple[np.ndarray, np.ndarray]:
    x_cols = np.array([45.0, 35.0, 25.0, 15.0, 5.0])
    z_rows = np.arange(-45.0, 46.0, 10.0)
    xyz, normals = [], []
    for board in range(4):
        for channel in range(50):
            x0 = x_cols[channel // 10]
            lower = board in (2, 3)
            xyz.append((x0 if board in (0, 2) else -x0,
                        -0.5 * physics.board_distance_mm if lower else 0.5 * physics.board_distance_mm,
                        z_rows[channel % 10]))
            normals.append((0.0, 1.0 if lower else -1.0, 0.0))
    return np.asarray(xyz), np.asarray(normals)


def transfer(points_mm: np.ndarray, physics: Physics, emitters: np.ndarray,
             normals: np.ndarray) -> np.ndarray:
    delta = points_mm[:, None, :] - emitters[None, :, :]
    radius = np.maximum(np.linalg.norm(delta, axis=2), 1.0e-6)
    cos_theta = np.maximum(np.einsum("mnj,nj->mn", delta, normals) / radius, 0.0)
    # Cosine baffled-piston approximation, matching the deployed New solver.
    return cos_theta * np.exp(1j * physics.k_mm * radius) / radius


class GorkovModel:
    def __init__(self, physics: Physics, force_step_mm: float = 0.55,
                 pressure_step_mm: float = 0.18):
        self.physics = physics
        self.force_step_mm = force_step_mm
        self.pressure_step_mm = pressure_step_mm
        self.emitters, self.normals = emitter_geometry(physics)

    def _u_samples(self, points_mm: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        offsets = np.vstack((np.eye(3), -np.eye(3))) * self.force_step_mm
        u_points = (points_mm[:, None, :] + offsets[None, :, :]).reshape(-1, 3)
        h = self.pressure_step_mm
        p_offsets = np.vstack((np.zeros((1, 3)), np.eye(3) * h, -np.eye(3) * h))
        sample_points = (u_points[:, None, :] + p_offsets[None, :, :]).reshape(-1, 3)
        rows = transfer(sample_points, self.physics, self.emitters, self.normals)
        rows = rows.reshape(len(u_points), 7, -1)
        h0 = rows[:, 0]
        grad = np.stack(((rows[:, 1] - rows[:, 4]) / (2.0 * h),
                         (rows[:, 2] - rows[:, 5]) / (2.0 * h),
                         (rows[:, 3] - rows[:, 6]) / (2.0 * h)), axis=1)
        return h0, grad

    def compile(self, points_mm: np.ndarray) -> "CompiledForce":
        h0, grad = self._u_samples(np.asarray(points_mm, dtype=float))
        return CompiledForce(self, np.asarray(points_mm, dtype=float), h0, grad)

    def focus_drive(self, target_mm: np.ndarray) -> np.ndarray:
        row = transfer(np.asarray(target_mm, dtype=float).reshape(1, 3), self.physics,
                       self.emitters, self.normals)[0]
        q = np.exp(-1j * np.angle(row))
        # A pi split between opposed boards seeds a pressure node/twin trap.
        q[:100] *= -1.0
        return q


class CompiledForce:
    def __init__(self, model: GorkovModel, points: np.ndarray, h0: np.ndarray,
                 grad: np.ndarray):
        self.model, self.points, self.h0, self.grad = model, points, h0, grad

    def subset(self, indices: np.ndarray) -> "CompiledForce":
        h0 = self.h0.reshape(len(self.points), 6, -1)[indices].reshape(-1, self.h0.shape[1])
        grad = self.grad.reshape(len(self.points), 6, 3, -1)[indices].reshape(-1, 3, self.h0.shape[1])
        return CompiledForce(self.model, self.points[indices], h0, grad)

    def _single_force_and_jacobian(self, phases: np.ndarray, jacobian: bool):
        q = np.exp(1j * phases)
        pressure = self.h0 @ q
        velocity_term = np.einsum("mdi,i->md", self.grad, q)
        beta = self.model.physics.beta_mm2
        u = np.abs(pressure)**2 - beta * np.sum(np.abs(velocity_term)**2, axis=1)
        u = u.reshape(len(self.points), 6)
        h = self.model.force_step_mm
        force = -(u[:, :3] - u[:, 3:]) / (2.0 * h)
        if not jacobian:
            return force

        z_p = np.conj(pressure)[:, None] * self.h0 * q[None, :]
        z_g = np.sum(np.conj(velocity_term)[:, :, None] * self.grad, axis=1) * q[None, :]
        du = -2.0 * np.imag(z_p) + 2.0 * beta * np.imag(z_g)
        du = du.reshape(len(self.points), 6, -1)
        dforce = -(du[:, :3] - du[:, 3:]) / (2.0 * h)
        return force, dforce

    def force_and_jacobian(self, phases: np.ndarray, jacobian: bool = True):
        phase_frames = np.atleast_2d(phases)
        if not jacobian:
            forces = [self._single_force_and_jacobian(frame, False) for frame in phase_frames]
            return np.mean(forces, axis=0)
        pairs = [self._single_force_and_jacobian(frame, True) for frame in phase_frames]
        force = np.mean([pair[0] for pair in pairs], axis=0)
        frame_jacobian = np.stack([pair[1] for pair in pairs], axis=2) / len(phase_frames)
        return force, frame_jacobian


def target_set(case: str, path_samples: int = 96) -> np.ndarray:
    if case == "point":
        return np.array([[0.0, 0.0, 0.0]])
    if case == "two-points":
        return np.array([[-16.0, 0.0, 0.0], [16.0, 0.0, 0.0]])
    if case == "circle":
        theta = np.linspace(0.0, 2.0 * np.pi, path_samples, endpoint=False)
        return np.column_stack((18.0 * np.cos(theta), np.zeros_like(theta), 18.0 * np.sin(theta)))
    raise ValueError(f"unknown case: {case}")


def soft_nearest_vectors(points: np.ndarray, targets: np.ndarray,
                         temperature_mm2: float = 5.0) -> tuple[np.ndarray, np.ndarray]:
    delta = points[:, None, :] - targets[None, :, :]
    d2 = np.sum(delta * delta, axis=2)
    logits = -(d2 - np.min(d2, axis=1, keepdims=True)) / (2.0 * temperature_mm2)
    weights = np.exp(np.clip(logits, -80.0, 0.0))
    weights /= np.sum(weights, axis=1, keepdims=True)
    nearest_delta = np.sum(weights[:, :, None] * delta, axis=1)
    distance = np.linalg.norm(nearest_delta, axis=1)
    direction = -nearest_delta / np.maximum(distance[:, None], 1.0e-9)
    magnitude = np.clip(distance / 12.0, 0.18, 1.0)
    return direction * magnitude[:, None], distance


def training_points(case: str) -> np.ndarray:
    # Sub-half-wavelength spacing prevents a solver from hiding force reversals
    # between collocation points.
    x = np.linspace(-32.0, 32.0, 15)
    y = np.linspace(-22.0, 22.0, 11)
    z = np.linspace(-32.0, 32.0, 15)
    grid = np.stack(np.meshgrid(x, y, z, indexing="ij"), axis=-1).reshape(-1, 3)
    targets = target_set(case)
    _, distance = soft_nearest_vectors(grid, targets)
    return grid[distance > 3.0]


def quantize_phases(phases: np.ndarray, levels: int) -> tuple[np.ndarray, np.ndarray]:
    wrapped = np.mod(phases, 2.0 * np.pi)
    ticks = np.mod(np.rint(wrapped * levels / (2.0 * np.pi)).astype(np.int32), levels)
    return ticks, ticks.astype(float) * (2.0 * np.pi / levels)


def optimize(case: str, out_dir: Path, iterations: int, seed: int,
             max_seconds: float, frames: int = 4, batch_size: int = 320,
             log_every: int = 25) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    physics = Physics()
    model = GorkovModel(physics)
    points = training_points(case)
    desired_unit, distance = soft_nearest_vectors(points, target_set(case))
    compiled = model.compile(points)
    rng = np.random.default_rng(seed)
    base = np.angle(model.focus_drive(target_set(case)[0]))
    phases = base[None, :] + rng.normal(0.0, 0.35, (frames, 200))
    initial_force = compiled.force_and_jacobian(phases, jacobian=False)
    scale = max(float(np.percentile(np.linalg.norm(initial_force, axis=1), 65.0)), 1.0e-10)
    desired_norm = np.linalg.norm(desired_unit, axis=1)
    desired_direction = desired_unit / np.maximum(desired_norm[:, None], 1.0e-12)

    m = np.zeros_like(phases)
    v = np.zeros_like(phases)
    best = (float("inf"), phases.copy())
    history = []
    started = time.monotonic()
    completed = 0
    for step in range(1, iterations + 1):
        if time.monotonic() - started > max_seconds:
            print(f"[{case}] timeout at iteration {step - 1}", flush=True)
            break
        indices = rng.choice(len(points), size=min(batch_size, len(points)), replace=False)
        batch = compiled.subset(indices)
        force, jac = batch.force_and_jacobian(phases, jacobian=True)
        batch_direction = desired_direction[indices]
        force_norm = np.linalg.norm(force, axis=1)
        cosine = np.sum(force * batch_direction, axis=1) / np.maximum(force_norm, 1.0e-12)
        direction_loss = float(np.mean(1.0 - cosine))
        grad_force = -(batch_direction / np.maximum(force_norm[:, None], 1.0e-12)
                       - cosine[:, None] * force / np.maximum(force_norm[:, None] ** 2, 1.0e-12)) / len(force)

        # Cosine alone admits vanishing forces. Preserve a low-tail force floor.
        force_floor = 0.18 * scale
        shortfall = np.maximum((force_floor - force_norm) / scale, 0.0)
        magnitude_loss = float(np.mean(shortfall * shortfall))
        active = shortfall > 0.0
        if np.any(active):
            grad_force[active] += (0.30 * 2.0 * shortfall[active, None]
                * (-force[active] / np.maximum(force_norm[active, None], 1.0e-12))
                / scale / len(force))
        loss = direction_loss + 0.30 * magnitude_loss
        grad_loss = np.einsum("pdfk,pd->fk", jac, grad_force)
        grad_loss -= np.mean(grad_loss, axis=1, keepdims=True)  # per-frame global phase is null
        grad_loss = np.clip(grad_loss, -2.0, 2.0)

        m = 0.9 * m + 0.1 * grad_loss
        v = 0.999 * v + 0.001 * grad_loss * grad_loss
        mhat = m / (1.0 - 0.9**step)
        vhat = v / (1.0 - 0.999**step)
        lr = 0.045 * (0.25 + 0.75 * 0.5 * (1.0 + math.cos(math.pi * step / iterations)))
        phases -= lr * mhat / (np.sqrt(vhat) + 1.0e-8)
        phases = np.mod(phases + np.pi, 2.0 * np.pi) - np.pi
        completed = step

        if step == 1 or step % log_every == 0:
            full_force = compiled.force_and_jacobian(phases, jacobian=False)
            alignment = vector_alignment(full_force, desired_direction)
            validation_loss = 1.0 - alignment
            if validation_loss < best[0]:
                best = (validation_loss, phases.copy())
            history.append((step, loss, alignment))
            print(f"[{case}] {step:4d}/{iterations} batch_loss={loss:.6f} full_alignment={alignment:.3f}", flush=True)

    ticks, phases_q = quantize_phases(best[1], physics.phase_levels)
    elapsed = time.monotonic() - started
    with (out_dir / f"{case}_phases.csv").open("w", newline="", encoding="ascii") as f:
        writer = csv.writer(f)
        writer.writerow(("frame", "channel", "phase_tick", "phase_rad"))
        writer.writerows((frame, channel, int(ticks[frame, channel]), float(phases_q[frame, channel]))
                         for frame in range(frames) for channel in range(200))
    np.savez_compressed(out_dir / f"{case}_solution.npz", phases=phases_q, ticks=ticks,
                        training_points=points, desired_vectors=desired_unit,
                        targets=target_set(case), history=np.asarray(history),
                        force_scale=np.asarray([scale]))
    result = {"case": case, "best_loss": best[0], "iterations_completed": completed,
              "elapsed_seconds": elapsed, "physics": asdict(physics), "beta_mm2": physics.beta_mm2,
              "rayleigh_ka": physics.rayleigh_ka, "frames": frames,
              "phase_file": f"{case}_phases.csv"}
    (out_dir / f"{case}_optimization.json").write_text(json.dumps(result, indent=2), encoding="ascii")
    return result


def vector_alignment(force: np.ndarray, desired: np.ndarray) -> float:
    fn = np.linalg.norm(force, axis=1)
    dn = np.linalg.norm(desired, axis=1)
    cosine = np.sum(force * desired, axis=1) / np.maximum(fn * dn, 1.0e-12)
    return float(np.mean(cosine))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=CASES + ("all",), default="all")
    parser.add_argument("--iterations", type=int, default=450)
    parser.add_argument("--seed", type=int, default=29)
    parser.add_argument("--max-seconds", type=float, default=180.0)
    parser.add_argument("--frames", type=int, default=4)
    parser.add_argument("--batch-size", type=int, default=320)
    parser.add_argument("--out-dir", type=Path, default=Path("results"))
    args = parser.parse_args()
    cases = CASES if args.case == "all" else (args.case,)
    for case in cases:
        optimize(case, args.out_dir, args.iterations, args.seed, args.max_seconds,
                 args.frames, args.batch_size)


if __name__ == "__main__":
    main()
