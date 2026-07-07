#!/usr/bin/env python3
"""Compile one static, phase-only hologram with two locally stable traps.

The objective is deliberately local: maximise the weakest eigenvalue of the
Gor'kov-potential Hessian at either requested trap while driving the residual
force at both trap centres towards zero.  This is a better levitation target
than matching a vector field throughout the whole chamber.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict
from pathlib import Path

import numpy as np

from Vector.vector_optimizer import Physics, emitter_geometry, quantize_phases, transfer


def sample_stencil(targets: np.ndarray, h: float) -> tuple[np.ndarray, list[dict]]:
    points: list[np.ndarray] = []
    maps: list[dict] = []
    for target in targets:
        m: dict[tuple, int] = {}
        m[(0,)] = len(points); points.append(target)
        for a in range(3):
            for s in (-1, 1):
                p = target.copy(); p[a] += s*h
                m[(a, s)] = len(points); points.append(p)
        for a in range(3):
            for b in range(a + 1, 3):
                for sa in (-1, 1):
                    for sb in (-1, 1):
                        p = target.copy(); p[a] += sa*h; p[b] += sb*h
                        m[(a, b, sa, sb)] = len(points); points.append(p)
        maps.append(m)
    return np.asarray(points), maps


class PotentialModel:
    def __init__(self, physics: Physics, points: np.ndarray, derivative_step_mm: float = 0.18):
        self.physics = physics
        emitters, normals = emitter_geometry(physics)
        offsets = np.vstack((np.zeros((1, 3)), np.eye(3), -np.eye(3))) * derivative_step_mm
        samples = (points[:, None, :] + offsets[None, :, :]).reshape(-1, 3)
        rows = transfer(samples, physics, emitters, normals).reshape(len(points), 7, -1)
        self.p = rows[:, 0]
        self.g = np.stack(((rows[:, 1] - rows[:, 4])/(2*derivative_step_mm),
                           (rows[:, 2] - rows[:, 5])/(2*derivative_step_mm),
                           (rows[:, 3] - rows[:, 6])/(2*derivative_step_mm)), axis=1)

    def evaluate(self, phases: np.ndarray, calibration: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray]:
        q = np.exp(1j*phases)
        if calibration is not None:
            q = q*calibration
        pressure = self.p @ q
        gradient = np.einsum("pdi,i->pd", self.g, q)
        beta = self.physics.beta_mm2
        potential = np.abs(pressure)**2 - beta*np.sum(np.abs(gradient)**2, axis=1)
        zp = np.conj(pressure)[:, None]*self.p*q[None, :]
        zg = np.sum(np.conj(gradient)[:, :, None]*self.g, axis=1)*q[None, :]
        derivative = -2*np.imag(zp) + 2*beta*np.imag(zg)
        return potential, derivative


def local_derivatives(u: np.ndarray, du: np.ndarray, m: dict, h: float):
    grad = np.empty(3); dgrad = np.empty((3, du.shape[1]))
    hess = np.empty((3, 3)); dhess = np.empty((3, 3, du.shape[1]))
    c, dc = u[m[(0,)]], du[m[(0,)]]
    for a in range(3):
        up, um = u[m[(a, 1)]], u[m[(a, -1)]]
        dup, dum = du[m[(a, 1)]], du[m[(a, -1)]]
        grad[a] = (up-um)/(2*h); dgrad[a] = (dup-dum)/(2*h)
        hess[a, a] = (up-2*c+um)/(h*h)
        dhess[a, a] = (dup-2*dc+dum)/(h*h)
    for a in range(3):
        for b in range(a+1, 3):
            val = dval = 0
            for sa in (-1, 1):
                for sb in (-1, 1):
                    sign = sa*sb; idx = m[(a, b, sa, sb)]
                    val += sign*u[idx]; dval = dval + sign*du[idx]
            hess[a, b] = hess[b, a] = val/(4*h*h)
            dhess[a, b] = dhess[b, a] = dval/(4*h*h)
    return grad, dgrad, hess, dhess


def initial_drive(targets: np.ndarray, physics: Physics) -> np.ndarray:
    emitters, normals = emitter_geometry(physics)
    rows = transfer(targets, physics, emitters, normals)
    drives = np.exp(-1j*np.angle(rows))
    drives[:, :100] *= -1.0
    # Equal-power complex holographic superposition, normalized per emitter.
    return np.angle(np.sum(drives, axis=0))


def optimize(targets: np.ndarray, board_mm: float, iterations: int, seed: int,
             out: Path, h: float = 0.65) -> dict:
    physics = Physics(board_distance_mm=board_mm)
    points, maps = sample_stencil(targets, h)
    model = PotentialModel(physics, points)
    rng = np.random.default_rng(seed)
    phases = initial_drive(targets, physics) + rng.normal(0, 0.04, 200)
    # Optimise against nominal hardware plus fixed phase/amplitude-error worlds.
    # This turns calibration tolerance into part of the design, not just a test.
    calibrations = [np.ones(200, dtype=complex)]
    for _ in range(4):
        amplitude = np.maximum(rng.normal(1.0, 0.05, 200), 0.2)
        phase_error = rng.normal(0.0, np.deg2rad(5.0), 200)
        calibrations.append(amplitude*np.exp(1j*phase_error))
    u0, _ = model.evaluate(phases)
    scale = max(float(np.percentile(np.abs(u0), 70))/25.0, 1e-9)
    m_adam = np.zeros(200); v_adam = np.zeros(200)
    best = (-np.inf, phases.copy(), None)

    for step in range(1, iterations+1):
        eigs, eig_grads, centers, center_grads = [], [], [], []
        nominal_eigs = []
        for scenario, calibration in enumerate(calibrations):
            u, du = model.evaluate(phases, calibration)
            for mapping in maps:
                g, dg, H, dH = local_derivatives(u, du, mapping, h)
                values, vectors = np.linalg.eigh(H)
                v = vectors[:, 0]
                eigs.append(values[0])
                eig_grads.append(np.einsum("a,abk,b->k", v, dH, v))
                centers.append(g); center_grads.append(dg)
                if scenario == 0: nominal_eigs.append(values[0])
        eigs = np.asarray(eigs); eig_grads = np.asarray(eig_grads)
        centers = np.asarray(centers); center_grads = np.asarray(center_grads)

        # Smooth minimum makes the weaker trap dominate without a discontinuity.
        tau = 0.12*scale
        weights = np.exp(-(eigs-np.min(eigs))/tau); weights /= np.sum(weights)
        weak = -tau*(math.log(np.sum(np.exp(-(eigs-np.min(eigs))/tau))) - np.min(eigs)/tau)
        objective = weak/scale
        grad_obj = np.sum(weights[:, None]*eig_grads, axis=0)/scale

        # A finite-difference Hessian can look good while its minimum is displaced.
        # Weight 0.30; derive the mean explicitly so scenario count stays correct.
        force_penalty = 0.30*np.mean((centers/scale)**2)
        grad_penalty = (0.60/centers.size)*np.einsum("sa,sak->k",
            centers.reshape(-1,3)/scale, center_grads.reshape(-1,3,200)/scale)
        objective -= force_penalty
        grad_obj -= grad_penalty
        grad_obj -= np.mean(grad_obj)
        grad_obj = np.clip(grad_obj, -5.0, 5.0)

        if objective > best[0] and np.all(eigs > 0):
            # Score and phases must come from the same pre-update iterate.
            best = (float(objective), phases.copy(), eigs.copy())

        m_adam = .9*m_adam + .1*grad_obj
        v_adam = .999*v_adam + .001*grad_obj*grad_obj
        mh = m_adam/(1-.9**step); vh = v_adam/(1-.999**step)
        lr = .025*(.2 + .8*.5*(1+math.cos(math.pi*step/iterations)))
        phases += lr*mh/(np.sqrt(vh)+1e-8)
        phases = (phases+np.pi)%(2*np.pi)-np.pi
        if step == 1 or step % 50 == 0:
            print(f"step={step:4d} weak_stiffness={np.min(eigs):.6g} "
                  f"nominal={np.asarray(nominal_eigs)} "
                  f"worst_force_bias={np.max(np.linalg.norm(centers.reshape(-1,3), axis=1)):.5g}", flush=True)

    if best[2] is None:
        raise RuntimeError("optimizer did not find two positive-definite traps")
    ticks, quantized = quantize_phases(best[1], physics.phase_levels)
    uq, _ = model.evaluate(quantized)
    verification = []
    for mapping in maps:
        g, _, H, _ = local_derivatives(uq, np.zeros((len(uq), 1)), mapping, h)
        verification.append({"gradient": g.tolist(), "hessian": H.tolist(),
                             "eigenvalues": np.linalg.eigvalsh(H).tolist()})
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(out, ticks.astype(int), fmt="%d")
    report = {"method": "robust dual-trap maximin Gorkov eigenstiffness hologram",
              "targets_mm": targets.tolist(), "physics": asdict(physics),
              "stencil_mm": h, "phase_file": str(out),
              "design_uncertainty": {"scenarios": len(calibrations), "phase_sd_deg": 5.0,
                                     "amplitude_sd_fraction": 0.05},
              "quantized_verification": verification}
    out.with_suffix(".json").write_text(json.dumps(report, indent=2), encoding="ascii")
    return report


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--targets", default="-8,0,0;8,0,0")
    p.add_argument("--board-mm", type=float, default=135.0)
    p.add_argument("--iterations", type=int, default=1200)
    p.add_argument("--seed", type=int, default=47)
    p.add_argument("--output", type=Path, default=Path("cli/two_particle_phases.txt"))
    a = p.parse_args()
    targets = np.asarray([[float(x) for x in item.split(",")] for item in a.targets.split(";")])
    if targets.shape != (2, 3): raise SystemExit("--targets requires x,y,z;x,y,z")
    optimize(targets, a.board_mm, a.iterations, a.seed, a.output)


if __name__ == "__main__":
    main()
