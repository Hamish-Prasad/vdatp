#!/usr/bin/env python3
"""Independent local, quantisation, calibration-error and crosstalk checks."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from Vector.vector_optimizer import Physics, emitter_geometry, transfer


def potential(points, drive, physics, amplitude=None, d=0.12):
    emitters, normals = emitter_geometry(physics)
    offsets = np.vstack((np.zeros((1, 3)), np.eye(3), -np.eye(3)))*d
    rows = transfer((points[:, None, :]+offsets[None, :, :]).reshape(-1, 3),
                    physics, emitters, normals).reshape(len(points), 7, -1)
    q = drive if amplitude is None else drive*amplitude
    p = rows[:, 0]@q
    g = np.stack(((rows[:, 1]-rows[:, 4])@q, (rows[:, 2]-rows[:, 5])@q,
                  (rows[:, 3]-rows[:, 6])@q), axis=1)/(2*d)
    return np.abs(p)**2-physics.beta_mm2*np.sum(np.abs(g)**2, axis=1)


def derivatives(target, drive, physics, amplitude=None, h=.3):
    points = [target]
    for a in range(3):
        for s in (-1, 1):
            p=target.copy(); p[a]+=s*h; points.append(p)
    for a in range(3):
        for b in range(a+1,3):
            for sa in (-1,1):
                for sb in (-1,1):
                    p=target.copy(); p[a]+=sa*h; p[b]+=sb*h; points.append(p)
    u=potential(np.asarray(points),drive,physics,amplitude); c=u[0]; idx=1
    grad=np.zeros(3); H=np.zeros((3,3)); axial={}
    for a in range(3):
        um,up=u[idx],u[idx+1]; idx+=2
        grad[a]=(up-um)/(2*h); H[a,a]=(up-2*c+um)/(h*h)
        axial[a]=(um-c,up-c)
    for a in range(3):
        for b in range(a+1,3):
            vals=u[idx:idx+4]; idx+=4
            H[a,b]=H[b,a]=(vals[0]-vals[1]-vals[2]+vals[3])/(4*h*h)
    return grad,H,axial


def main():
    p=argparse.ArgumentParser(); p.add_argument("--phases",type=Path,default=Path("cli/two_particle_phases.txt"))
    p.add_argument("--trials",type=int,default=250); p.add_argument("--output",type=Path,default=Path("cli/two_particle_verification.json")); a=p.parse_args()
    ticks=np.loadtxt(a.phases,dtype=int)
    if ticks.shape != (200,) or np.any(ticks < 0) or np.any(ticks >= 512):
        raise SystemExit("phase file must contain exactly 200 integers in 0..511")
    drive=np.exp(1j*ticks*2*np.pi/512)
    metadata_path=a.phases.with_suffix(".json")
    if not metadata_path.exists():
        raise SystemExit(f"missing optimizer metadata: {metadata_path}")
    metadata=json.loads(metadata_path.read_text(encoding="ascii"))
    physics=Physics(**metadata["physics"])
    targets=np.asarray(metadata["targets_mm"],dtype=float)
    if targets.shape != (2,3): raise SystemExit("metadata must contain exactly two 3D targets")
    report={"phase_file":str(a.phases),"targets_mm":targets.tolist(),"stencils":{},"equilibria":[],"robustness":{}}
    for h in (.2,.3,.45,.7):
        report["stencils"][str(h)]=[]
        for t in targets:
            g,H,ax=derivatives(t,drive,physics,h=h)
            report["stencils"][str(h)].append({"gradient":g.tolist(),"eigenvalues":np.linalg.eigvalsh(H).tolist(),"axis_well_margins":ax})
    # Newton-refine the actual zero-force locations near the requested points.
    for target in targets:
        point=target.copy()
        for _ in range(8):
            g,H,_=derivatives(point,drive,physics,h=.25)
            step=np.linalg.solve(H,g)
            if np.linalg.norm(step)>1.0: raise SystemExit("local equilibrium search left its trusted basin")
            point-=step
            if np.linalg.norm(step)<1e-7: break
        g,H,_=derivatives(point,drive,physics,h=.2)
        eig=np.linalg.eigvalsh(H)
        if np.min(eig)<=0 or np.linalg.norm(point-target)>1.0:
            raise SystemExit("no nearby positive-definite equilibrium")
        report["equilibria"].append({"position_mm":point.tolist(),"offset_mm":(point-target).tolist(),
                                      "residual_gradient":g.tolist(),"eigenvalues":eig.tolist()})
    rng=np.random.default_rng(903)
    for phase_sd_deg,amp_sd in ((2,0.03),(5,0.05),(10,0.10),(20,0.15)):
        minima=[]; stable=0
        for _ in range(a.trials):
            noisy=drive*np.exp(1j*rng.normal(0,np.deg2rad(phase_sd_deg),200))
            amplitude=np.maximum(rng.normal(1,amp_sd,200),0)
            trial=[]
            for t in targets:
                _,H,_=derivatives(t,noisy,physics,amplitude,.3); trial.append(float(np.min(np.linalg.eigvalsh(H))))
            minima.append(min(trial)); stable += min(trial)>0
        report["robustness"][f"phase{phase_sd_deg}deg_amp{amp_sd:.2f}"]={"stable_fraction":stable/a.trials,"minimum_stiffness_p05":float(np.percentile(minima,5)),"median":float(np.median(minima))}
    # Verify that the midpoint is not an accidentally competitive third well.
    centers=potential(np.vstack((targets,[[0.,0,0]])),drive,physics)
    report["center_potentials"]={"left":float(centers[0]),"right":float(centers[1]),"midpoint":float(centers[2])}
    # Conditional dimensional estimate using the Physics reference pressure.
    emitters,normals=emitter_geometry(physics)
    center_row=transfer(np.zeros((1,3)),physics,emitters,normals)[0]
    pressure_scale=physics.reference_focus_rms_pa/np.sum(np.abs(center_row))
    radius_m=0.5e-3*physics.particle_diameter_mm
    volume=4*np.pi*radius_m**3/3
    force_scale=volume*physics.f1/(2*physics.air_density_kg_m3*physics.sound_speed_m_s**2)*pressure_scale**2*1000
    weight=volume*physics.particle_density_kg_m3*9.80665
    weakest=min(min(item["eigenvalues"]) for item in report["equilibria"])
    report["conditional_physical_estimate"]={"reference_focus_rms_pa":physics.reference_focus_rms_pa,
        "weakest_stiffness_n_per_m":force_scale*weakest*1000,
        "particle_weight_n":weight,"linear_gravity_sag_mm":weight/(force_scale*weakest)}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(report,indent=2),encoding="ascii")
    print(json.dumps(report,indent=2))

if __name__=="__main__": main()
