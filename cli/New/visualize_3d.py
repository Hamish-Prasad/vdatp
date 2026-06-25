#!/usr/bin/env python3
"""Generate self-contained 3D HTML trap visualisations with no external libs."""

from __future__ import annotations

import html
import json
from pathlib import Path

import quick_validate as qv


TRAPS = [(-12.0, 0.0, 0.0), (0.0, 0.0, 0.0), (12.0, 0.0, 0.0)]
CASES = [
    ("replicate-wgs", "replicate_wgs"),
    ("shadow-nullspace", "shadow_nullspace"),
    ("curvature-boost", "curvature_boost"),
]


def volume_points(phases: list[int]):
    rows = []
    values = []
    for x in range(-24, 25, 3):
        for y in range(-12, 13, 3):
            for z in range(-18, 19, 3):
                g = qv.sample(phases, (float(x), float(y), float(z)))[2]
                values.append(g)
                rows.append([x, y, z, g])
    values_sorted = sorted(values)
    cutoff = values_sorted[max(0, int(0.18 * len(values_sorted)) - 1)]
    selected = [r for r in rows if r[3] <= cutoff]
    mn = min(r[3] for r in selected)
    mx = max(r[3] for r in selected)
    pts = []
    for x, y, z, g in selected:
        t = (g - mn) / (mx - mn + 1e-30)
        pts.append({"x": x, "y": y, "z": z, "v": t})
    return pts, cutoff


def write_html(path: Path, method: str, pts, phases, cutoff: float) -> None:
    metrics = []
    for i, trap in enumerate(TRAPS):
        pressure2, grad2, g = qv.sample(phases, trap)
        metrics.append({
            "trap": i,
            "x": trap[0],
            "y": trap[1],
            "z": trap[2],
            "pressure2": pressure2,
            "grad2": grad2,
            "gorkov": g,
            "curvature": qv.score(phases, trap),
        })
    doc = f"""<!doctype html>
<html lang="en">
<meta charset="utf-8">
<title>{html.escape(method)} 3D trap visualisation</title>
<style>
body {{ margin:0; font-family:Segoe UI, Arial, sans-serif; background:#101315; color:#e8ecef; }}
#wrap {{ display:grid; grid-template-columns:1fr 360px; min-height:100vh; }}
canvas {{ width:100%; height:100vh; display:block; background:#11161a; }}
aside {{ padding:18px; border-left:1px solid #28313a; background:#171d22; }}
h1 {{ font-size:20px; margin:0 0 12px; }}
table {{ border-collapse:collapse; width:100%; font-size:13px; }}
td, th {{ border-bottom:1px solid #2b343c; padding:6px; text-align:right; }}
th:first-child, td:first-child {{ text-align:left; }}
.hint {{ color:#a9b4bd; font-size:13px; line-height:1.4; }}
</style>
<div id="wrap">
<canvas id="c"></canvas>
<aside>
<h1>{html.escape(method)}</h1>
<p class="hint">Drag to rotate. Blue/green points are the lowest 18% of the sampled Gor'kov-like volume; gold points are requested particle centers.</p>
<p class="hint">Cutoff: {cutoff:.6g}</p>
<table><tr><th>trap</th><th>curv.</th><th>U</th></tr>
{''.join(f'<tr><td>{m["trap"]}</td><td>{m["curvature"]:.4g}</td><td>{m["gorkov"]:.4g}</td></tr>' for m in metrics)}
</table>
</aside>
</div>
<script>
const points = {json.dumps(pts)};
const traps = {json.dumps([{"x": x, "y": y, "z": z} for x, y, z in TRAPS])};
const canvas = document.getElementById('c');
const ctx = canvas.getContext('2d');
let ax = -0.45, ay = 0.65, dragging = false, lx = 0, ly = 0;
function resize() {{ canvas.width = canvas.clientWidth * devicePixelRatio; canvas.height = canvas.clientHeight * devicePixelRatio; }}
addEventListener('resize', resize); resize();
canvas.onpointerdown = e => {{ dragging = true; lx = e.clientX; ly = e.clientY; canvas.setPointerCapture(e.pointerId); }};
canvas.onpointerup = () => dragging = false;
canvas.onpointermove = e => {{ if (!dragging) return; ay += (e.clientX-lx)*0.008; ax += (e.clientY-ly)*0.008; lx=e.clientX; ly=e.clientY; draw(); }};
function project(p) {{
  let x=p.x, y=p.y, z=p.z;
  let cy=Math.cos(ay), sy=Math.sin(ay), cx=Math.cos(ax), sx=Math.sin(ax);
  let x1=x*cy+z*sy, z1=-x*sy+z*cy;
  let y1=y*cx-z1*sx, z2=y*sx+z1*cx;
  let scale = canvas.height / 95;
  return {{x: canvas.width/2 + x1*scale, y: canvas.height/2 - y1*scale, z:z2}};
}}
function draw() {{
  ctx.clearRect(0,0,canvas.width,canvas.height);
  ctx.fillStyle='#11161a'; ctx.fillRect(0,0,canvas.width,canvas.height);
  let all = points.map(p => [project(p), p.v, 2.2]).concat(traps.map(t => [project(t), -1, 7]));
  all.sort((a,b)=>a[0].z-b[0].z);
  for (const [p,v,r0] of all) {{
    if (v < 0) {{ ctx.fillStyle='#ffca4f'; }}
    else {{
      let b=Math.floor(220-100*v), g=Math.floor(95+130*(1-v)), r=Math.floor(35+60*v);
      ctx.fillStyle=`rgb(${{r}},${{g}},${{b}})`;
    }}
    let r = r0 * devicePixelRatio * (1.05 + (p.z+45)/180);
    ctx.beginPath(); ctx.arc(p.x,p.y,r,0,Math.PI*2); ctx.fill();
  }}
}}
draw();
</script>
</html>
"""
    path.write_text(doc, encoding="utf-8")
    with path.with_suffix(".3d_metrics.json").open("w", encoding="ascii") as f:
        json.dump({"method": method, "cutoff": cutoff, "metrics": metrics}, f, indent=2)


def main() -> None:
    for method, tag in CASES:
        phases = qv.make_phases(method, TRAPS)
        pts, cutoff = volume_points(phases)
        write_html(Path(f"{tag}_3d.html"), method, pts, phases, cutoff)
        print(f"{tag}_3d.html points={len(pts)} cutoff={cutoff:.6g}")


if __name__ == "__main__":
    main()
