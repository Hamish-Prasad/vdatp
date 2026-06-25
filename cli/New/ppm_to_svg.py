#!/usr/bin/env python3
"""Convert ASCII PPM validator heatmaps to compact SVG previews."""

from __future__ import annotations

import sys
from pathlib import Path


def read_ppm(path: Path):
    data = path.read_text(encoding="ascii").split()
    if data[0] != "P3":
        raise ValueError("expected ASCII PPM/P3")
    w, h, maxv = map(int, data[1:4])
    nums = list(map(int, data[4:]))
    pix = [tuple(nums[i:i + 3]) for i in range(0, len(nums), 3)]
    if len(pix) != w * h or maxv != 255:
        raise ValueError("unexpected PPM shape")
    return w, h, pix


def write_svg(ppm: Path, svg: Path, step: int = 3) -> None:
    w, h, pix = read_ppm(ppm)
    cell = 3
    out_w = ((w + step - 1) // step) * cell
    out_h = ((h + step - 1) // step) * cell
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{out_w}" height="{out_h}" viewBox="0 0 {out_w} {out_h}">',
        '<rect width="100%" height="100%" fill="black"/>',
    ]
    for y in range(0, h, step):
        for x in range(0, w, step):
            r, g, b = pix[y * w + x]
            lines.append(
                f'<rect x="{(x // step) * cell}" y="{(y // step) * cell}" '
                f'width="{cell}" height="{cell}" fill="rgb({r},{g},{b})"/>'
            )
    lines.append("</svg>")
    svg.write_text("\n".join(lines), encoding="ascii")


def main() -> None:
    if len(sys.argv) < 2:
        print("usage: ppm_to_svg.py <file.ppm> [...]")
        raise SystemExit(1)
    for name in sys.argv[1:]:
        ppm = Path(name)
        svg = ppm.with_suffix(".svg")
        write_svg(ppm, svg)
        print(svg)


if __name__ == "__main__":
    main()
