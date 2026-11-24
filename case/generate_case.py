#!/usr/bin/env python3
"""
ind3236 3D-printed bottom case generator (left + right).

Reads the real PCB data from ../ind-assemble_{L,R}/ind-assemble_{L,R}.kicad_pcb
(board outline, corner radius, Pro Micro and TRRS positions) and writes
STL (for slicing) and STEP (for editing in Fusion 360 etc.) next to this script.

    pip install cadquery
    python3 case/generate_case.py

Coordinate system of the output: top view equals the KiCad front-side view,
X = KiCad X, Y = -KiCad Y (the USB / back edge of the board is the +Y wall),
Z = up, floor underside at Z = 0. The case is centred on the origin.
"""
import math
import os
import re

import cadquery as cq

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# ── Parameters (mm) ───────────────────────────────────────────────────
CLEARANCE = 0.7      # gap between PCB edge and inner wall, per side
WALL_T = 3.0         # wall thickness
BOT_T = 3.0          # floor thickness
PCB_LIFT = 4.0       # height of the PCB underside above the floor (your spacers)
WALL_ABOVE = 13.5    # wall height above the PCB underside reference
PCB_T = 1.6
RIM_CHAMFER = 0.8

WALL_H = PCB_LIFT + WALL_ABOVE          # inner wall height above floor
TOTAL_H = BOT_T + WALL_H                # 20.5 mm
Z_PCB_TOP = BOT_T + PCB_LIFT + PCB_T    # 8.6 mm

# Pro Micro / USB-C: open-top notch in the back wall, as wide as the Pro Micro
PM_W = 17.78                 # Pro Micro board width (footprint F.Fab)
PM_CENTER_DX = -1.27         # footprint origin -> board centre, X
NOTCH_W = PM_W + 0.4
USB_ABOVE_PCB = 7.0          # measured: USB-C centre 7 mm above PCB top face
NOTCH_BOTTOM_BELOW_USB = 4.0  # notch floor = USB centre - 4 mm

# TRRS jack (mounted on the top face): round hole in the side wall
TRRS_ABOVE_PCB = 2.0         # measured: jack centre 2 mm above PCB top face
TRRS_D = 11.0


# ── KiCad parsing ─────────────────────────────────────────────────────
def _circumradius(p1, p2, p3):
    a, b, c = math.dist(p1, p2), math.dist(p2, p3), math.dist(p3, p1)
    (x1, y1), (x2, y2), (x3, y3) = p1, p2, p3
    area = abs((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2
    return a * b * c / (4 * area)


def _footprint_at(text, lib_name):
    i = text.index(f'(footprint "{lib_name}"')
    m = re.search(r"\n\s*\(at ([-\d.]+) ([-\d.]+)", text[i:i + 2000])
    return float(m.group(1)), float(m.group(2))


def read_pcb(side):
    path = os.path.join(ROOT, f"ind-assemble_{side}", f"ind-assemble_{side}.kicad_pcb")
    t = open(path, encoding="utf-8").read()
    pts = []
    for m in re.finditer(r'\((gr_line|gr_arc)(.*?)\(layer "Edge.Cuts"\)', t, re.S):
        for p in re.finditer(r"\((?:start|mid|end) ([-\d.]+) ([-\d.]+)\)", m.group(2)):
            pts.append((float(p.group(1)), float(p.group(2))))
    radii = []
    for m in re.finditer(r"\(gr_arc \(start ([-\d.]+) ([-\d.]+)\) \(mid ([-\d.]+) ([-\d.]+)\) "
                         r"\(end ([-\d.]+) ([-\d.]+)\)", t):
        v = [float(g) for g in m.groups()]
        radii.append(_circumradius(v[0:2], v[2:4], v[4:6]))
    return dict(
        xmin=min(p[0] for p in pts), xmax=max(p[0] for p in pts),
        ymin=min(p[1] for p in pts), ymax=max(p[1] for p in pts),
        r=sum(radii) / len(radii),
        pm=_footprint_at(t, "kbd_Parts:Micon_ProMicro"),
        trrs=_footprint_at(t, "kbd_Parts:TRRS_MJ-4PP-9"),
    )


# ── Geometry ──────────────────────────────────────────────────────────
def build(P):
    W, H = P["xmax"] - P["xmin"], P["ymax"] - P["ymin"]
    cav_w, cav_h, cav_r = W + 2 * CLEARANCE, H + 2 * CLEARANCE, P["r"] + CLEARANCE
    out_w, out_h, out_r = cav_w + 2 * WALL_T, cav_h + 2 * WALL_T, cav_r + WALL_T

    def to_case(x, y):  # KiCad absolute -> case-centred, front-side-up top view
        return (x - P["xmin"]) - W / 2, H / 2 - (y - P["ymin"])

    body = (cq.Workplane("XY").sketch().rect(out_w, out_h).vertices().fillet(out_r)
            .finalize().extrude(TOTAL_H))
    cavity = (cq.Workplane("XY").workplane(offset=BOT_T).sketch().rect(cav_w, cav_h)
              .vertices().fillet(cav_r).finalize().extrude(WALL_H + 0.1))
    body = body.cut(cavity)
    body = body.faces(">Z").chamfer(RIM_CHAMFER)

    # USB-C / Pro Micro notch (back wall, open to the top)
    ux, _ = to_case(P["pm"][0] + PM_CENTER_DX, P["ymin"])
    nz0 = Z_PCB_TOP + USB_ABOVE_PCB - NOTCH_BOTTOM_BELOW_USB
    body = body.cut(cq.Workplane("XY").workplane(offset=nz0)
                    .center(ux, cav_h / 2 + WALL_T / 2)
                    .rect(NOTCH_W, WALL_T + 4).extrude(TOTAL_H - nz0 + 1))

    # TRRS hole (side wall nearest to the jack)
    tx, ty = to_case(*P["trrs"])
    tz = Z_PCB_TOP + TRRS_ABOVE_PCB
    left = tx < 0
    x0 = -(out_w / 2 + 1) if left else (out_w / 2 + 1)
    d = WALL_T + 4
    body = body.cut(cq.Workplane("YZ").workplane(offset=x0).center(ty, tz)
                    .circle(TRRS_D / 2).extrude(d if left else -d))
    return body, (out_w, out_h, TOTAL_H)


def main():
    for side, name in (("L", "left"), ("R", "right")):
        shape, size = build(read_pcb(side))
        for ext in ("stl", "step"):
            out = os.path.join(HERE, f"ind3236_case_{side}.{ext}")
            cq.exporters.export(shape, out)
        print(f"{name}: {size[0]:.1f} x {size[1]:.1f} x {size[2]:.1f} mm -> ind3236_case_{side}.stl/.step")


if __name__ == "__main__":
    main()
