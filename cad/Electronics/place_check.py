"""Real placement checking: courtyard overlaps and board-edge violations, from the actual
footprint geometry rather than from an assumed part size.

The first floorplan looked fine as a table of coordinates and was wrong in six places once
the footprints were real: terminal blocks hanging over the edge, two M3 holes underneath
connectors, and the 0805 rows in each rail overlapping each other. Coordinates are cheap to
write and expensive to eyeball, so they get checked.
"""
import os, sys, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_pcb as G, layout as LO, netlist as N


def courtyard(fpnode, px, py, prot):
    """(x0, y0, x1, y1) of the footprint's courtyard in board coords; falls back to pads."""
    xs, ys = [], []
    for key in ('fp_rect', 'fp_line', 'fp_poly', 'fp_circle'):
        for g in G.getall(fpnode, key):
            lay = G.get(g, 'layer')
            if not lay or 'CrtYd' not in lay[1]:
                continue
            for sub in ('start', 'end', 'center', 'mid'):
                p = G.get(g, sub)
                if p:
                    x, y = G.rot(float(p[1]), float(p[2]), prot)
                    xs.append(px + x); ys.append(py + y)
            pts = G.get(g, 'pts')
            if pts:
                for xy in G.getall(pts, 'xy'):
                    x, y = G.rot(float(xy[1]), float(xy[2]), prot)
                    xs.append(px + x); ys.append(py + y)
    if not xs:
        for pad in G.getall(fpnode, 'pad'):
            at = G.get(pad, 'at'); size = G.get(pad, 'size')
            x, y = G.rot(float(at[1]), float(at[2]), prot)
            w, h = float(size[1]) / 2, float(size[2]) / 2
            xs += [px + x - w, px + x + w]; ys += [py + y - h, py + y + h]
    return min(xs), min(ys), max(xs), max(ys)


def overlap(a, b, tol=0.0):
    return not (a[2] <= b[0] + tol or b[2] <= a[0] + tol or
                a[3] <= b[1] + tol or b[3] <= a[1] + tol)


def run():
    parts, _ = N.build()
    places = LO.placement()
    boxes = {}
    for p in parts:
        px, py, prot = places[p.ref]
        boxes[p.ref] = courtyard(G.find_fp(p.footprint), px, py, prot)
    hw, hh = LO.BOARD_W / 2, LO.BOARD_H / 2
    problems = []
    for ref, b in sorted(boxes.items()):
        if b[0] < -hw or b[2] > hw or b[1] < -hh or b[3] > hh:
            over = max(-hw - b[0], b[2] - hw, -hh - b[1], b[3] - hh)
            problems.append(('EDGE', ref, f'extends {over:.2f} mm past the board edge '
                                          f'(box {b[0]:.1f},{b[1]:.1f} .. {b[2]:.1f},{b[3]:.1f})'))
    refs = sorted(boxes)
    for i, ra in enumerate(refs):
        for rb in refs[i + 1:]:
            if overlap(boxes[ra], boxes[rb], tol=0.01):
                a, b = boxes[ra], boxes[rb]
                ox = min(a[2], b[2]) - max(a[0], b[0])
                oy = min(a[3], b[3]) - max(a[1], b[1])
                problems.append(('OVERLAP', f'{ra}/{rb}', f'{ox:.2f} x {oy:.2f} mm'))
    return boxes, problems


if __name__ == '__main__':
    boxes, problems = run()
    edge = [p for p in problems if p[0] == 'EDGE']
    ov = [p for p in problems if p[0] == 'OVERLAP']
    print(f'{len(boxes)} parts: {len(edge)} over the edge, {len(ov)} courtyard overlaps')
    for k, r, m in edge: print(f'  EDGE     {r:8s} {m}')
    for k, r, m in ov: print(f'  OVERLAP  {r:16s} {m}')
    raise SystemExit(0 if not problems else 1)
