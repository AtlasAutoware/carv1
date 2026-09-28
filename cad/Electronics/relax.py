"""Take the hand floorplan as a seed and separate it until nothing overlaps.

The hand-written coordinates in layout.py encode the intent: which part belongs next to
which, and which side of the regulator the quiet nodes live on. What they got wrong was
spacing, because a 1210 rotated 90 degrees has a 4.6 mm courtyard and I wrote 3.4, and
because a terminal block's origin is pin 1 rather than its centre.

So: keep the intent, fix the spacing numerically. Parts are pushed apart along whichever
axis they overlap least (which preserves the row/column structure instead of scattering
them), pulled back inside the board edge, and pushed off the mounting holes. Anything in
FROZEN does not move - the regulators, inductors and mounting holes define the plan and
everything else arranges itself around them.

    python3 relax.py            prints the solved placement and writes layout_solved.py
"""
import os, sys, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_pcb as G, layout as LO, netlist as N, place_check as PC

FROZEN = {'H1', 'H2', 'H3', 'H4', 'U3', 'U4', 'U5', 'L1', 'L2', 'L3'}
GAP = 0.15            # extra separation beyond touching courtyards
EDGE_GAP = 0.5        # courtyard to board edge
HOLE_R = 3.6          # keep-out radius around an M3 hole centre


def solve(iters=600):
    parts, _ = N.build()
    places = {k: list(v) for k, v in LO.placement().items()}
    fps = {p.ref: G.find_fp(p.footprint) for p in parts}
    hw, hh = LO.BOARD_W / 2 - EDGE_GAP, LO.BOARD_H / 2 - EDGE_GAP
    holes = [(places[h][0], places[h][1]) for h in ('H1', 'H2', 'H3', 'H4')]

    def box(ref):
        x, y, r = places[ref]
        return PC.courtyard(fps[ref], x, y, r)

    refs = [p.ref for p in parts]
    for _ in range(iters):
        boxes = {r: box(r) for r in refs}
        moved = False
        for i, ra in enumerate(refs):
            for rb in refs[i + 1:]:
                a, b = boxes[ra], boxes[rb]
                if PC.overlap(a, b, tol=-GAP):
                    ox = min(a[2], b[2]) - max(a[0], b[0]) + GAP
                    oy = min(a[3], b[3]) - max(a[1], b[1]) + GAP
                    ca = ((a[0] + a[2]) / 2, (a[1] + a[3]) / 2)
                    cb = ((b[0] + b[2]) / 2, (b[1] + b[3]) / 2)
                    if ox < oy:
                        d, ax = ox, 0
                        s = 1 if cb[0] >= ca[0] else -1
                    else:
                        d, ax = oy, 1
                        s = 1 if cb[1] >= ca[1] else -1
                    fa = 0.0 if ra in FROZEN else (0.5 if rb not in FROZEN else 1.0)
                    fb = 0.0 if rb in FROZEN else (0.5 if ra not in FROZEN else 1.0)
                    places[ra][ax] -= s * d * fa
                    places[rb][ax] += s * d * fb
                    boxes[ra], boxes[rb] = box(ra), box(rb)
                    moved = True
        for r in refs:
            if r in FROZEN:
                continue
            a = box(r)
            dx = dy = 0.0
            if a[0] < -hw: dx = -hw - a[0]
            if a[2] > hw:  dx = hw - a[2]
            if a[1] < -hh: dy = -hh - a[1]
            if a[3] > hh:  dy = hh - a[3]
            if dx or dy:
                places[r][0] += dx; places[r][1] += dy; moved = True
            a = box(r)
            cx, cy = (a[0] + a[2]) / 2, (a[1] + a[3]) / 2
            for hx, hy in holes:
                if a[0] - HOLE_R < hx < a[2] + HOLE_R and a[1] - HOLE_R < hy < a[3] + HOLE_R:
                    vx, vy = cx - hx, cy - hy
                    n = max((vx * vx + vy * vy) ** 0.5, 1e-6)
                    places[r][0] += vx / n * 0.6; places[r][1] += vy / n * 0.6
                    moved = True
        if not moved:
            break
    return places


if __name__ == '__main__':
    places = solve()
    seed = LO.placement(solved=False)
    moves = sorted(((max(abs(places[r][0] - seed[r][0]), abs(places[r][1] - seed[r][1])), r)
                    for r in places), reverse=True)
    print('largest moves from the hand floorplan:')
    for d, r in moves[:10]:
        print(f'  {r:8s} {d:5.2f} mm  ({seed[r][0]:+.1f},{seed[r][1]:+.1f}) -> '
              f'({places[r][0]:+.1f},{places[r][1]:+.1f})')
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'layout_solved.json'), 'w') as f:
        json.dump({k: [round(v[0], 3), round(v[1], 3), v[2]] for k, v in places.items()}, f, indent=1)
    print('\nwrote layout_solved.json')
