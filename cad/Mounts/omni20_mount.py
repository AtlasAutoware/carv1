"""Cradle for the Omnicharge Omni 20+ on the clipless mounting system.

Omni 20+: 127 x 122 x 27 mm, 611 g (omnicharge.co). Its ports sit mid-edge on the long
sides, so the cradle holds the pack with four corner posts and leaves every edge open;
two hook-and-loop straps through floor slots keep it from lifting. Two pegs on the
underside drop into two neighbouring clipless pieces along the car's long axis.

    python3 omni20_mount.py            writes omni20_cradle.step/.stl and an assembly
    PEG_PITCH=50 python3 omni20_mount.py   for a different hole pair
"""
import os, sys
import cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from clipless import foot, foot_cutout, clipless_piece, plate_stub, PEG_DEPTH, RIM_PROUD, RIM_TOP, PLATE_T, FOOT_FLANGE_T

OMNI = (127.0, 122.0, 27.0)
CLR = 0.6                    # per side, so the pack drops in without prying
FLOOR = 4.0                  # spans 29 mm between the pegs under 611 g; no rib fits below
POST_H = 14.0                # above the floor; ports start higher than this
POST_LEG = 20.0
POST_T = 3.0
STRAP_W, STRAP_T = 25.0, 3.5 # 1 inch hook-and-loop
PEG_PITCH = float(os.environ.get('PEG_PITCH', 49.0))

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out')
os.makedirs(OUT, exist_ok=True)

L, W = OMNI[0] + 2 * CLR, OMNI[1] + 2 * CLR          # inside dimensions
OL, OW = L + 2 * POST_T, W + 2 * POST_T              # outside


def cradle():
    # floor, with big windows so it prints light and the pack's underside can breathe
    c = cq.Workplane('XY').rect(OL, OW).extrude(FLOOR).edges('|Z').fillet(4)
    # Windows must leave solid floor under each peg: a 24 mm bar at +-PEG_PITCH/2, a
    # 14 mm border all round, and open floor everywhere else.
    win_h = W - 2 * 14
    bar = 36.0                   # wide enough for the foot flange recess plus a wall
    spans = [(-L / 2 + 14, -PEG_PITCH / 2 - bar / 2),
             (-PEG_PITCH / 2 + bar / 2, PEG_PITCH / 2 - bar / 2),
             (PEG_PITCH / 2 + bar / 2, L / 2 - 14)]
    for x0, x1 in spans:
        if x1 - x0 < 8: continue
        c = c.cut(cq.Workplane('XY').center((x0 + x1) / 2, 0).rect(x1 - x0, win_h).extrude(FLOOR)
                  .edges('|Z').fillet(3))
    # corner posts: an L in each corner, outside the pack footprint
    for sx in (-1, 1):
        for sy in (-1, 1):
            leg_x = (cq.Workplane('XY').workplane(offset=FLOOR)
                     .center(sx * (L / 2 + POST_T / 2), sy * (W / 2 - POST_LEG / 2 + POST_T))
                     .rect(POST_T, POST_LEG).extrude(POST_H))
            leg_y = (cq.Workplane('XY').workplane(offset=FLOOR)
                     .center(sx * (L / 2 - POST_LEG / 2 + POST_T), sy * (W / 2 + POST_T / 2))
                     .rect(POST_LEG, POST_T).extrude(POST_H))
            c = c.union(leg_x).union(leg_y)
    # strap slots: two straps across the width, each through the floor at both long edges
    for sx in (-1, 1):
        for sy in (-1, 1):
            slot = (cq.Workplane('XY').center(sx * L / 4, sy * (W / 2 - 8))
                    .rect(STRAP_W + 1, STRAP_T + 1).extrude(FLOOR))
            c = c.cut(slot)
    # foot cutouts along x (the car's long axis); the feet are separate prints
    for sx in (-1, 1):
        c = c.cut(foot_cutout(FLOOR, 'x').translate((sx * PEG_PITCH / 2, 0, 0)))
    return c


def feet():
    return [foot('x').translate((sx * PEG_PITCH / 2, 0, FLOOR)) for sx in (-1, 1)]


def assembly(c):
    """Cradle seated on two clipless pieces in a stub of baseplate, plate top at z=0."""
    a = cq.Assembly(name='omni20_on_clipless')
    holes = [(-PEG_PITCH / 2, 0), (PEG_PITCH / 2, 0)]
    a.add(plate_stub(holes, size=(OL + 20, OW + 20)), name='baseplate', color=cq.Color(0.55, 0.55, 0.6))
    for i, (x, y) in enumerate(holes):
        a.add(clipless_piece().translate((x, y, -(RIM_TOP - RIM_PROUD))),
              name=f'clipless_{i}', color=cq.Color(0.63, 0.63, 0.63))
    a.add(c.translate((0, 0, RIM_PROUD)), name='cradle', color=cq.Color(0.9, 0.45, 0.1))
    for i, f in enumerate(feet()):
        a.add(f.translate((0, 0, RIM_PROUD)), name=f'foot_{i}', color=cq.Color(0.2, 0.5, 0.9))
    omni = (cq.Workplane('XY').rect(*OMNI[:2]).extrude(OMNI[2]).edges('|Z').fillet(8)
            .translate((0, 0, RIM_PROUD + FLOOR)))
    a.add(omni, name='omni20', color=cq.Color(0.15, 0.15, 0.15))
    return a


if __name__ == '__main__':
    c = cradle()
    bb = c.val().BoundingBox()
    N = 'omni20_cradle' + os.environ.get('OUT_SUFFIX', '')
    cq.exporters.export(c, os.path.join(OUT, N + '.step'))
    cq.exporters.export(c, os.path.join(OUT, N + '.stl'), tolerance=0.02, angularTolerance=0.1)
    f = foot('x')
    cq.exporters.export(f, os.path.join(OUT, 'clipless_foot.step'))
    cq.exporters.export(f, os.path.join(OUT, 'clipless_foot.stl'), tolerance=0.02, angularTolerance=0.1)
    a = assembly(c)
    a.save(os.path.join(OUT, N.replace('cradle', 'assembly') + '.step'))
    vol = c.val().Volume()
    print(f'cradle {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm, {vol/1000:.1f} cm3 '
          f'(~{vol/1000*1.24*0.6:.0f} g PLA at 60% effective), peg pitch {PEG_PITCH} mm')
