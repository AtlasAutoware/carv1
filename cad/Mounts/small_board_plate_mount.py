"""Generic 100 x 60 plate for the small boards on the car, on the clipless mounting system.

- 2.6 mm holes on a 5 mm grid, so M2.5 standoffs go anywhere.
- The Adafruit PCA9685 16-channel PWM breakout (product 815) on four 3.5 mm heat-set
  insert bosses at its exact hole pattern: 55.88 x 19.05 mm, 2.5 mm holes, on a
  62.23 x 25.40 mm board (EagleCAD "Adafruit PCA9685 rev C.brd" in
  github.com/adafruit/Adafruit-16-Channel-PWM-Servo-Driver-PCB; board size also on
  adafruit.com/product/815 as 62.5 x 25.4 x 3 mm).
- Two 26 x 4.5 strap slots for a 25 mm hook-and-loop strap over a 4-port USB hub. The hub
  is not identified, HUB below is an estimate.
- Two clipless feet along the car. Their 30.5 x 34.5 flange recesses take most of the
  plate's middle band, so the plate is shifted PLATE_OFFSET along x relative to the foot
  pair (one PCA9685 boss column in the strip between the feet, one beyond the +x foot)
  and the strap slots sit in the bands above and below the feet.

    python3 small_board_plate_mount.py
    PEG_PITCH=50 python3 small_board_plate_mount.py
"""
import os, sys
import cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from clipless import foot, foot_cutout, clipless_piece, plate_stub, RIM_PROUD, RIM_TOP, FOOT_FLANGE, FOOT_CLR
from render import render
from jetson_orin_nano_mount import lines_png, box, inter

# ---- components -----------------------------------------------------------------------
PCA_BOARD = (62.23, 25.40, 1.6)     # rev C outline, x -1.905..60.325, y -6.477..18.923
PCA_HOLES_DX, PCA_HOLES_DY = 55.88, 19.05
PCA_HOLE_D = 2.5
PCA_TOP_H = 12.0                    # ESTIMATE: 3x4 servo headers and terminal block
PCA_UNDER = 2.5                     # ESTIMATE: solder tails below the board
HUB = (85.0, 26.0, 12.0)            # ESTIMATE: 4-port USB hub, not identified

# ---- plate ----------------------------------------------------------------------------
PLATE = (100.0, 60.0)
FLOOR = 4.0
GRID_PITCH, GRID_D = 5.0, 2.6
BOSS_OD, BOSS_H = 9.5, 6.0          # 3.5 hole + 3 mm wall, 6 mm standoff
INSERT_D, INSERT_DEPTH = 3.5, 5.0
STRAP_W, STRAP_T = 26.0, 4.5
PEG_PITCH = float(os.environ.get('PEG_PITCH', 49.0))
# The solid strip between the two foot recesses is PEG_PITCH - 34.5 wide (14.5 at 49,
# 5.5 at 40). One PCA9685 boss column (9.5 OD) has to stand in it without overhanging a
# recess, or the foot cannot drop in. At 49 the column sits at x -3.30 (PCA_X 24.64); at
# 40 the strip is narrower than the boss, so the column goes to x 0 exactly between the
# recesses (PCA_X 27.94), which pushes the far column to 55.88 and the plate to
# PLATE_OFFSET 10.75 to keep it on the plate.
_NARROW = PEG_PITCH - (FOOT_FLANGE[0] + 2 * FOOT_CLR) < 9.5 + 2.0
PLATE_OFFSET = float(os.environ.get('PLATE_OFFSET', 10.75 if _NARROW else 8.5))   # plate centre relative to the foot-pair centre, +x
                                    # (8.5 .. 10.75 keeps a 4 mm rim past the foot recesses at 40 mm pitch)
PCA_X = float(os.environ.get('PCA_X', PCA_HOLES_DX / 2 if _NARROW else 24.64))

# RETAIN='snap': under-board variant (snap_retain.py). The floor thickens to SNAP_FLOOR
# and the two flange recesses become the dovetail slide channel + snap tongues of
# rplidar_c1_mount.py along x through the plate's middle band (open at -x). Grid holes stay
# out of the 40.5 mm band and the +y strap slot moves just outside it. SNAP_FEET=1 keeps a
# single foot at the pair centre (the whole plate carries well under 100 g): stop ahead of
# its flange, tongues behind it, same retention, 1 degree of rotational play in the pocket.
RETAIN = os.environ.get('RETAIN', 'recess')
SNAP_FEET = int(os.environ.get('SNAP_FEET', 2))
SNAP_FLOOR = 7.0 + 1.6
SUFFIX = ('_under' if RETAIN == 'snap' else '') + os.environ.get('OUT_SUFFIX', '')
if RETAIN == 'snap':
    FLOOR = SNAP_FLOOR
# Solid floor (outside the two 30.5 x 34.5 foot recesses at x +-24.5, |y| < 17.25) is the
# centre strip |x| < 9.25, the +x end x > 39.75, and the two bands |y| > 17.25. The
# PCA9685 boss columns are 55.88 apart, so one goes in the centre strip and one at the
# +x end; the hub strap slots go in the two bands.
PCA_C = (PCA_X, 15.1)               # PCA9685 centre; boss columns at PCA_X -+ 27.94
HUB_C = (PLATE_OFFSET, -14.5)       # hub centre, along the -y half of the plate
SLOT_X = -25.0                      # strap slots at y -24.75 (outside the hub) and +21.5
                                    # (the strap runs over the plate past the hub's +y edge)

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out')
os.makedirs(OUT, exist_ok=True)

PX, PY = PLATE
FEET = [(-PEG_PITCH / 2, 0.0), (PEG_PITCH / 2, 0.0)]
if RETAIN == 'snap' and SNAP_FEET == 1:
    FEET = [(0.0, 0.0)]
PCA_BOSSES = [(PCA_C[0] + sx * PCA_HOLES_DX / 2, PCA_C[1] + sy * PCA_HOLES_DY / 2)
              for sx in (-1, 1) for sy in (-1, 1)]
SLOTS = [(SLOT_X, -24.75), (SLOT_X, 24.0 if RETAIN == 'snap' else 21.5)]
Z_PCA = FLOOR + BOSS_H


def _rect_hit(cx, cy, r, x0, x1, y0, y1):
    return (x0 - r) < cx < (x1 + r) and (y0 - r) < cy < (y1 + r)


def grid_points():
    """5 mm grid holes that stay 1 mm clear of foot recesses, bosses and slots."""
    pts = []
    fx, fy = FOOT_FLANGE[0] + 2 * FOOT_CLR, FOOT_FLANGE[1] + 2 * FOOT_CLR
    nx, ny = int(PX // GRID_PITCH), int(PY // GRID_PITCH)
    for i in range(-(nx // 2), nx // 2 + 1):
        for j in range(-(ny // 2), ny // 2 + 1):
            x, y = PLATE_OFFSET + i * GRID_PITCH, j * GRID_PITCH
            if abs(x - PLATE_OFFSET) > PX / 2 - 4 or abs(y) > PY / 2 - 4:
                continue
            if RETAIN == 'snap' and abs(y) < FOOT_FLANGE[1] / 2 + FOOT_CLR + 3.0 + 1.0:
                continue                      # channel band, roof must stay closed
            r = GRID_D / 2 + 1.0
            if any(_rect_hit(x, y, r, fx0 - fx / 2, fx0 + fx / 2, -fy / 2, fy / 2) for fx0, _ in FEET):
                continue
            if any((x - bx) ** 2 + (y - by) ** 2 < (BOSS_OD / 2 + r) ** 2 for bx, by in PCA_BOSSES):
                continue
            if any(_rect_hit(x, y, r, sx - STRAP_W / 2, sx + STRAP_W / 2, sy - STRAP_T / 2, sy + STRAP_T / 2)
                   for sx, sy in SLOTS):
                continue
            pts.append((x, y))
    return pts


def plate():
    p = (cq.Workplane('XY').center(PLATE_OFFSET, 0).rect(PX, PY).extrude(FLOOR)
         .edges('|Z').fillet(4))
    for (x, y) in PCA_BOSSES:
        p = p.union(cq.Workplane('XY').workplane(offset=FLOOR).center(x, y).circle(BOSS_OD / 2).extrude(BOSS_H))
        p = p.cut(cq.Workplane('XY').workplane(offset=Z_PCA - INSERT_DEPTH).center(x, y)
                  .circle(INSERT_D / 2).extrude(INSERT_DEPTH + 1))
    for (x, y) in SLOTS:
        p = p.cut(box(x - STRAP_W / 2, x + STRAP_W / 2, y - STRAP_T / 2, y + STRAP_T / 2, -1, FLOOR + 1))
    pts = grid_points()
    p = p.cut(cq.Workplane('XY').pushPoints(pts).circle(GRID_D / 2).extrude(FLOOR + 1).translate((0, 0, -0.5)))
    if RETAIN == 'snap':
        from snap_retain import channel_solids
        pair = FEET if len(FEET) == 2 else [(FEET[0][0] - 1e-3, 0.0), (FEET[0][0] + 1e-3, 0.0)]
        bar, cuts, adds = channel_solids(pair, 'x', FLOOR, extent=(PLATE_OFFSET - PX / 2, PLATE_OFFSET + PX / 2))
        p = p.union(bar)
        for c in cuts:
            p = p.cut(c)
        for a in adds:
            p = p.union(a)
        return p, pts
    for (x, y) in FEET:
        p = p.cut(foot_cutout(FLOOR, 'x').translate((x, y, 0)))
    return p, pts


def feet():
    if RETAIN == 'snap':
        from snap_retain import snap_feet
        return snap_feet(FEET, 'x')
    return [foot('x').translate((x, y, FLOOR)) for (x, y) in FEET]


def envelope():
    bx, by, bt = PCA_BOARD
    cx, cy = PCA_C
    e = box(cx - bx / 2, cx + bx / 2, cy - by / 2, cy + by / 2, Z_PCA, Z_PCA + bt)
    e = e.union(box(cx - bx / 2, cx + bx / 2, cy - by / 2, cy + by / 2, Z_PCA + bt, Z_PCA + bt + PCA_TOP_H))
    # solder tails under the board, between the bosses only
    e = e.union(box(cx - bx / 2 + 8, cx + bx / 2 - 8, cy - by / 2 + 2, cy + by / 2 - 2, Z_PCA - PCA_UNDER, Z_PCA))
    hx, hy, hz = HUB
    e = e.union(box(HUB_C[0] - hx / 2, HUB_C[0] + hx / 2, HUB_C[1] - hy / 2, HUB_C[1] + hy / 2, FLOOR, FLOOR + hz))
    return e


def assembly(p):
    if RETAIN == 'snap':
        from snap_retain import under_assembly
        return under_assembly('small_board_plate_under_clipless', p, feet(), FEET, envelope(),
                              plate_stub(FEET, size=(PX + 30, PY + 30)).translate((PLATE_OFFSET, 0, 0)))
    a = cq.Assembly(name='small_board_plate_on_clipless')
    a.add(plate_stub(FEET, size=(PX + 30, PY + 30)).translate((PLATE_OFFSET, 0, 0)),
          name='baseplate', color=cq.Color(0.55, 0.55, 0.6))
    for i, (x, y) in enumerate(FEET):
        a.add(clipless_piece().translate((x, y, -(RIM_TOP - RIM_PROUD))),
              name=f'clipless_{i}', color=cq.Color(0.63, 0.63, 0.63))
    a.add(p.translate((0, 0, RIM_PROUD)), name='plate', color=cq.Color(0.9, 0.45, 0.1))
    for i, f in enumerate(feet()):
        a.add(f.translate((0, 0, RIM_PROUD)), name=f'foot_{i}', color=cq.Color(0.2, 0.5, 0.9))
    a.add(envelope().translate((0, 0, RIM_PROUD)), name='boards_envelope', color=cq.Color(0.35, 0.35, 0.35))
    return a


def checks(p, pts):
    fs = feet()
    env = envelope()
    if RETAIN == 'snap':
        from snap_retain import hang, piece_from_above, play_check
        stub = plate_stub(FEET, size=(PX + 30, PY + 30)).translate((PLATE_OFFSET, 0, 0))
        pieces = [piece_from_above().translate((x, y, 0)) for (x, y) in FEET]
        at_stop, at_bump, play = play_check(p, fs, FEET, 'x', inter)
        return {
            'solids': len(p.val().Solids()),
            'plate_x_feet': sum(inter(p, f) for f in fs),
            'plate_x_feet_at_stop': at_stop,
            'plate_x_feet_at_bump': at_bump,
            'fore_aft_play_between_stop_and_bump': play,
            'feet_x_clipless': sum(inter(hang(f), q) for f in fs for q in pieces),
            'plate_x_plate_stub': inter(hang(p), stub),
            'plate_x_envelope': inter(p, env),
            'feet_x_envelope': sum(inter(f, env) for f in fs),
            'hanging_depth_below_plate_top': round(-hang(env).val().BoundingBox().zmin, 2),
            'grid_holes': len(pts),
            'feet': FEET,
            'slots': SLOTS,
        }
    stub = plate_stub(FEET, size=(PX + 30, PY + 30)).translate((PLATE_OFFSET, 0, -RIM_PROUD))
    pieces = [clipless_piece().translate((x, y, -RIM_TOP)) for (x, y) in FEET]
    # nothing may stand over a flange recess, or the foot cannot be dropped in
    over = 0.0
    for (x, y) in FEET:
        col = (cq.Workplane('XY').workplane(offset=FLOOR).rect(FOOT_FLANGE[0], FOOT_FLANGE[1])
               .extrude(100).translate((x, y, 0)))
        over += inter(p, col)
    return {
        'solids': len(p.val().Solids()),
        'material_over_foot_slots': over,
        'plate_x_feet': sum(inter(p, f) for f in fs),
        'feet_x_clipless': sum(inter(f, q) for f in fs for q in pieces),
        'plate_x_plate_stub': inter(p, stub),
        'plate_x_envelope': inter(p, env),
        'feet_x_envelope': sum(inter(f, env) for f in fs),
        'grid_holes': len(pts),
        'pca_bosses': PCA_BOSSES,
        'slots': SLOTS,
    }


if __name__ == '__main__':
    p, pts = plate()
    bb = p.val().BoundingBox()
    N = 'small_board_plate' + SUFFIX
    cq.exporters.export(p, os.path.join(OUT, N + '.step'))
    cq.exporters.export(p, os.path.join(OUT, N + '.stl'), tolerance=0.02, angularTolerance=0.1)
    a = assembly(p)
    a.save(os.path.join(OUT, N + '_assembly.step'))
    vol = p.val().Volume()
    print(f'plate {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm, {vol/1000:.1f} cm3 '
          f'(~{vol/1000*1.24*0.6:.0f} g PLA at 60% effective), feet at {FEET}, retention {RETAIN}')
    for k, v in checks(p, pts).items():
        print(f'  {k}: {v}')
    render([(p, (0.9, 0.45, 0.1))], os.path.join(OUT, N + '.png'),
           views=[(28, -55), (28, 125)], title=f'Small-board plate ({RETAIN})')
    fs = feet()
    stub = plate_stub(FEET, size=(PX + 30, PY + 30)).translate((PLATE_OFFSET, 0, 0))
    if RETAIN == 'snap':
        from snap_retain import under_shaded
        shaded = under_shaded(p, fs, FEET, envelope(), stub)
        title, el, eye = 'Small-board plate hanging under the plate (grey = PCA9685 and hub)', -25, (0.57, -0.82, -0.47)
    else:
        pieces = [clipless_piece().translate((x, y, -(RIM_TOP - RIM_PROUD))) for (x, y) in FEET]
        up = lambda s: s.translate((0, 0, RIM_PROUD))
        shaded = [(stub, (0.55, 0.55, 0.6))] + [(q, (0.63, 0.63, 0.63)) for q in pieces] + \
                 [(up(p), (0.9, 0.45, 0.1))] + [(up(f), (0.2, 0.5, 0.9)) for f in fs] + \
                 [(up(envelope()), (0.35, 0.35, 0.35))]
        title, el, eye = 'Small-board plate on clipless (grey = PCA9685 and hub envelopes)', 28, (0.57, -0.82, 0.47)
    render(shaded, os.path.join(OUT, N + '_assembly.png'), views=[(el, -55), (el, 125)], title=title)
    lines_png([q for q, _ in shaded], os.path.join(OUT, N + '_assembly_lines.svg'),
              os.path.join(OUT, N + '_assembly_lines.png'), eye=eye)
    lines_png([p], os.path.join(OUT, N + '_lines.svg'), os.path.join(OUT, N + '_lines.png'))
