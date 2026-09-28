"""Stand-off plate for the AtlasPower-4S power board on the clipless mounting system.

The board is 88 x 68 x 1.6 mm with four M3 holes on a 76 x 56 mm pattern (+-38, +-28 from
the board centre), dissipates about 8 W and has connectors on both x edges (screw terminal
and barrel jack at -x, screw terminals and a servo header at +x). So this is a flat plate,
not a tray: no walls anywhere, the board stands on four M3 heat-set-insert bosses BOSS_H
above the floor, and the two x edges are completely open. Two vent windows in the y bands
outside the foot recesses give the underside of the board a through-draught.

Two clipless feet along the car on PEG_PITCH; the feet are dropped into their flange
recesses from above before the board goes on, exactly as in small_board_plate_mount.py.
The board's holes are at x +-38, well outside the foot recesses (which reach x +-35.25),
so no boss and no window stands over a foot slot; checks() measures that.

PITCH_Y is read for the plate geometry report: the plate is wider than the hole pitch
across the car, so it covers part of the neighbouring hole column. See the MD.

    python3 power_board_mount.py
    PEG_PITCH=49 PITCH_Y=47 python3 power_board_mount.py
"""
import os, sys
import cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from clipless import (foot, foot_cutout, clipless_piece, plate_stub, RIM_PROUD, RIM_TOP,
                      HOLE, FOOT_FLANGE, FOOT_FLANGE_T, FOOT_CLR)
from render import render
from jetson_orin_nano_mount import lines_png, box, inter

# ---- component ------------------------------------------------------------------------
BOARD = (88.0, 68.0, 1.6)           # outline x, y, thickness; corners R3
BOARD_R = 3.0
HOLES_DX, HOLES_DY = 76.0, 56.0     # M3 holes at (+-38, +-28) from the board centre
HOLE_D = 3.2
BOARD_TOP_H = 12.0                  # tallest components above the board top face:
                                    # 13.5 x 12.5 x 6.2 shielded inductors, 10 x 10.5 caps
BOARD_UNDER = 2.0                   # lead / solder protrusion below the board
CONN_OUT = 15.0                     # how far the x-edge connectors reach past the outline
CONN_H = 12.0                       # connector height above the board top face
BOARD_MASS = 90.0                   # g, ESTIMATE (see the MD)

# ---- plate ----------------------------------------------------------------------------
PLATE = (92.0, 72.0)                # 2 mm proud of the board all round: the bosses need
                                    # 3.25 mm of floor outside their 9.5 mm OD
FLOOR = 4.0
CORNER_R = 4.0                      # >= the 2 mm minimum on vertical outside edges
BOSS_OD = 9.5                       # 3.5 mm insert hole + 3 mm wall
BOSS_H = 8.0                        # clearance under the board; 2 mm of that is leads,
                                    # so 6 mm of free air remains under the solder side
INSERT_D = float(os.environ.get('INSERT_D', 3.5))    # M3 heat-set, per the brief
INSERT_DEPTH = 5.0
VENT_Y = (20.5, 32.5)               # vent window band, |y|; 3.25 mm clear of the foot
VENT_X = 29.0                       # recesses and 3.5 mm clear of the plate edge
VENT_R = 2.0
WALL_MIN = 3.0

PEG_PITCH = float(os.environ.get('PEG_PITCH', 40.0))    # along the car
PITCH_Y = float(os.environ.get('PITCH_Y', 41.0))        # across the car
SUFFIX = os.environ.get('OUT_SUFFIX', '')

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out')
os.makedirs(OUT, exist_ok=True)

PX, PY = PLATE
FEET = [(-PEG_PITCH / 2, 0.0), (PEG_PITCH / 2, 0.0)]
BOSSES = [(sx * HOLES_DX / 2, sy * HOLES_DY / 2) for sx in (-1, 1) for sy in (-1, 1)]
Z_BOARD = FLOOR + BOSS_H
RECESS = (FOOT_FLANGE[0] + 2 * FOOT_CLR, FOOT_FLANGE[1] + 2 * FOOT_CLR)   # 30.5 x 34.5


def foot_access(floor_t, along_axis='x'):
    """Clear column above one foot's flange recess, from the recess floor to well above
    the part. Subtracting this guarantees the foot can be dropped in and pulled out."""
    fx, fy = FOOT_FLANGE if along_axis == 'x' else FOOT_FLANGE[::-1]
    return (cq.Workplane('XY').workplane(offset=floor_t - FOOT_FLANGE_T)
            .rect(fx + 2 * FOOT_CLR, fy + 2 * FOOT_CLR).extrude(200)
            .edges('|Z').fillet(2 + FOOT_CLR))


def _circle_to_rect(cx, cy, r, x0, x1, y0, y1):
    """Gap between a circle and an axis-aligned rectangle, in plan. Negative if they
    overlap in both axes; this is the number a wall-thickness rule needs, not a
    single-axis difference (a boss can be past the recess in x and still be clear
    because it is beside it in y)."""
    dx = max(x0 - cx, 0.0, cx - x1)
    dy = max(y0 - cy, 0.0, cy - y1)
    if dx == 0.0 and dy == 0.0:
        return -r - min(cx - x0, x1 - cx, cy - y0, y1 - cy)
    return (dx ** 2 + dy ** 2) ** 0.5 - r


def clearances():
    """Every distance the layout depends on, so the checks can report them."""
    rx, ry = RECESS
    boss_r = BOSS_OD / 2
    boss_gap = min(_circle_to_rect(bx, by, boss_r, fx - rx / 2, fx + rx / 2, -ry / 2, ry / 2)
                   for bx, by in BOSSES for fx, _ in FEET)
    return {
        'boss_to_foot_recess': round(boss_gap, 2),
        'boss_to_plate_edge_x': round(PX / 2 - (HOLES_DX / 2 + boss_r), 2),
        'boss_to_plate_edge_y': round(PY / 2 - (HOLES_DY / 2 + boss_r), 2),
        'vent_to_foot_recess_y': round(VENT_Y[0] - ry / 2, 2),
        'vent_to_plate_edge_y': round(PY / 2 - VENT_Y[1], 2),
        'vent_to_boss': round(min(_circle_to_rect(bx, by, boss_r, -VENT_X, VENT_X,
                                                  min(sy * VENT_Y[0], sy * VENT_Y[1]),
                                                  max(sy * VENT_Y[0], sy * VENT_Y[1]))
                                  for bx, by in BOSSES for sy in (-1, 1)), 2),
        'strip_between_foot_recesses_x': round(PEG_PITCH - rx, 2),
        'clearance_under_board': BOSS_H,
        'free_air_under_leads': round(BOSS_H - BOARD_UNDER, 2),
    }


def plate():
    p = cq.Workplane('XY').rect(PX, PY).extrude(FLOOR).edges('|Z').fillet(CORNER_R)
    for (x, y) in BOSSES:
        p = p.union(cq.Workplane('XY').workplane(offset=FLOOR).center(x, y)
                    .circle(BOSS_OD / 2).extrude(BOSS_H))
        p = p.cut(cq.Workplane('XY').workplane(offset=Z_BOARD - INSERT_DEPTH).center(x, y)
                  .circle(INSERT_D / 2).extrude(INSERT_DEPTH + 1))
    # airflow windows, in the two y bands outside the foot flange recesses
    for sy in (-1, 1):
        y0, y1 = sorted((sy * VENT_Y[0], sy * VENT_Y[1]))
        p = p.cut(box(-VENT_X, VENT_X, y0, y1, -1, FLOOR + 1).edges('|Z').fillet(VENT_R))
    for (x, y) in FEET:
        p = p.cut(foot_cutout(FLOOR, 'x').translate((x, y, 0)))
        # nothing may stand over the flange recess or the foot cannot be dropped in;
        # foot_cutout only reaches FLOOR + 1, so clear the whole column as well
        p = p.cut(foot_access(FLOOR, 'x').translate((x, y, 0)))
    return p


def feet():
    return [foot('x').translate((x, y, FLOOR)) for (x, y) in FEET]


def envelope():
    """The board where it really sits: outline with R3 corners on the bosses, component
    block above it, leads below it inside the hole pattern, connectors off both x edges."""
    bx, by, bt = BOARD
    e = (cq.Workplane('XY').workplane(offset=Z_BOARD).rect(bx, by).extrude(bt)
         .edges('|Z').fillet(BOARD_R))
    e = e.union(cq.Workplane('XY').workplane(offset=Z_BOARD + bt).rect(bx, by)
                .extrude(BOARD_TOP_H).edges('|Z').fillet(BOARD_R))
    # leads: the area inside the mounting-hole pattern, so it stays off the bosses
    lx = HOLES_DX / 2 - BOSS_OD / 2 - 2.0
    ly = HOLES_DY / 2 - BOSS_OD / 2 - 2.0
    e = e.union(box(-lx, lx, -ly, ly, Z_BOARD - BOARD_UNDER, Z_BOARD))
    # connectors off the -x and +x edges, at board level
    for sx in (-1, 1):
        x0, x1 = sorted((sx * bx / 2, sx * (bx / 2 + CONN_OUT)))
        e = e.union(box(x0, x1, -by / 2 + 6, by / 2 - 6, Z_BOARD, Z_BOARD + CONN_H))
    return e


def _stub():
    return plate_stub(FEET, size=(PX + 30, PY + 30))


def assembly(p):
    a = cq.Assembly(name='power_board_on_clipless')
    a.add(_stub(), name='baseplate', color=cq.Color(0.55, 0.55, 0.6))
    for i, (x, y) in enumerate(FEET):
        a.add(clipless_piece().translate((x, y, -(RIM_TOP - RIM_PROUD))),
              name=f'clipless_{i}', color=cq.Color(0.63, 0.63, 0.63))
    a.add(p.translate((0, 0, RIM_PROUD)), name='plate', color=cq.Color(0.9, 0.45, 0.1))
    for i, f in enumerate(feet()):
        a.add(f.translate((0, 0, RIM_PROUD)), name=f'foot_{i}', color=cq.Color(0.2, 0.5, 0.9))
    a.add(envelope().translate((0, 0, RIM_PROUD)), name='board_envelope',
          color=cq.Color(0.35, 0.35, 0.35))
    return a


def checks(p):
    fs = feet()
    env = envelope()
    stub = _stub().translate((0, 0, -RIM_PROUD))
    pieces = [clipless_piece().translate((x, y, -RIM_TOP)) for (x, y) in FEET]
    # nothing may stand in the column above a flange recess, from the flange top face up:
    # the 2 mm ledge the flange lands on is below that and is meant to be there
    over = 0.0
    for (x, y) in FEET:
        col = (cq.Workplane('XY').workplane(offset=FLOOR).rect(FOOT_FLANGE[0], FOOT_FLANGE[1])
               .extrude(200).translate((x, y, 0)))
        over += inter(p, col)
    res = {
        'solids': len(p.val().Solids()),
        'material_over_foot_slots': round(over, 2),
        'plate_x_feet': sum(inter(p, f) for f in fs),
        'feet_x_clipless': sum(inter(f, q) for f in fs for q in pieces),
        'plate_x_plate_stub': inter(p, stub),
        'plate_x_envelope': inter(p, env),
        'feet_x_envelope': sum(inter(f, env) for f in fs),
        'feet': FEET,
        'bosses': BOSSES,
        'board_underside_z': Z_BOARD,
        'plate_covers_adjacent_hole_across': round(PY / 2 - (PITCH_Y - HOLE / 2), 2),
    }
    res.update(clearances())
    res['min_wall_ok'] = all(v >= WALL_MIN for k, v in clearances().items()
                             if k.startswith(('boss_to', 'vent_to')))
    return res


if __name__ == '__main__':
    p = plate()
    bb = p.val().BoundingBox()
    N = 'power_board_mount' + SUFFIX
    cq.exporters.export(p, os.path.join(OUT, N + '.step'))
    cq.exporters.export(p, os.path.join(OUT, N + '.stl'), tolerance=0.02, angularTolerance=0.1)
    assembly(p).save(os.path.join(OUT, N + '_assembly.step'))
    vol = p.val().Volume()
    print(f'plate {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm, {vol/1000:.1f} cm3 '
          f'(~{vol/1000*1.24*0.6:.0f} g PLA at 60% effective), board {BOARD[0]} x {BOARD[1]} '
          f'on {HOLES_DX} x {HOLES_DY} holes, feet at {FEET} (pitch {PEG_PITCH} x {PITCH_Y})')
    for k, v in checks(p).items():
        print(f'  {k}: {v}')
    render([(p, (0.9, 0.45, 0.1))], os.path.join(OUT, N + '.png'),
           views=[(28, -55), (28, 125)], title='AtlasPower-4S plate')
    fs = feet()
    stub = _stub()
    pieces = [clipless_piece().translate((x, y, -(RIM_TOP - RIM_PROUD))) for (x, y) in FEET]
    up = lambda s: s.translate((0, 0, RIM_PROUD))
    shaded = [(stub, (0.55, 0.55, 0.6))] + [(q, (0.63, 0.63, 0.63)) for q in pieces] + \
             [(up(p), (0.9, 0.45, 0.1))] + [(up(f), (0.2, 0.5, 0.9)) for f in fs] + \
             [(up(envelope()), (0.35, 0.35, 0.35))]
    render(shaded, os.path.join(OUT, N + '_assembly.png'), views=[(28, -55), (28, 125)],
           title='AtlasPower-4S plate on clipless (grey = board, components, leads, connectors)')
    lines_png([q for q, _ in shaded], os.path.join(OUT, N + '_assembly_lines.svg'),
              os.path.join(OUT, N + '_assembly_lines.png'), eye=(0.57, -0.82, 0.47))
    lines_png([p], os.path.join(OUT, N + '_lines.svg'), os.path.join(OUT, N + '_lines.png'))
