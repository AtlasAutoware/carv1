"""Plinth for the SICK TiM561-2050101 (part 1071419) on the clipless mounting system, new
car. Four feet on two dovetail slide channels (snap_retain.channel_solids), the lidar in a
keyed pocket, two M3 screws up into its bottom threads and two M3 screws through rear
tabs into its rear-face threads.

TiM561 (SICK datasheet dataSheet_TiM561-2050101_1071419_en.pdf, dimensional drawing;
operating instructions 8015886): housing 60 wide x 61 deep, 58.45 tall to the hood base,
85.75 to the top of the optics hood; scan plane 62.46 above the bottom face; 250 g. Two M3
threads in the bottom face 51 apart, 16.79 from the rear face, and two in the rear face 51
apart, 24.4 up from the bottom; all 2.8 mm deep, 0.8 Nm max. A swivel connector unit at
the rear-bottom carries the M12 5-pin power plug and the M12 D-coded Ethernet socket; it
projects 17.37 behind the rear face when turned horizontal (the position used here; turned
down it would hang 15.4 below the bottom face, into the deck). Aperture 270 degrees, the
90 degree blind sector at the rear, where the connectors are. The swivel unit's width and
height, and the M12 plug bodies, are not dimensioned in the datasheet: SWIVEL_W / SWIVEL_H
/ M12 below are ESTIMATES and parameters (see TIM561_MOUNT.md).

Frame: feet-pair centre at the origin, +x forward, +z up, floor underside at z 0. The
lidar is shifted so its bottom screws fall at x = 0, between the two feet flanges of each
channel (the flanges end at |x| 5), where a counterbore from below reaches them through
the channel. Feet at (+-PEG_PITCH/2, +-PITCH_Y/2) = (+-20, +-20.5) on Baseplate v2.

    python3 tim561_mount.py
    PEG_PITCH=40 PITCH_Y=41 SWIVEL_W=50 python3 tim561_mount.py
"""
import os, sys, math
HERE = os.path.dirname(os.path.abspath(__file__))
PARENT = os.path.dirname(HERE)
sys.path.insert(0, PARENT)
import cadquery as cq
from clipless import foot, clipless_piece, plate_stub, RIM_PROUD, RIM_TOP, FOOT_FLANGE, FOOT_CLR
from render import render
from jetson_orin_nano_mount import lines_png, box, inter
from snap_retain import channel_solids, snap_feet, play_check, CH_LIP_Z0, CH_LIP_Z1, SNAP_FLOOR

# ---- component (SICK datasheet drawing) -------------------------------------------------
LIDAR_W = 60.0               # across the car (y)
LIDAR_D = 61.0               # along the car (x), rear face to front face
LIDAR_BODY_H = 58.45         # die-cast lower part, bottom face to hood base
LIDAR_H = 85.75              # to the top of the optics hood
LIDAR_SCAN_H = 62.46         # scan plane above the bottom face
LIDAR_MASS = 250.0
HOLE_DY = 51.0               # both M3 pairs, across
BOTTOM_HOLE_FROM_REAR = 16.79
REAR_HOLE_Z = 24.4           # rear-face pair above the bottom face
THREAD_DEPTH = 2.8           # blind, max engagement
SWIVEL_BEHIND = 17.37        # swivel connector unit behind the rear face, horizontal
HOOD_R0, HOOD_R1 = 29.0, 21.0    # ESTIMATE: conical hood, base and top radii (drawing raster)
SWIVEL_W = float(os.environ.get("SWIVEL_W", 40.0))    # ESTIMATE, swivel unit width (the rear M3 holes at |y| 25.5 must stay clear of it)
SWIVEL_H = float(os.environ.get('SWIVEL_H', 18.0))    # ESTIMATE, swivel unit height above the bottom face
M12 = (15.0, 45.0, 9.0, 10.0)   # ESTIMATE: plug body dia, length behind the swivel unit, axis z, |y| of the two plugs

# ---- plinth ------------------------------------------------------------------------------
PEG_PITCH = float(os.environ.get('PEG_PITCH', 40.0))
PITCH_Y = float(os.environ.get('PITCH_Y', 41.0))
CLR = 0.25                   # pocket, snug
WALL = 3.0
REAR_WALL_T = 3.5            # carries the rear M3 screws: 3.5 + 2.5 engagement = M3 x 6
DECK_T = 4.0
BAR_T = 10.0                 # channel bar (>= SNAP_FLOOR 8.6)
RIM_H = 2.0                  # keyed pocket depth
SCREW_HOLE = 3.2
SCREW_HEAD_D = 6.0           # counterbore for an M3 socket head (5.5)
BOTTOM_SCREW_LEN = 6.0       # M3 x 6 socket head (3 mm tall head in the z 7..10 counterbore): 4 in the deck + 2 in the lidar (max 2.8)
REAR_SCREW_LEN = 6.0         # M3 x 6: 3.5 in the wall + 2.5 in the lidar
TAB_IN = float(os.environ.get("TAB_IN", SWIVEL_W / 2 + 1.5))   # rear wall tabs from |y| TAB_IN outward, swivel unit between
TAB_H_ABOVE_HOLE = 6.0
BAR_END = 4.0                # stop wall beyond S_STOP (snap_retain STOP_WALL)

OUT = os.path.join(HERE, 'out')
os.makedirs(OUT, exist_ok=True)
N = 'tim561' + os.environ.get('OUT_SUFFIX', '')

Z_DECK = BAR_T + DECK_T                              # 14
Z_TOP = Z_DECK + RIM_H                               # 16
X_REAR = -BOTTOM_HOLE_FROM_REAR                      # lidar rear face: bottom screws at x = 0
X_FRONT = X_REAR + LIDAR_D
BX0, BX1 = X_REAR - CLR - REAR_WALL_T, X_FRONT + CLR + WALL      # body -20.54 .. 47.46
BY = LIDAR_W / 2 + CLR + WALL                                    # 33.25
FEET = [(sx * PEG_PITCH / 2, sy * PITCH_Y / 2) for sy in (-1, 1) for sx in (-1, 1)]
ROWS = sorted(set(y for _, y in FEET))
S_STOP = PEG_PITCH / 2 + FOOT_FLANGE[0] / 2 + 0.75               # 35.75
X_BUMP = -(PEG_PITCH / 2 + FOOT_FLANGE[0] / 2) - 1.0             # -36
BAR_X0, BAR_X1 = X_BUMP - 3.5, S_STOP + BAR_END                  # -39.5 .. 39.75
BOTTOM_HOLES = [(0.0, sy * HOLE_DY / 2) for sy in (-1, 1)]
REAR_HOLES = [(sy * HOLE_DY / 2, Z_DECK + REAR_HOLE_Z) for sy in (-1, 1)]
Z_TAB_TOP = Z_DECK + REAR_HOLE_Z + TAB_H_ABOVE_HOLE


def plinth():
    # body: box with a 4 mm deck, hollow below it, walls to the floor
    p = box(BX0, BX1, -BY, BY, 0, Z_TOP).edges('|Z').fillet(2)
    p = p.cut(box(BX0 + REAR_WALL_T, BX1 - WALL, -BY + WALL, BY - WALL, -1, Z_DECK - DECK_T))
    # rear wall: tabs at the corners rise to the rear M3 holes; the middle is open from the
    # deck up for the swivel connector unit
    for sy in (-1, 1):
        ya, yb = sorted((sy * TAB_IN, sy * BY))
        p = p.union(box(BX0, BX0 + REAR_WALL_T, ya, yb, Z_TOP - 1, Z_TAB_TOP).edges('|X').fillet(1.5))
    p = p.cut(box(BX0 - 1, BX0 + REAR_WALL_T + 1, -TAB_IN, TAB_IN, Z_DECK, Z_TAB_TOP + 1))
    # channel bars, one per foot row, with the dovetail slide + tongues (snap_retain)
    for row in ROWS:
        pair = [(x, y) for (x, y) in FEET if y == row]
        bar, cuts, adds = channel_solids(pair, 'x', BAR_T, extent=(BAR_X0, BAR_X1))
        p = p.union(bar)
        for c in cuts:
            p = p.cut(c)
        for a in adds:
            p = p.union(a)
    # keyed pocket for the housing (0.25 a side), open to the rear between the tabs
    p = p.cut(box(X_REAR - CLR, X_FRONT + CLR, -LIDAR_W / 2 - CLR, LIDAR_W / 2 + CLR, Z_DECK, Z_TOP + 1)
              .edges('|Z').fillet(3))
    # lightening window in the deck, inside the screw bosses and the pocket rim
    p = p.cut(box(6.0, X_FRONT - 8.0, -18.0, 18.0, Z_DECK - DECK_T - 1, Z_DECK + 1).edges('|Z').fillet(4))
    # bottom screws: 3.2 through the deck, 6 mm counterbore in the channel ROOF (z 7..10) so
    # the head sits above the flange path (flanges ride at z 2..4 and sweep the whole channel
    # length when the plinth slides on); the driver reaches it from below through the peg slot
    for (x, y) in BOTTOM_HOLES:
        p = p.cut(cq.Workplane('XY').workplane(offset=CH_LIP_Z1 - 0.01).center(x, y).circle(SCREW_HOLE / 2).extrude(Z_DECK + 2))
        p = p.cut(cq.Workplane('XY').workplane(offset=CH_LIP_Z1 - 0.01).center(x, y).circle(SCREW_HEAD_D / 2)
                  .extrude(Z_DECK - DECK_T - CH_LIP_Z1 + 0.01))
    # rear screws through the tabs
    for (y, z) in REAR_HOLES:
        p = p.cut(cq.Workplane('YZ', origin=(BX0 - 1, 0, 0)).center(y, z).circle(SCREW_HOLE / 2).extrude(REAR_WALL_T + 2))
    # front mark
    p = p.cut(cq.Workplane('XY').workplane(offset=Z_TOP - 0.6).center(BX1 - 0.5, 0)
              .polyline([(-3, -2.5), (-3, 2.5), (0, 0)]).close().extrude(1))
    return p


def feet():
    return snap_feet(FEET, 'x')


def envelope():
    """Lidar where it sits: housing block, conical hood, swivel unit and two M12 plug
    bodies pointing aft (horizontal swivel position)."""
    z0 = Z_DECK
    e = box(X_REAR, X_FRONT, -LIDAR_W / 2, LIDAR_W / 2, z0, z0 + LIDAR_BODY_H).edges('|Z').fillet(4)
    hood = cq.Workplane('XY').add(cq.Solid.makeCone(HOOD_R0, HOOD_R1, LIDAR_H - LIDAR_BODY_H,
                                                    cq.Vector((X_REAR + X_FRONT) / 2, 0, z0 + LIDAR_BODY_H)))
    e = e.union(hood)
    e = e.union(box(X_REAR - SWIVEL_BEHIND, X_REAR + 0.01, -SWIVEL_W / 2, SWIVEL_W / 2, z0, z0 + SWIVEL_H))
    d, l, zc, yc = M12
    for sy in (-1, 1):
        e = e.union(cq.Workplane('XY').add(cq.Solid.makeCylinder(
            d / 2, l, cq.Vector(X_REAR - SWIVEL_BEHIND - l, sy * yc, z0 + zc), cq.Vector(1, 0, 0))))
    return e


def scan_plane_z():
    """Scan plane above the board top for a mount sitting on the rim tops."""
    return RIM_PROUD + Z_DECK + LIDAR_SCAN_H


def assembly(p):
    stub = plate_stub(FEET, size=(BAR_X1 - BAR_X0 + 30, 2 * BY + 40))
    a = cq.Assembly(name='tim561_on_clipless')
    a.add(stub, name='baseplate', color=cq.Color(0.55, 0.55, 0.6))
    for i, (x, y) in enumerate(FEET):
        a.add(clipless_piece().translate((x, y, -(RIM_TOP - RIM_PROUD))), name=f'clipless_{i}', color=cq.Color(0.63, 0.63, 0.63))
    a.add(p.translate((0, 0, RIM_PROUD)), name='plinth', color=cq.Color(0.9, 0.45, 0.1))
    for i, f in enumerate(feet()):
        a.add(f.translate((0, 0, RIM_PROUD)), name=f'foot_{i}', color=cq.Color(0.2, 0.5, 0.9))
    a.add(envelope().translate((0, 0, RIM_PROUD)), name='tim561_envelope', color=cq.Color(0.35, 0.35, 0.35))
    return a


def checks(p):
    fs = feet()
    env = envelope()
    stub = plate_stub(FEET, size=(BAR_X1 - BAR_X0 + 30, 2 * BY + 40)).translate((0, 0, -RIM_PROUD))
    pieces = [clipless_piece().translate((x, y, -RIM_TOP)) for (x, y) in FEET]
    at_stop, at_bump, play = play_check(p, fs, FEET, 'x', inter)
    return {
        'solids': len(p.val().Solids()),
        'plinth_x_feet': sum(inter(p, f) for f in fs),
        'plinth_x_feet_at_stop': at_stop,
        'plinth_x_feet_at_bump': at_bump,
        'fore_aft_play_between_stop_and_bump': play,
        'feet_x_clipless': sum(inter(f, c) for f in fs for c in pieces),
        'plinth_x_plate': inter(p, stub),
        'plinth_x_envelope': inter(p, env),
        'feet_x_envelope': sum(inter(f, env) for f in fs),
        'deck_z': Z_DECK,
        'scan_plane_above_board_top': round(scan_plane_z(), 2),
        'lidar_top_above_board_top': round(RIM_PROUD + Z_DECK + LIDAR_H, 2),
        'bottom_screw_engagement': BOTTOM_SCREW_LEN - DECK_T,
        'bottom_screw_head_clear_above_flange_path': round((Z_DECK - DECK_T) - 3.0 - CH_LIP_Z0, 2),
        'rear_screw_engagement': REAR_SCREW_LEN - REAR_WALL_T,
        'thread_depth_max': THREAD_DEPTH,
        'rear_hole_z_in_part': REAR_HOLES[0][1],
        'body_x': (BX0, BX1), 'body_y': (-BY, BY), 'bar_x': (BAR_X0, BAR_X1),
        'swivel_gap_between_tabs': 2 * TAB_IN,
    }


if __name__ == '__main__':
    p = plinth()
    bb = p.val().BoundingBox()
    cq.exporters.export(p, os.path.join(OUT, N + '.step'))
    cq.exporters.export(p, os.path.join(OUT, N + '.stl'), tolerance=0.02, angularTolerance=0.1)
    assembly(p).save(os.path.join(OUT, N + '_assembly.step'))
    vol = p.val().Volume()
    print(f'plinth {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm, {vol/1000:.1f} cm3 '
          f'({vol/1000*1.24:.0f} g PLA solid, ~{vol/1000*1.24*0.6:.0f} g at 60% effective), feet {FEET}')
    for k, v in checks(p).items():
        print(f'  {k}: {v}')
    render([(p, (0.9, 0.45, 0.1))], os.path.join(OUT, N + '.png'), views=[(28, -55), (-25, 125)], title='TiM561 plinth')
    fs = feet()
    pieces = [clipless_piece().translate((x, y, -(RIM_TOP - RIM_PROUD))) for (x, y) in FEET]
    stub = plate_stub(FEET, size=(BAR_X1 - BAR_X0 + 30, 2 * BY + 40))
    up = lambda s: s.translate((0, 0, RIM_PROUD))
    shaded = [(stub, (0.55, 0.55, 0.6))] + [(c, (0.63, 0.63, 0.63)) for c in pieces] + \
             [(up(p), (0.9, 0.45, 0.1))] + [(up(f), (0.2, 0.5, 0.9)) for f in fs] + [(up(envelope()), (0.35, 0.35, 0.35))]
    render(shaded, os.path.join(OUT, N + '_assembly.png'), views=[(28, -55), (28, 125)],
           title='TiM561 plinth on clipless (grey = lidar envelope, connectors aft)')
    lines_png([q for q, _ in shaded], os.path.join(OUT, N + '_assembly_lines.svg'), os.path.join(OUT, N + '_assembly_lines.png'))
    lines_png([p], os.path.join(OUT, N + '_lines.svg'), os.path.join(OUT, N + '_lines.png'))
