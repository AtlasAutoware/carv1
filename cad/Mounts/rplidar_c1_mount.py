"""Plinth for the SLAMTEC RPLIDAR C1 on the clipless mounting system.

RPLIDAR C1 (datasheet rev 1.1, Figure 4-1): 55.6 x 55.6 mm square base, 23.1 mm base plus
18.2 mm optical head = 41.3 mm tall, 110 g. Scan plane 29.8 mm above the underside. Four
M2.5 threaded holes in the underside on a 43 x 43 mm square, screw engagement 4 mm max.
XH2.54-5P connector at the centre of the rear edge of the base; the zero-angle mark and
the x-axis point away from it (Figure 2-4).

The plinth is a 62 x 62 hollow box whose deck carries the lidar in a 2 mm keyed pocket
(the only gap in the pocket rim is the connector notch at the rear, so the lidar can only
sit with its x-axis forward). The deck height is set by LIDAR_SCAN_CLEARANCE: the scan
plane must clear the tallest thing on the board. Four M2.5 x 8 screws go up through the
deck into the lidar's own threads; their heads are reached from the open underside before
the plinth goes on the car.

Feet: nothing on this part sits on the feet flanges to trap them (the deck is above them),
so instead of the drop-in recess of clipless.foot_cutout the floor carries a dovetail
slide channel along the car: the two feet go into their clipless pockets first, the plinth
slides on from the rear over both flanges, a spring tongue snaps up behind the rear flange
and the 45 degree lips stop the plinth lifting off the feet. Same foot, same clearances.

    python3 rplidar_c1_mount.py
    LIDAR_SCAN_CLEARANCE=50 PEG_PITCH=50 python3 rplidar_c1_mount.py
"""
import os, sys, math
import cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from clipless import (foot, clipless_piece, plate_stub, RIM_PROUD, RIM_TOP, FOOT_FLANGE,
                      FOOT_FLANGE_T, FOOT_CLR, PEG_ACROSS)
from render import render
from jetson_orin_nano_mount import lines_png, box, inter

# ---- component (SLAMTEC_rplidar_datasheet_C1_v1.1_en.pdf, Figure 4-1) ----------------
LIDAR_BASE = 55.6            # square base, mm
LIDAR_BASE_H = 23.1          # base block height
LIDAR_HEAD_H = 18.2          # rotating optical head above the base
LIDAR_SCAN_H = 29.8          # laser emitting/receiving plane above the underside
LIDAR_HOLES = 43.0           # 4 x M2.5, square pattern
LIDAR_SCREW_MAX = 4.0        # max thread engagement in the lidar
LIDAR_MASS = 110.0
LIDAR_HEAD_D = 44.0          # ESTIMATE from the drawing raster (head is narrower than the base)
CONN = (6.0, 12.0, 7.0)      # ESTIMATE: XH2.54-5P socket protruding from the rear base edge
                             # (x protrusion, y width, z height above the underside)

# ---- plinth ------------------------------------------------------------------------------
LIDAR_SCAN_CLEARANCE = float(os.environ.get('LIDAR_SCAN_CLEARANCE', 34.0 + 10.0))  # above board top
PEG_PITCH = float(os.environ.get('PEG_PITCH', 49.0))
CLR = 0.25                   # pocket clearance per side (snug: the pocket locates the lidar)
WALL = 3.0
DECK_T = 4.0
RIM_H = 2.0                  # keyed pocket depth
SCREW_HOLE = 2.8
SCREW_HEAD_D = 5.5           # counterbore for an M2.5 socket head (4.5)
SCREW_LEN = 8.0              # M2.5 x 8: 4 in the deck + 4 in the lidar (the maximum)
BAR_W = FOOT_FLANGE[1] + 2 * FOOT_CLR + 2 * WALL    # 40.5, floor bar across the channel
BAR_T = 10.0                 # floor bar height, channel tunnel inside it

# channel (dovetail slide, see slide_channel)
CH_FLOOR = 2.0               # ledge under the flange
CH_LIP_Z0 = CH_FLOOR + FOOT_FLANGE_T          # 4.0, lip starts above the flange
CH_LIP_Z1 = CH_LIP_Z0 + 3.0                   # 7.0, lip top after a 45 degree flank
TONGUE_L = 14.5
TONGUE_SLOT = 0.6
BUMP_H = 0.6

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out')
N = 'rplidar_c1' + os.environ.get('OUT_SUFFIX', '')   # output file stem
os.makedirs(OUT, exist_ok=True)

# deck height: scan plane = RIM_PROUD + Z_DECK + LIDAR_SCAN_H >= LIDAR_SCAN_CLEARANCE
Z_DECK = max(math.ceil(LIDAR_SCAN_CLEARANCE - LIDAR_SCAN_H - RIM_PROUD), BAR_T + DECK_T)
Z_TOP = Z_DECK + RIM_H
BODY = LIDAR_BASE + 2 * CLR + 2 * WALL           # 62.1
POCKET = LIDAR_BASE + 2 * CLR                    # 56.1
FEET = [(-PEG_PITCH / 2, 0.0), (PEG_PITCH / 2, 0.0)]
HOLES = [(sx * LIDAR_HOLES / 2, sy * LIDAR_HOLES / 2) for sx in (-1, 1) for sy in (-1, 1)]
# channel runs along x, open at the rear (-x); closed-end stop just past the front flange
S_STOP = PEG_PITCH / 2 + FOOT_FLANGE[0] / 2 + 0.75             # 0.75 clear of the front flange
X_BUMP = -(PEG_PITCH / 2 + FOOT_FLANGE[0] / 2) - 1.0           # steep face 1.0 behind the rear flange
BAR_X0 = X_BUMP - 3.5                                          # rear (open) end
BAR_END = float(os.environ.get('BAR_END', 5.0))               # front end wall (>= 3 mm)
BAR_X1 = S_STOP + BAR_END


def slide_channel(s_open, s_stop, z_top=None):
    """Cut solid for the dovetail slide channel, running along +x from the open end at
    x=s_open to the stop face at x=s_stop, centred on y=0, floor underside at z=0.

    Cross-section (y, z): peg slot 24.3 wide through the 2 mm ledge, flange slot 34.5 wide
    from z 2 to 4, then 45 degree lips closing to 28.5 wide at z 7. The flange (34 x 2) sits
    on the ledge; it can lift 0.25 mm before its top corners wedge under the lips. Above
    z_top nothing is cut (default: the lips' top, so the floor above is left solid).
    """
    hp = PEG_ACROSS / 2 + FOOT_CLR                # 12.15
    hf = FOOT_FLANGE[1] / 2 + FOOT_CLR            # 17.25
    hl = hf - (CH_LIP_Z1 - CH_LIP_Z0)             # 14.25
    z_top = CH_LIP_Z1 if z_top is None else z_top
    pts = [(-hp, -1), (hp, -1), (hp, CH_FLOOR), (hf, CH_FLOOR), (hf, CH_LIP_Z0), (hl, CH_LIP_Z1)]
    if z_top > CH_LIP_Z1:
        pts += [(hl, z_top), (-hl, z_top)]
    pts += [(-hl, CH_LIP_Z1), (-hf, CH_LIP_Z0), (-hf, CH_FLOOR), (-hp, CH_FLOOR)]
    return (cq.Workplane('YZ', origin=(s_open, 0, 0)).polyline(pts).close()
            .extrude(s_stop - s_open))


def snap_tongues(s_edge, x_face):
    """Cuts and bumps that turn the last TONGUE_L of each ledge, from the open floor edge
    at x=s_edge, into a spring tongue (2 x 4.5 mm section, free at the edge) with a bump
    whose steep face at x=x_face snaps up behind the rear foot flange.
    Returns (cuts, adds): solids to subtract from and add to the floor."""
    hp = PEG_ACROSS / 2 + FOOT_CLR
    hf = FOOT_FLANGE[1] / 2 + FOOT_CLR
    root = s_edge + TONGUE_L
    cuts, adds = [], []
    for sy in (-1, 1):
        # slot between tongue and channel wall
        ya, yb = sorted((sy * (hf - TONGUE_SLOT), sy * hf))
        cuts.append(box(s_edge - 1, root, ya, yb, -1, CH_FLOOR + 0.6))
        # bump: ramp from the edge up to BUMP_H, short flat, 63 degree drop toward the foot
        y0, y1 = sorted((sy * (hp + 0.1), sy * (hf - TONGUE_SLOT - 0.1)))
        prof = [(s_edge, CH_FLOOR - 0.3), (x_face - 0.6, CH_FLOOR + BUMP_H), (x_face - 0.3, CH_FLOOR + BUMP_H),
                (x_face, CH_FLOOR - 0.3)]
        adds.append(cq.Workplane('XZ', origin=(0, y0, 0)).polyline(prof).close().extrude(-(y1 - y0)))
    return cuts, adds


def plinth():
    # floor bar with the slide channel tunnel
    bar = box(BAR_X0, BAR_X1, -BAR_W / 2, BAR_W / 2, 0, BAR_T)
    # hollow box body, open underneath, with a 4 mm deck
    body = cq.Workplane('XY').rect(BODY, BODY).extrude(Z_TOP).edges('|Z').fillet(2)
    body = body.cut(box(-BODY / 2 + WALL, BODY / 2 - WALL, -BODY / 2 + WALL, BODY / 2 - WALL, -1, Z_DECK - DECK_T))
    p = body.union(bar)
    p = p.cut(slide_channel(BAR_X0 - 1, S_STOP))
    cuts, adds = snap_tongues(BAR_X0, X_BUMP)
    for c in cuts:
        p = p.cut(c)
    for a in adds:
        p = p.union(a)
    # keyed pocket for the lidar base
    p = p.cut(cq.Workplane('XY').workplane(offset=Z_DECK).rect(POCKET, POCKET).extrude(RIM_H + 1)
              .edges('|Z').fillet(4))
    # connector notch at the rear: the only break in the rim, so it keys the orientation;
    # continues down the rear wall so the cable drops onto the bar extension
    p = p.cut(box(-BODY / 2 - 1, -POCKET / 2, -CONN[1] / 2 - 1, CONN[1] / 2 + 1, BAR_T, Z_TOP + 1))
    # lightening window in the deck centre, well inside the 43 mm hole pattern
    p = p.cut(cq.Workplane('XY').workplane(offset=Z_DECK - DECK_T - 1).rect(26, 26).extrude(DECK_T + 2)
              .edges('|Z').fillet(4))
    # front mark: small arrow notch in the front rim top, apex forward
    p = p.cut(cq.Workplane('XY').workplane(offset=Z_TOP - 0.6).center(BODY / 2 - 0.5, 0)
              .polyline([(-3, -2.5), (-3, 2.5), (0, 0)]).close().extrude(1))
    # screw holes on the lidar's pattern: 2.8 through the deck, 5.5 counterbore from below
    for (x, y) in HOLES:
        p = p.cut(cq.Workplane('XY').workplane(offset=Z_DECK - DECK_T - 1).center(x, y)
                  .circle(SCREW_HOLE / 2).extrude(DECK_T + 2))
        p = p.cut(cq.Workplane('XY').workplane(offset=-1).center(x, y)
                  .circle(SCREW_HEAD_D / 2).extrude(Z_DECK - DECK_T + 1))
    return p


def feet():
    return [foot('x').translate((x, y, CH_LIP_Z0)) for (x, y) in FEET]


def envelope():
    """Lidar where it sits: base block, head cylinder, connector at the rear edge."""
    z0 = Z_DECK
    e = (cq.Workplane('XY').workplane(offset=z0).rect(LIDAR_BASE, LIDAR_BASE).extrude(LIDAR_BASE_H)
         .edges('|Z').fillet(5))
    e = e.union(cq.Workplane('XY').workplane(offset=z0 + LIDAR_BASE_H).circle(LIDAR_HEAD_D / 2)
                .extrude(LIDAR_HEAD_H))
    e = e.union(box(-LIDAR_BASE / 2 - CONN[0], -LIDAR_BASE / 2 + 0.01, -CONN[1] / 2, CONN[1] / 2, z0, z0 + CONN[2]))
    return e


def assembly(p):
    a = cq.Assembly(name='rplidar_c1_on_clipless')
    a.add(plate_stub(FEET, size=(BAR_X1 - BAR_X0 + 20, BODY + 20)), name='baseplate', color=cq.Color(0.55, 0.55, 0.6))
    for i, (x, y) in enumerate(FEET):
        a.add(clipless_piece().translate((x, y, -(RIM_TOP - RIM_PROUD))), name=f'clipless_{i}',
              color=cq.Color(0.63, 0.63, 0.63))
    a.add(p.translate((0, 0, RIM_PROUD)), name='plinth', color=cq.Color(0.9, 0.45, 0.1))
    for i, f in enumerate(feet()):
        a.add(f.translate((0, 0, RIM_PROUD)), name=f'foot_{i}', color=cq.Color(0.2, 0.5, 0.9))
    a.add(envelope().translate((0, 0, RIM_PROUD)), name='rplidar_c1_envelope', color=cq.Color(0.35, 0.35, 0.35))
    return a


def checks(p):
    fs = feet()
    env = envelope()
    stub = plate_stub(FEET, size=(BAR_X1 - BAR_X0 + 20, BODY + 20)).translate((0, 0, -RIM_PROUD))
    pieces = [clipless_piece().translate((x, y, -RIM_TOP)) for (x, y) in FEET]
    # the plinth pushed against its stop / its bump with the feet where the plate puts them
    to_stop = p.translate((-(S_STOP - (PEG_PITCH / 2 + FOOT_FLANGE[0] / 2 + FOOT_CLR)), 0, 0))
    to_bump = p.translate((-(PEG_PITCH / 2 + FOOT_FLANGE[0] / 2 + FOOT_CLR) - X_BUMP, 0, 0))
    res = {
        'solids': len(p.val().Solids()),
        'plinth_x_feet': sum(inter(p, f) for f in fs),
        'plinth_x_feet_at_stop': sum(inter(to_stop, f) for f in fs),
        'plinth_x_feet_at_bump': sum(inter(to_bump, f) for f in fs),
        'fore_aft_play_between_stop_and_bump': (S_STOP - (PEG_PITCH / 2 + FOOT_FLANGE[0] / 2))
                                               + (-(PEG_PITCH / 2 + FOOT_FLANGE[0] / 2) - X_BUMP),
        'feet_x_clipless': sum(inter(f, c) for f in fs for c in pieces),
        'plinth_x_plate': inter(p, stub),
        'plinth_x_envelope': inter(p, env),
        'feet_x_envelope': sum(inter(f, env) for f in fs),
        'deck_z': Z_DECK,
        'scan_plane_above_board_top': RIM_PROUD + Z_DECK + LIDAR_SCAN_H,
        'required': LIDAR_SCAN_CLEARANCE,
        'lidar_top_above_board_top': RIM_PROUD + Z_DECK + LIDAR_BASE_H + LIDAR_HEAD_H,
        'screw_engagement_in_lidar': SCREW_LEN - DECK_T,
    }
    return res


if __name__ == '__main__':
    p = plinth()
    bb = p.val().BoundingBox()
    cq.exporters.export(p, os.path.join(OUT, N + '.step'))
    cq.exporters.export(p, os.path.join(OUT, N + '.stl'), tolerance=0.02, angularTolerance=0.1)
    a = assembly(p)
    a.save(os.path.join(OUT, N + '_assembly.step'))
    vol = p.val().Volume()
    print(f'plinth {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm, {vol/1000:.1f} cm3 '
          f'({vol/1000*1.24:.0f} g PLA solid, ~{vol/1000*1.24*0.6:.0f} g at 60% effective), '
          f'peg pitch {PEG_PITCH} mm')
    for k, v in checks(p).items():
        print(f'  {k}: {v}')
    render([(p, (0.9, 0.45, 0.1))], os.path.join(OUT, N + '.png'),
           views=[(28, -55), (28, 125)], title='RPLIDAR C1 plinth')
    fs = feet()
    pieces = [clipless_piece().translate((x, y, -(RIM_TOP - RIM_PROUD))) for (x, y) in FEET]
    stub = plate_stub(FEET, size=(BAR_X1 - BAR_X0 + 20, BODY + 20))
    up = lambda s: s.translate((0, 0, RIM_PROUD))
    shaded = [(stub, (0.55, 0.55, 0.6))] + [(c, (0.63, 0.63, 0.63)) for c in pieces] + \
             [(up(p), (0.9, 0.45, 0.1))] + [(up(f), (0.2, 0.5, 0.9)) for f in fs] + \
             [(up(envelope()), (0.35, 0.35, 0.35))]
    render(shaded, os.path.join(OUT, N + '_assembly.png'),
           views=[(28, -55), (28, 125)], title='RPLIDAR C1 plinth on clipless (grey = lidar envelope)')
    lines_png([stub] + pieces + [up(p)] + [up(f) for f in fs] + [up(envelope())],
              os.path.join(OUT, N + '_assembly_lines.svg'),
              os.path.join(OUT, N + '_assembly_lines.png'))
    lines_png([p], os.path.join(OUT, N + '_lines.svg'), os.path.join(OUT, N + '_lines.png'))
