"""Screw-free tray for the NVIDIA Jetson Orin Nano Developer Kit carrier (P3768) on the
clipless mounting system.

Carrier board: 100.00 x 79.00 mm PCB, 1.57 mm thick, components 16.70 mm max above and
4.30 mm max below the board (Figure 4-1, Jetson Orin Nano Developer Kit Carrier Board
Specification SP-11324-001_v1.0). Four plain mounting holes, 2.7 mm, on a 92.0 x 58.0 mm
pattern: 4.0 mm in from both short edges, 4.0 mm in from the 40-pin-header edge and
17.0 mm in from the connector edge (measured off the Figure 4-1 raster, see the MD).

The board drops onto four 6 mm standoff bosses (3.5 mm heat-set holes for optional M2.5
screws), hooks under two fixed lips at the -x end and snaps under two thin fingers at the
+x end. Both long edges are open: the connector edge (-y: DC jack, USB-C, 2x2 USB-A,
Ethernet, DisplayPort) has no wall at all, the header edge (+y) only has short corner
stubs outside the 40-pin header. Floor windows leave the M.2 sockets on the underside in
free air. Four separate clipless feet on a PEG_PITCH x PITCH_Y grid.

    python3 jetson_orin_nano_mount.py
    PEG_PITCH=50 PITCH_Y=45.4 python3 jetson_orin_nano_mount.py
"""
import os, sys
import cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from clipless import foot, foot_cutout, clipless_piece, plate_stub, RIM_PROUD, RIM_TOP, FOOT_FLANGE_T
from render import render

# ---- component (carrier board spec, SP-11324-001_v1.0, Figure 4-1) ------------------
BOARD = (100.0, 79.0, 1.57)      # x along the car, y across, PCB thickness
ABOVE_MAX = 16.70                # connector height above the board
BELOW_MAX = 4.30                 # M.2 cards below the board
HOLE_DX, HOLE_DY = 92.0, 58.0    # mounting-hole pattern (measured off Figure 4-1)
HOLE_EDGE = 4.0                  # inset from the short edges and the header edge
HOLE_D = 2.7
TOP_H = 27.0                     # ESTIMATE: module + heatsink + fan above the board top
                                 # (kit is 34.77 tall including plastic base and feet)

# ---- tray -----------------------------------------------------------------------------
CLR = 0.5                        # per side, drop-in
FLOOR = 4.0
WALL = 3.0
STANDOFF = 6.0                   # airflow / M.2 clearance under the board
BOSS_OD = 9.5                    # 3.5 mm insert hole + 3 mm wall
INSERT_D, INSERT_DEPTH = 3.5, 5.0
LEG = 12.0                       # corner post leg length along the board edge
LEG_SHORT = 6.5                  # (+x,+y) corner: keep clear of the button header J14
LIP_FIXED = 1.2                  # overhang of the fixed lips at -x
LIP_SNAP = 0.4                   # overhang of the snap lips at +x
FINGER_T = 1.6                   # snap finger thickness (deliberate spring, below the
                                 # 3 mm wall rule; see the MD)
LIP_CLR = 0.3                    # above the board top
LIP_T = 1.5
PEG_PITCH = float(os.environ.get('PEG_PITCH', 49.0))    # along the car
PITCH_Y = float(os.environ.get('PITCH_Y', 47.0))        # across the car

# RETAIN='snap': under-board variant. The flange recesses are replaced by the dovetail
# slide channel + snap tongues of rplidar_c1_mount.py (via snap_retain.py) so the tray can
# hang from feet plugged in from below. The floor grows to SNAP_FLOOR in the channel bands.
# SNAP_ROWS=2 (default) keeps four feet on two channels at y = +-PITCH_Y/2; SNAP_ROWS=1
# puts one channel on the tray centreline with two feet (the kit is under 400 g).
RETAIN = os.environ.get('RETAIN', 'recess')
SNAP_ROWS = int(os.environ.get('SNAP_ROWS', 2))
SNAP_FLOOR = 7.0 + 1.6                       # snap_retain.SNAP_FLOOR (channel + roof)
if RETAIN == 'snap':
    FLOOR = SNAP_FLOOR
SUFFIX = ('_under' if RETAIN == 'snap' else '') + os.environ.get('OUT_SUFFIX', '')

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out')
os.makedirs(OUT, exist_ok=True)

BX, BY, BT = BOARD
IX, IY = BX + 2 * CLR, BY + 2 * CLR          # pocket
OL, OW = IX + 2 * WALL, IY + 2 * WALL        # tray outline 107 x 86
Z_BOARD = FLOOR + STANDOFF                   # board underside
Z_LIP = Z_BOARD + BT + LIP_CLR               # lip underside
Z_TOP = Z_LIP + LIP_T                        # post top
HOLES = [(sx * HOLE_DX / 2, BY / 2 - HOLE_EDGE - dy) for sx in (-1, 1) for dy in (0, HOLE_DY)]
if RETAIN == 'snap' and SNAP_ROWS == 1:
    FEET = [(sx * PEG_PITCH / 2, 0.0) for sx in (-1, 1)]
else:
    FEET = [(sx * PEG_PITCH / 2, sy * PITCH_Y / 2) for sx in (-1, 1) for sy in (-1, 1)]
FEET_ROWS = sorted(set(y for _, y in FEET))


def box(x0, x1, y0, y1, z0, z1):
    return (cq.Workplane('XY').workplane(offset=z0)
            .center((x0 + x1) / 2, (y0 + y1) / 2).rect(x1 - x0, y1 - y0).extrude(z1 - z0))


def tray():
    t = cq.Workplane('XY').rect(OL, OW).extrude(FLOOR).edges('|Z').fillet(4)
    # airflow windows: a cross between the four foot pads, well clear of the bosses
    t = t.cut(box(-6.5, 6.5, -OW / 2 + 6, OW / 2 - 6, -1, FLOOR + 1).edges('|Z').fillet(2))
    t = t.cut(box(-OL / 2 + 6, OL / 2 - 6, -3.5, 3.5, -1, FLOOR + 1).edges('|Z').fillet(1.5))
    # standoff bosses on the carrier's hole pattern, with heat-set insert holes
    for (x, y) in HOLES:
        t = t.union(cq.Workplane('XY').workplane(offset=FLOOR).center(x, y)
                    .circle(BOSS_OD / 2).extrude(STANDOFF))
        t = t.cut(cq.Workplane('XY').workplane(offset=Z_BOARD - INSERT_DEPTH).center(x, y)
                  .circle(INSERT_D / 2).extrude(INSERT_DEPTH + 1))
    # -x end: two 3 mm legs along the short edge with fixed lips over the board
    for sy in (-1, 1):
        y0, y1 = (IY / 2 - LEG, IY / 2) if sy > 0 else (-IY / 2, -IY / 2 + LEG)
        t = t.union(box(-IX / 2 - WALL, -IX / 2, y0, y1, FLOOR, Z_TOP))
        t = t.union(box(-IX / 2 - 0.01, -IX / 2 + LIP_FIXED, y0, y1, Z_LIP, Z_TOP))
    # +x end: thin snap fingers with a chamfered lip
    for sy in (-1, 1):
        leg = LEG_SHORT if sy > 0 else LEG
        y0, y1 = (IY / 2 - leg, IY / 2) if sy > 0 else (-IY / 2, -IY / 2 + leg)
        t = t.union(box(IX / 2, IX / 2 + FINGER_T, y0, y1, FLOOR, Z_TOP))
        xi = IX / 2 + 0.01
        prof = [(xi, Z_LIP), (xi - LIP_SNAP, Z_LIP + LIP_SNAP), (xi - LIP_SNAP, Z_TOP - 0.5),
                (xi, Z_TOP)]
        lip = (cq.Workplane('XZ', origin=(0, y1, 0)).polyline(prof).close().extrude(y1 - y0))
        t = t.union(lip)
    # header edge (+y): corner stubs outside the 40-pin header, locate the board in +y
    for sx in (-1, 1):
        x0, x1 = (IX / 2 - LEG, IX / 2 + WALL) if sx > 0 else (-IX / 2 - WALL, -IX / 2 + LEG)
        t = t.union(box(x0, x1, IY / 2, IY / 2 + WALL, FLOOR, Z_TOP))
    if RETAIN == 'snap':
        # slide channel(s) along x through the feet, open at the -x end; the band under
        # each channel is made solid again first (the airflow windows crossed it)
        from snap_retain import channel_solids
        for row in FEET_ROWS:
            pair = [(x, y) for (x, y) in FEET if y == row]
            bar, cuts, adds = channel_solids(pair, 'x', FLOOR, extent=(-OL / 2, OL / 2))
            t = t.union(bar)
            for c in cuts:
                t = t.cut(c)
            for a in adds:
                t = t.union(a)
        return t
    # foot cutouts, four, along x
    for (x, y) in FEET:
        t = t.cut(foot_cutout(FLOOR, 'x').translate((x, y, 0)))
    return t


def feet():
    if RETAIN == 'snap':
        from snap_retain import snap_feet
        return snap_feet(FEET, 'x')
    return [foot('x').translate((x, y, FLOOR)) for (x, y) in FEET]


def envelope():
    """Where the kit really sits: board, connectors (overhanging -y by 2 mm), header,
    module/heatsink block on top, M.2 card underneath."""
    z0 = Z_BOARD
    parts = [
        box(-BX / 2, BX / 2, -BY / 2, BY / 2, z0, z0 + BT),                       # PCB
        # connector strip: DC jack starts 2.4 mm in from the -x edge (Figure 4-1), USB-C
        # about the same from +x; bodies overhang the -y edge by up to 2 mm
        box(-BX / 2 + 2.4, BX / 2 - 2.4, -BY / 2 - 2.0, -BY / 2 + 17.5, z0 + BT, z0 + BT + ABOVE_MAX),
        box(-28.0, 7.0, BY / 2 - 5.5, BY / 2 - 0.5, z0 + BT, z0 + BT + 8.5),        # 40-pin header
        box(-43.0, 43.0, -21.0, 33.0, z0 + BT, z0 + BT + TOP_H),                  # module + heatsink
        box(-40.0, 40.0, -20.0, 2.0, z0 - BELOW_MAX, z0),                         # M.2 2280 underneath
    ]
    e = parts[0]
    for p in parts[1:]:
        e = e.union(p)
    return e


def assembly(t):
    if RETAIN == 'snap':
        from snap_retain import under_assembly
        return under_assembly('jetson_orin_nano_under_clipless', t, feet(), FEET, envelope(),
                              plate_stub(FEET, size=(OL + 20, OW + 20)))
    a = cq.Assembly(name='jetson_orin_nano_on_clipless')
    a.add(plate_stub(FEET, size=(OL + 20, OW + 20)), name='baseplate', color=cq.Color(0.55, 0.55, 0.6))
    for i, (x, y) in enumerate(FEET):
        a.add(clipless_piece().translate((x, y, -(RIM_TOP - RIM_PROUD))),
              name=f'clipless_{i}', color=cq.Color(0.63, 0.63, 0.63))
    a.add(t.translate((0, 0, RIM_PROUD)), name='tray', color=cq.Color(0.9, 0.45, 0.1))
    for i, f in enumerate(feet()):
        a.add(f.translate((0, 0, RIM_PROUD)), name=f'foot_{i}', color=cq.Color(0.2, 0.5, 0.9))
    a.add(envelope().translate((0, 0, RIM_PROUD)), name='jetson_envelope', color=cq.Color(0.35, 0.35, 0.35))
    return a


def inter(a, b):
    try:
        r = a.intersect(b)
        return sum(s.Volume() for s in r.solids().vals()) if r.solids().size() else 0.0
    except Exception:
        return 0.0


def checks(t):
    fs = feet()
    env = envelope()
    if RETAIN == 'snap':
        from snap_retain import hang, piece_from_above, play_check
        stub = plate_stub(FEET, size=(OL + 20, OW + 20))
        pieces = [piece_from_above().translate((x, y, 0)) for (x, y) in FEET]
        at_stop, at_bump, play = play_check(t, fs, FEET, 'x', inter)
        res = {
            'solids': len(t.val().Solids()),
            'tray_x_feet': sum(inter(t, f) for f in fs),
            'tray_x_feet_at_stop': at_stop,
            'tray_x_feet_at_bump': at_bump,
            'fore_aft_play_between_stop_and_bump': play,
            'feet_x_clipless': sum(inter(hang(f), p) for f in fs for p in pieces),
            'tray_x_plate': inter(hang(t), stub),
            'tray_x_envelope': inter(t, env),
            'feet_x_envelope': sum(inter(f, env) for f in fs),
            'hanging_depth_below_plate_top': round(-hang(env).val().BoundingBox().zmin, 2),
        }
    else:
        stub = plate_stub(FEET, size=(OL + 20, OW + 20)).translate((0, 0, -RIM_PROUD))
        pieces = [clipless_piece().translate((x, y, -RIM_TOP)) for (x, y) in FEET]
        res = {
            'solids': len(t.val().Solids()),
            'tray_x_feet': sum(inter(t, f) for f in fs),
            'feet_x_clipless': sum(inter(f, p) for f in fs for p in pieces),
            'tray_x_plate': inter(t, stub),
            'tray_x_envelope': inter(t, env),
            'feet_x_envelope': sum(inter(f, env) for f in fs),
        }
    # geometric sanity
    res['boss_top_z'] = Z_BOARD
    res['lip_underside_z'] = Z_LIP
    res['pocket'] = (IX, IY)
    return res


def lines_png(shapes, svg_path, png_path, eye=(0.57, -0.82, 0.47)):
    """Hidden-line SVG via the OCC HLR exporter, rasterised with cairosvg.

    projectionDir is the eye direction, but the exporter's SVG comes out rotated 180
    degrees (checked against render.py on an asymmetric test part), so the PNG is
    rotated back. eye=(0.57, -0.82, 0.47) is the same viewpoint as render()'s (28, -55).
    """
    comp = cq.Compound.makeCompound([s.val() if hasattr(s, 'val') else s for s in shapes])
    cq.exporters.export(comp, svg_path, opt={
        'width': 1400, 'height': 900, 'marginLeft': 40, 'marginTop': 40,
        'showAxes': False, 'projectionDir': eye, 'strokeWidth': 0.6,
        'strokeColor': (0, 0, 0), 'hiddenColor': (170, 170, 170), 'showHidden': True})
    import cairosvg
    from PIL import Image
    cairosvg.svg2png(url=svg_path, write_to=png_path, background_color='white')
    Image.open(png_path).rotate(180).save(png_path)


if __name__ == '__main__':
    t = tray()
    bb = t.val().BoundingBox()
    N = 'jetson_orin_nano' + SUFFIX
    cq.exporters.export(t, os.path.join(OUT, N + '.step'))
    cq.exporters.export(t, os.path.join(OUT, N + '.stl'), tolerance=0.02, angularTolerance=0.1)
    a = assembly(t)
    a.save(os.path.join(OUT, N + '_assembly.step'))
    vol = t.val().Volume()
    print(f'tray {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm, {vol/1000:.1f} cm3 '
          f'(~{vol/1000*1.24*0.6:.0f} g PLA at 60% effective), feet on {PEG_PITCH} x {PITCH_Y} mm, '
          f'retention {RETAIN}, feet {FEET}')
    print('holes (x, y) relative to board centre:', HOLES)
    for k, v in checks(t).items():
        print(f'  {k}: {v}')
    # renders
    render([(t, (0.9, 0.45, 0.1))], os.path.join(OUT, N + '.png'),
           views=[(28, -55), (28, 125)], title=f'Jetson Orin Nano tray ({RETAIN})')
    fs = feet()
    stub = plate_stub(FEET, size=(OL + 20, OW + 20))
    if RETAIN == 'snap':
        from snap_retain import under_shaded
        shaded = under_shaded(t, fs, FEET, envelope(), stub)
        title = 'Jetson Orin Nano tray hanging under the plate (grey = kit envelope)'
    else:
        pieces = [clipless_piece().translate((x, y, -(RIM_TOP - RIM_PROUD))) for (x, y) in FEET]
        up = lambda s: s.translate((0, 0, RIM_PROUD))
        shaded = [(stub, (0.55, 0.55, 0.6))] + [(p, (0.63, 0.63, 0.63)) for p in pieces] + \
                 [(up(t), (0.9, 0.45, 0.1))] + [(up(f), (0.2, 0.5, 0.9)) for f in fs] + \
                 [(up(envelope()), (0.35, 0.35, 0.35))]
        title = 'Jetson Orin Nano tray on clipless (grey = kit envelope)'
    el = -25 if RETAIN == 'snap' else 28       # look up from below at a hanging tray
    render(shaded, os.path.join(OUT, N + '_assembly.png'), views=[(el, -55), (el, 125)], title=title)
    lines_png([s for s, _ in shaded], os.path.join(OUT, N + '_assembly_lines.svg'),
              os.path.join(OUT, N + '_assembly_lines.png'),
              eye=(0.57, -0.82, -0.47) if RETAIN == 'snap' else (0.57, -0.82, 0.47))
    lines_png([t], os.path.join(OUT, N + '_lines.svg'), os.path.join(OUT, N + '_lines.png'))
