"""Snap-in cup for the Jetson Orin Nano Developer Kit's 45 W adapter (Lite-On, marked
NVIDIA45W2520004701, 19 V 2.37 A, folding US prongs on one end face, barrel cable out of
the other) on the clipless mounting system.

Brick body: 85 x 58 x 28 mm is an ESTIMATE (no dimensioned source found, see the MD);
BRICK_L / BRICK_W / BRICK_H are env vars so the measured numbers can go in.

The brick plugs straight into the Omni 20+'s AC outlet, so the cup holds it with the
prong face free: the cup is open at the prong end (-x), the brick overhangs that end by
OVERHANG so its face can reach the pack, and the brick's x position is set by the outlet,
not by the cup (the cable-end wall has X_CLR of play). The cup holds y and z: a 4 mm floor,
two 3 mm side walls to half height and, in a gap in each wall, a 1.6 mm snap finger with
a lip over the brick's top edge; the cable end has a notched wall. Two feet across the car
(foot('y'), pitch PITCH_Y) under a transverse bar that is wider than the cup; the bar sits
FEET_X_OFFSET from the brick centre so the pair lands on the plate column next to the
cradle. Geometry assumed between cup and cradle is in assumed_geometry().

    python3 liteon_45w_brick_mount.py
    BRICK_L=88 BRICK_W=57 BRICK_H=27.5 OUTLET_Z=14 python3 liteon_45w_brick_mount.py
"""
import os, sys
import cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from clipless import (foot, foot_cutout, clipless_piece, plate_stub, RIM_PROUD, RIM_TOP,
                      FOOT_FLANGE, FOOT_FLANGE_T, FOOT_CLR)
from render import render
from jetson_orin_nano_mount import lines_png, box, inter

# ---- component (ESTIMATE, see the MD) -------------------------------------------------
BRICK_L = float(os.environ.get('BRICK_L', 85.0))    # x, prong face to cable face
BRICK_W = float(os.environ.get('BRICK_W', 58.0))    # y
BRICK_H = float(os.environ.get('BRICK_H', 28.0))    # z
BRICK_MASS = 180.0                                  # g, ESTIMATE
PRONG_DZ = float(os.environ.get('PRONG_DZ', 0.0))   # prong centre above the brick mid-height
PRONG_L, PRONG_PITCH = 16.0, 12.7                   # NEMA 1-15 blades (UL 498: 12.7 mm apart)
CABLE_D = 3.5

# ---- the cradle next door (omni20_mount.py) and the outlet ---------------------------
OMNI_HALF = 122.0 / 2           # the pack edge with the AC outlet is 61 mm from the pack centre
                                # (122 mm edge; the cradle is assumed turned so it faces +x)
CRADLE_RIM = 61.6 + 3.0         # cradle outer wall 64.6 from the cradle centre (0.6 clr + 3 mm post)
CRADLE_PEG_PITCH = 49.0         # its two feet along x
OUTLET_Z = float(os.environ.get('OUTLET_Z', 13.5))  # outlet centre above the cradle floor top, ESTIMATE
PLUG_GAP = 0.5                  # brick face to pack face when plugged in

# ---- cup ------------------------------------------------------------------------------
CLR = 0.5
X_CLR = 2.0                     # extra play at the cable end: x is fixed by the outlet
FLOOR0 = 4.0
WALL = 3.0
WALL_H = 14.0                   # rigid side walls, above the floor
OVERHANG = 4.0                  # brick face proud of the cup's prong end
FINGER_T, FINGER_W = 1.6, 16.0
FINGER_X = os.environ.get('FINGER_X')   # finger centre relative to the brick centre; default: computed
                                        # below, clear of the foot recesses in the bar (see finger_x())
LIP, LIP_CLR, LIP_T = 0.6, 0.3, 2.0
NOTCH_W = 12.0
BAR_L = 40.0                    # transverse foot bar along x
PEG_PITCH = float(os.environ.get('PEG_PITCH', 49.0))    # plate pitch along the car
PITCH_Y = float(os.environ.get('PITCH_Y', 47.0))
# the brick centre sits OMNI_HALF + PLUG_GAP + BRICK_L/2 from the cradle centre; the cradle's
# feet are at +-CRADLE_PEG_PITCH/2 and the next columns PEG_PITCH apart
_default_off = (CRADLE_PEG_PITCH / 2 + 2 * PEG_PITCH) - (OMNI_HALF + PLUG_GAP + BRICK_L / 2)
FEET_X_OFFSET = float(os.environ.get('FEET_X_OFFSET', round(_default_off, 1)))

# RETAIN='snap': under-board variant (snap_retain.py). The floor thickens to SNAP_FLOOR and
# the two flange recesses in the transverse bar become the dovetail slide channel + snap
# tongues of rplidar_c1_mount.py running ACROSS the car (open at -y); the bar grows to
# 40.5 along x and to the channel's stop wall across.
RETAIN = os.environ.get('RETAIN', 'recess')
SNAP_FLOOR = 7.0 + 1.6
SUFFIX = ('_under' if RETAIN == 'snap' else '') + os.environ.get('OUT_SUFFIX', '')
if RETAIN == 'snap':
    FLOOR0 = SNAP_FLOOR
    BAR_L = 40.5

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out')
os.makedirs(OUT, exist_ok=True)

# floor pad so the prong centre meets the outlet centre (both floors are 4 mm on the same plate)
PAD = max(0.0, OUTLET_Z - (BRICK_H / 2 + PRONG_DZ))
FLOOR = FLOOR0 + PAD
IY = BRICK_W + 2 * CLR
OW = IY + 2 * WALL
X0 = -BRICK_L / 2 + OVERHANG                      # cup's open prong end
X1 = BRICK_L / 2 + CLR + X_CLR + WALL             # outside of the cable-end wall
Z_BRICK = FLOOR
Z_LIP = Z_BRICK + BRICK_H + LIP_CLR
Z_TOP = Z_LIP + LIP_T
FEET = [(FEET_X_OFFSET, -PITCH_Y / 2), (FEET_X_OFFSET, PITCH_Y / 2)]
FINGER_X = float(FINGER_X) if FINGER_X is not None else None
BAR_W = PITCH_Y + FOOT_FLANGE[0] + 2 * FOOT_CLR + 2 * WALL   # 83.5 across: foot('y') flange is 30 along y


def finger_x():
    """The snap fingers rise from the floor, so they must not stand over the foot flange
    recesses (34.5 mm along x, centred on FEET_X_OFFSET): a finger over a recess is cut off
    its floor. Centre the fingers in the longer of the two floor spans beside the recess
    (toward the prong end or toward the cable-end wall), keeping 1 mm to the recess."""
    r0 = FEET_X_OFFSET - (FOOT_FLANGE[1] / 2 + FOOT_CLR) - 1.0
    r1 = FEET_X_OFFSET + (FOOT_FLANGE[1] / 2 + FOOT_CLR) + 1.0
    spans = [(X0 + 1.0, r0), (r1, X1 - WALL - 1.0)]
    spans = [sp for sp in spans if sp[1] - sp[0] >= FINGER_W + 2]
    assert spans, 'no floor span beside the foot recess is long enough for the snap fingers'
    lo, hi = max(spans, key=lambda sp: sp[1] - sp[0])
    return round((lo + hi) / 2, 2)
if RETAIN == 'snap':
    BAR_W = 2 * (PITCH_Y / 2 + FOOT_FLANGE[0] / 2 + 0.75 + 4.0)   # to the channel's stop wall
if FINGER_X is None:
    FINGER_X = finger_x()


def cup():
    c = box(X0, X1, -OW / 2, OW / 2, 0, FLOOR).edges('|Z').fillet(3)
    # transverse bar under the two feet, wider than the cup
    bar = box(FEET_X_OFFSET - BAR_L / 2, FEET_X_OFFSET + BAR_L / 2, -BAR_W / 2, BAR_W / 2, 0, FLOOR)
    c = c.union(bar.edges('|Z').fillet(3))
    # lightening window in the floor ahead of the bar
    wx0, wx1 = X0 + 6, FEET_X_OFFSET - BAR_L / 2 - 4
    if wx1 - wx0 > 10:
        c = c.cut(box(wx0, wx1, -IY / 2 + 8, IY / 2 - 8, -1, FLOOR + 1).edges('|Z').fillet(3))
    # side walls to half height, with a gap where the finger is
    for sy in (-1, 1):
        ya, yb = sorted((sy * IY / 2, sy * (IY / 2 + WALL)))
        c = c.union(box(X0, X1, ya, yb, 0, FLOOR + WALL_H))
        c = c.cut(box(FINGER_X - FINGER_W / 2 - 1.0, FINGER_X + FINGER_W / 2 + 1.0, ya - 1, yb + 1,
                      FLOOR, FLOOR + WALL_H + 1))
        # snap finger, from the floor, lip over the brick's top edge
        fa, fb = sorted((sy * IY / 2, sy * (IY / 2 + FINGER_T)))
        c = c.union(box(FINGER_X - FINGER_W / 2, FINGER_X + FINGER_W / 2, fa, fb, 0, Z_TOP))
        yi = sy * (IY / 2 + 0.01)
        prof = [(yi, Z_LIP), (yi - sy * LIP, Z_LIP + LIP), (yi - sy * LIP, Z_TOP - 0.5), (yi, Z_TOP)]
        c = c.union(cq.Workplane('YZ', origin=(FINGER_X - FINGER_W / 2, 0, 0)).polyline(prof).close()
                    .extrude(FINGER_W))
    # cable-end wall with a notch down to the floor
    c = c.union(box(X1 - WALL, X1, -OW / 2, OW / 2, 0, FLOOR + WALL_H))
    c = c.cut(box(X1 - WALL - 1, X1 + 1, -NOTCH_W / 2, NOTCH_W / 2, FLOOR, FLOOR + WALL_H + 1))
    # feet across the car
    if RETAIN == 'snap':
        from snap_retain import channel_solids
        bar, cuts, adds = channel_solids(FEET, 'y', FLOOR, extent=(-BAR_W / 2, BAR_W / 2))
        c = c.union(bar)
        for q in cuts:
            c = c.cut(q)
        for a in adds:
            c = c.union(a)
        return c
    for (x, y) in FEET:
        c = c.cut(foot_cutout(FLOOR, 'y').translate((x, y, 0)))
        # The foot drops in from above, so nothing may stand over its flange recess. The
        # recess reaches y 26..56 for a foot at y 41 and the 3 mm side wall runs y
        # 29.5..32.5, so the wall bridges straight over the slot and the foot cannot be
        # installed. foot_cutout only reaches FLOOR+1. Open the wall over the footprint.
        c = c.cut(foot_access(FLOOR).translate((x, y, 0)))
    return c


def foot_access(floor_t):
    """Clear column above one foot's flange recess, from the recess floor to well above
    the part. Subtracting this guarantees the foot can be dropped in and pulled out."""
    fx, fy = FOOT_FLANGE[::-1]      # foot('y'): 34 along x, 30 along y
    return (cq.Workplane('XY').workplane(offset=floor_t - FOOT_FLANGE_T)
            .rect(fx + 2 * FOOT_CLR, fy + 2 * FOOT_CLR).extrude(200)
            .edges('|Z').fillet(2 + FOOT_CLR))


def feet():
    if RETAIN == 'snap':
        from snap_retain import snap_feet
        return snap_feet(FEET, 'y')
    return [foot('y').translate((x, y, FLOOR)) for (x, y) in FEET]


def envelope():
    """Brick where it sits, prongs out of the -x face, cable out of +x."""
    z0 = Z_BRICK
    e = (cq.Workplane('XY').workplane(offset=z0).rect(BRICK_L, BRICK_W).extrude(BRICK_H)
         .edges('|Z').fillet(4))
    zc = z0 + BRICK_H / 2 + PRONG_DZ
    for sy in (-1, 1):
        e = e.union(box(-BRICK_L / 2 - PRONG_L, -BRICK_L / 2 + 0.01, sy * PRONG_PITCH / 2 - 0.8,
                        sy * PRONG_PITCH / 2 + 0.8, zc - 3.2, zc + 3.2))
    e = e.union(cq.Workplane('YZ', origin=(BRICK_L / 2 - 0.01, 0, zc)).circle(CABLE_D / 2).extrude(30))
    return e


def extraction(c):
    """Total material standing over the feet's flange recesses. Anything above zero means
    a foot cannot be dropped into that slot, whatever the interference checks say: the
    part is still one solid and still misses the foot, it just has a roof over it."""
    fx, fy = FOOT_FLANGE[::-1]
    total = 0.0
    for (x, y) in FEET:
        # From the flange's top face upward only: the 2 mm ledge the flange lands on is
        # below that and is meant to be there.
        sweep = (cq.Workplane('XY').workplane(offset=FLOOR).rect(fx, fy).extrude(200)
                 .translate((x, y, 0)))
        total += c.val().intersect(sweep.val()).Volume()
    return round(total, 2)


def assumed_geometry():
    """Numbers that tie this cup to omni20_mount.py's cradle, all from the cradle centre."""
    brick_c = OMNI_HALF + PLUG_GAP + BRICK_L / 2
    return {
        'pack_outlet_face_x': OMNI_HALF,
        'cradle_outer_wall_x': CRADLE_RIM,
        'brick_face_x': OMNI_HALF + PLUG_GAP,
        'cup_prong_end_x': OMNI_HALF + PLUG_GAP + OVERHANG,
        'gap_cup_to_cradle_wall': OMNI_HALF + PLUG_GAP + OVERHANG - CRADLE_RIM,
        'brick_centre_x': brick_c,
        'cup_feet_x': brick_c + FEET_X_OFFSET,
        'plate_columns_from_cradle_centre': [CRADLE_PEG_PITCH / 2 + k * PEG_PITCH for k in (0, 1, 2)],
        'outlet_centre_above_plate_rim_tops': FLOOR0 + OUTLET_Z,
        'prong_centre_above_plate_rim_tops': FLOOR + BRICK_H / 2 + PRONG_DZ,
    }


def _pieces(dz):
    return [clipless_piece().translate((x, y, dz)) for (x, y) in FEET]


def _stub():
    cx = (X0 + X1) / 2
    return plate_stub([(x - cx, y) for (x, y) in FEET], size=(X1 - X0 + 40, BAR_W + 20)).translate((cx, 0, 0))


def assembly(c):
    if RETAIN == 'snap':
        from snap_retain import under_assembly
        return under_assembly('liteon_45w_brick_under_clipless', c, feet(), FEET, envelope(), _stub())
    a = cq.Assembly(name='liteon_45w_brick_on_clipless')
    a.add(_stub(), name='baseplate', color=cq.Color(0.55, 0.55, 0.6))
    for i, p in enumerate(_pieces(-(RIM_TOP - RIM_PROUD))):
        a.add(p, name=f'clipless_{i}', color=cq.Color(0.63, 0.63, 0.63))
    a.add(c.translate((0, 0, RIM_PROUD)), name='cup', color=cq.Color(0.9, 0.45, 0.1))
    for i, f in enumerate(feet()):
        a.add(f.translate((0, 0, RIM_PROUD)), name=f'foot_{i}', color=cq.Color(0.2, 0.5, 0.9))
    a.add(envelope().translate((0, 0, RIM_PROUD)), name='brick_envelope', color=cq.Color(0.35, 0.35, 0.35))
    return a


def checks(c):
    fs = feet()
    env = envelope()
    if RETAIN == 'snap':
        from snap_retain import hang, piece_from_above, play_check
        stub = _stub()
        pieces = [piece_from_above().translate((x, y, 0)) for (x, y) in FEET]
        at_stop, at_bump, play = play_check(c, fs, FEET, 'y', inter)
        res = {
            'solids': len(c.val().Solids()),
            'cup_x_feet': sum(inter(c, f) for f in fs),
            'cup_x_feet_at_stop': at_stop,
            'cup_x_feet_at_bump': at_bump,
            'side_play_between_stop_and_bump': play,
            'feet_x_clipless': sum(inter(hang(f), p) for f in fs for p in pieces),
            'cup_x_plate': inter(hang(c), stub),
            'cup_x_envelope': inter(c, env),
            'feet_x_envelope': sum(inter(f, env) for f in fs),
            'hanging_depth_below_plate_top': round(-hang(c).val().BoundingBox().zmin, 2),
            'floor': FLOOR,
            'feet': FEET,
            'bar': (BAR_L, BAR_W),
        }
        return res
    stub = _stub().translate((0, 0, -RIM_PROUD))
    pieces = _pieces(-RIM_TOP)
    res = {
        'solids': len(c.val().Solids()),
        'cup_x_feet': sum(inter(c, f) for f in fs),
        'feet_x_clipless': sum(inter(f, p) for f in fs for p in pieces),
        'cup_x_plate': inter(c, stub),
        'cup_x_envelope': inter(c, env),
        'feet_x_envelope': sum(inter(f, env) for f in fs),
        'material_over_foot_slots': extraction(c),
        'floor': FLOOR,
        'lip_underside_z': Z_LIP,
        'brick_top_z': Z_BRICK + BRICK_H,
        'feet': FEET,
        'bar': (BAR_L, BAR_W),
    }
    res.update(assumed_geometry())
    return res


if __name__ == '__main__':
    c = cup()
    bb = c.val().BoundingBox()
    N = 'liteon_45w_brick' + SUFFIX
    cq.exporters.export(c, os.path.join(OUT, N + '.step'))
    cq.exporters.export(c, os.path.join(OUT, N + '.stl'), tolerance=0.02, angularTolerance=0.1)
    assembly(c).save(os.path.join(OUT, N + '_assembly.step'))
    vol = c.val().Volume()
    print(f'cup {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm, {vol/1000:.1f} cm3 '
          f'({vol/1000*1.24:.0f} g PLA solid, ~{vol/1000*1.24*0.6:.0f} g at 60% effective), '
          f'brick {BRICK_L} x {BRICK_W} x {BRICK_H} (ESTIMATE), feet across at {PITCH_Y}, '
          f'offset {FEET_X_OFFSET} from the brick centre')
    for k, v in checks(c).items():
        print(f'  {k}: {v}')
    render([(c, (0.9, 0.45, 0.1))], os.path.join(OUT, N + '.png'),
           views=[(28, -55), (28, 125)], title=f'Lite-On 45 W brick cup ({RETAIN})')
    fs = feet()
    stub = _stub()
    if RETAIN == 'snap':
        from snap_retain import under_shaded
        shaded = under_shaded(c, fs, FEET, envelope(), stub)
        title, el, eye = '45 W brick cup hanging under the plate (grey = brick envelope)', -25, (0.57, -0.82, -0.47)
    else:
        pieces = _pieces(-(RIM_TOP - RIM_PROUD))
        up = lambda s: s.translate((0, 0, RIM_PROUD))
        shaded = [(stub, (0.55, 0.55, 0.6))] + [(p, (0.63, 0.63, 0.63)) for p in pieces] + \
                 [(up(c), (0.9, 0.45, 0.1))] + [(up(f), (0.2, 0.5, 0.9)) for f in fs] + \
                 [(up(envelope()), (0.35, 0.35, 0.35))]
        title, el, eye = '45 W brick cup on clipless (grey = brick envelope)', 28, (0.57, -0.82, 0.47)
    render(shaded, os.path.join(OUT, N + '_assembly.png'), views=[(el, -55), (el, 125)], title=title)
    lines_png([q for q, _ in shaded], os.path.join(OUT, N + '_assembly_lines.svg'),
              os.path.join(OUT, N + '_assembly_lines.png'), eye=eye)
    lines_png([c], os.path.join(OUT, N + '_lines.svg'), os.path.join(OUT, N + '_lines.png'))
