"""Clamp-on saddle that carries the Flipsky FSESC 6.7 PRO tray on a side rail of the
Traxxas Slash 2WD tub, off the board (LAYOUT.md, "Final set on v2").

The tray is the one from vesc_fsesc67_mount.py (RETAIN=recess variant, loaded as a private
module copy so board_layout.py's snap variant is not disturbed): 4 mm floor with windows,
two 9 mm pads the case stands on (their foot recesses are filled here, there are no feet),
four M3 bosses on the estimated lid-screw pattern, corner stubs, two snap fingers over the
case top, zip-tie tabs at both ends. Case 95 x 92 x 24.5, 380 g (VESC_FSESC67_MOUNT.md).

Under the tray two 3 mm legs run the full tray length (along the case's W axis, which is
along the car once placed) and straddle the tub's side rail: rail width RAIL_W (12,
ESTIMATE), leg grip RAIL_H (18, ESTIMATE) below the rail top, 0.25 mm a side (RAIL_CLR).
SADDLE_DROP is how far the rail top sits ABOVE the tray floor underside: 0 = the floor
rests on the rail; positive = the tray is dropped over the rail, which comes up through a
slot in the floor and pads under a 3 mm ceiling that the case stands on (max +6, the
case bottom is at 9); negative = the floor is that far above the rail (legs only). The
default is +6, the lowest the tray can go: on the car the case top must pass under the
small-board plate's clipless flanges, 16 mm below the plate (LAYOUT.md, "Final set").
Closure: two 25 mm double-sided (back-to-back) hook-and-loop straps through 26 x 4.5
slots in the floor either side of the legs at two stations 70 mm apart. Each strap goes
down its outboard slot, under the rail's lip, up the inboard slot, and each END is folded
back onto itself under the floor (the case sits on the ceiling, so nothing can close over
the top). That needs a lip with a gap under it; if the tub side is a plain wall the
straps cannot wrap it and the screws below are the retention. Four 3.2 mm holes
(ESC_HOLES, ESTIMATE, optional printed standoffs ESC_STANDOFF) take M3 screws into the
tub's stock ESC bosses once they are measured.

Tray frame (from vesc_fsesc67_mount.py): case centre at the origin, +x the XT90 end, -x
the phase-lead end, button on -y, floor underside at z = 0. The rail's outer face is at
x = RAIL_X_OUT (4.5): the XT90 end is inboard, over the tub, the phase leads leave
outboard toward the motor's terminal end. board_layout.py turns the tray 90 degrees so
+x points to the car's left (+y) and puts the rail on the tub's right wall (y -50).

    python3 vesc_tub_bracket_mount.py
    RAIL_W=10 RAIL_H=15 SADDLE_DROP=0 python3 vesc_tub_bracket_mount.py
"""
import os, sys, json, importlib.util
import cadquery as cq
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from render import render
from jetson_orin_nano_mount import lines_png, box, inter

# ---- the tray, as its own module copy with RETAIN=recess -------------------------------------
_saved = os.environ.get('RETAIN')
os.environ['RETAIN'] = 'recess'
_saved_fx = os.environ.get('FINGER_X')
os.environ.setdefault('FINGER_X', '14.0')   # fingers 4 mm nearer the case centre than the clipless tray's 18:
                                            # on the car they then pass 4.5 mm clear of the Jetson tray's posts
_spec = importlib.util.spec_from_file_location('vesc_fsesc67_mount_recess', os.path.join(HERE, 'vesc_fsesc67_mount.py'))
VESC = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(VESC)
if _saved is None:
    del os.environ['RETAIN']
else:
    os.environ['RETAIN'] = _saved
if _saved_fx is None:
    del os.environ['FINGER_X']            # liteon_45w_brick_mount reads the same name

# ---- saddle parameters (chassis numbers are ESTIMATES until measured) ------------------------
RAIL_W = float(os.environ.get('RAIL_W', 12.0))          # tub side rail width
RAIL_H = float(os.environ.get('RAIL_H', 18.0))          # leg grip below the rail top
SADDLE_DROP = float(os.environ.get('SADDLE_DROP', 6.0))     # rail top above the floor underside (see docstring)
RAIL_X_OUT = float(os.environ.get('RAIL_X_OUT', 4.5))   # rail outer face, tray x
RAIL_CLR = 0.25                                         # per side, snug
LEG_T = 3.0
CEIL_T = 3.0
STRAP_SLOT = (26.0, 4.5)                                # 25 mm hook-and-loop
STRAP_Y = [float(v) for v in os.environ.get('STRAP_Y', '-35,35').split(',')]   # stations along the rail
STRAP_GAP = 1.5                                         # slot off the leg face
ESC_HOLE_D = 3.2
ESC_HOLES = json.loads(os.environ.get('ESC_HOLES', '[[36, -30], [46, -30], [36, 30], [46, 30]]'))   # ESTIMATE
ESC_STANDOFF = float(os.environ.get('ESC_STANDOFF', 0.0))   # printed standoff below the floor, 0 = holes only
ESC_STANDOFF_OD = 8.0
TUB_DEPTH = 28.0                                        # board_layout CHASSIS TUB_DEPTH, for the stub

# MODE=posts: the Slash tub has no rail to clamp. The stock ESC screws down onto two M3
# posts moulded into the tub floor, 38.5 mm apart for the XL-5 layout and 46.0 mm for the
# VXL-3S one. Three independent community mounts measured from their STLs (Printables
# 287721 "lines up with the XL-5": 2 x 3.5 mm, 38.5 apart; 960017 Hobbywing: 2 x 3.0, 38.6;
# 1129058 "uses the VXL-3S holes": 2 x 3.0, 46.0). So this mode drops the legs, the rail
# slot and the strap slots and carries both hole pairs on a solid centre strip; screws go
# down through the tray floor into the posts, the FSESC snaps in on top afterwards.
# HOLE_AXIS is which tray axis the post pair lies along: 'y' = along the car (default).
MODE = os.environ.get('MODE', 'rail')
POST_PITCHES = [float(v) for v in os.environ.get('POST_PITCHES', '38.5,46.0').split(',')]
HOLE_AXIS = os.environ.get('HOLE_AXIS', 'y')
POST_HOLE_D = 3.4
STRIP_W = 12.0
if MODE == 'posts':
    N_MODE = '_posts'
else:
    N_MODE = ''

OUT = os.path.join(HERE, 'out')
os.makedirs(OUT, exist_ok=True)
N = 'vesc_tub_bracket' + N_MODE + os.environ.get('OUT_SUFFIX', '')

OL, OW, PAD_H, PAD_W, PAD_D, FLOOR = VESC.OL, VESC.OW, VESC.PAD_H, VESC.PAD_W, VESC.PAD_D, VESC.FLOOR
X_RAIL0, X_RAIL1 = RAIL_X_OUT, RAIL_X_OUT + RAIL_W               # rail faces, tray x
X_LEG_OUT = (X_RAIL0 - RAIL_CLR - LEG_T, X_RAIL0 - RAIL_CLR)     # outer leg
X_LEG_IN = (X_RAIL1 + RAIL_CLR, X_RAIL1 + RAIL_CLR + LEG_T)      # inner leg
Z_RAIL = SADDLE_DROP                                             # rail top, tray z
Z_LEG0 = Z_RAIL - RAIL_H                                         # leg bottoms
assert Z_RAIL + CEIL_T <= PAD_H + 1e-6, f'SADDLE_DROP {SADDLE_DROP} leaves no {CEIL_T} mm ceiling under the case (max {PAD_H - CEIL_T})'
Y_LEG = OW / 2 - 1.0                                             # legs 1 mm inside the tray outline
SLOT_OUT = (X_LEG_OUT[0] - STRAP_GAP - STRAP_SLOT[1], X_LEG_OUT[0] - STRAP_GAP)
SLOT_IN = (X_LEG_IN[1] + STRAP_GAP, X_LEG_IN[1] + STRAP_GAP + STRAP_SLOT[1])


def post_mount():
    """Tray with both post-hole pairs on a centre strip; no legs, no supports."""
    t = VESC.tray()
    for (fx, _) in VESC.FEET:
        t = t.union(box(fx - PAD_W / 2, fx + PAD_W / 2, -PAD_D / 2, PAD_D / 2, 0, PAD_H).edges('|Z').fillet(2))
    reach = max(POST_PITCHES) / 2 + 6.0
    if HOLE_AXIS == 'y':
        t = t.union(box(-STRIP_W / 2, STRIP_W / 2, -reach, reach, 0, FLOOR))
    else:
        t = t.union(box(-reach, reach, -STRIP_W / 2, STRIP_W / 2, 0, FLOOR))
    # The two patterns are 3.75 mm apart per side; two 3.4 holes that close leave 0.35 mm
    # of web, so each side gets one slot spanning both, and a washer under the screw head.
    lo, hi = min(POST_PITCHES) / 2, max(POST_PITCHES) / 2
    for s_ in (-1, 1):
        c = s_ * (lo + hi) / 2
        L = hi - lo + POST_HOLE_D
        if HOLE_AXIS == 'y':
            slot = cq.Workplane('XY').workplane(offset=-1).center(0.0, c).slot2D(L, POST_HOLE_D, 90).extrude(PAD_H + 2)
        else:
            slot = cq.Workplane('XY').workplane(offset=-1).center(c, 0.0).slot2D(L, POST_HOLE_D, 0).extrude(PAD_H + 2)
        t = t.cut(slot)
    return t


def saddle():
    if MODE == 'posts':
        return post_mount()
    t = VESC.tray()
    # fill the foot recesses in the pads: no feet on the tub
    for (fx, _) in VESC.FEET:
        t = t.union(box(fx - PAD_W / 2, fx + PAD_W / 2, -PAD_D / 2, PAD_D / 2, 0, PAD_H).edges('|Z').fillet(2))
    # legs, full length, from the leg bottoms up into the floor (or to the ceiling when dropped)
    z_top = FLOOR if Z_RAIL <= 0 else Z_RAIL + CEIL_T
    for (xa, xb) in (X_LEG_OUT, X_LEG_IN):
        t = t.union(box(xa, xb, -Y_LEG, Y_LEG, Z_LEG0, z_top))
    if Z_RAIL > 0:
        # dropped: ceiling bridge on the legs (the rail slot is cut last, below)
        t = t.union(box(X_LEG_OUT[0], X_LEG_IN[1], -Y_LEG, Y_LEG, Z_RAIL, Z_RAIL + CEIL_T))
    # strap stations: solid floor patch across the leg band, then the two slots
    for sy in STRAP_Y:
        hl = STRAP_SLOT[0] / 2
        t = t.union(box(SLOT_OUT[0] - 3, SLOT_IN[1] + 3, max(sy - hl - 3, -OW / 2), min(sy + hl + 3, OW / 2), 0, FLOOR))
        for (xa, xb) in (SLOT_OUT, SLOT_IN):
            t = t.cut(box(xa, xb, sy - hl, sy + hl, -1, PAD_H + 1).edges('|Z').fillet(1.5))
    # ESC boss holes (3.2) with floor patches and optional standoffs
    for (hx, hy) in ESC_HOLES:
        t = t.union(box(hx - 4.5, hx + 4.5, hy - 4.5, hy + 4.5, 0, FLOOR))
        if ESC_STANDOFF > 0:
            t = t.union(cq.Workplane('XY').workplane(offset=-ESC_STANDOFF).center(hx, hy)
                        .circle(ESC_STANDOFF_OD / 2).extrude(ESC_STANDOFF))
        t = t.cut(cq.Workplane('XY').workplane(offset=-ESC_STANDOFF - 1).center(hx, hy)
                  .circle(ESC_HOLE_D / 2).extrude(ESC_STANDOFF + PAD_H + 2))
    if Z_RAIL > 0:
        # slot for the rail through floor, pads and patches, up to the ceiling
        t = t.cut(box(X_RAIL0 - RAIL_CLR, X_RAIL1 + RAIL_CLR, -OW, OW, Z_LEG0 - 1, Z_RAIL))
    return t


def envelope():
    return VESC.envelope()


def rail_stub(length=None):
    """A piece of tub wall: the rail (RAIL_W wide) with the wall continuing TUB_DEPTH down."""
    length = length or OW + 40
    return box(X_RAIL0, X_RAIL1, -length / 2, length / 2, Z_RAIL - TUB_DEPTH, Z_RAIL)


def checks(s):
    env, stub = envelope(), rail_stub()
    bb = s.val().BoundingBox()
    if MODE == 'posts':
        # every post hole must be a clean through-hole in solid floor: a 3.4 mm pin dropped
        # through it hits nothing, and a 9 mm washer face around it lands on floor
        holes, clear, bearing = [], [], []
        lo, hi = min(POST_PITCHES) / 2, max(POST_PITCHES) / 2
        for pitch in POST_PITCHES:
            for s_ in (-1, 1):
                hx, hy = (0.0, s_ * pitch / 2) if HOLE_AXIS == 'y' else (s_ * pitch / 2, 0.0)
                holes.append((hx, hy))
                pin = cq.Workplane('XY').workplane(offset=-10).center(hx, hy).circle(POST_HOLE_D / 2 - 0.05).extrude(PAD_H + 20)
                # an M3 washer (7 mm OD) centred on the screw, minus the slot it sits over
                slotw = (hi - lo + POST_HOLE_D + 0.6)
                ring = cq.Workplane('XY').center(hx, hy).circle(3.5).extrude(FLOOR)
                clear.append(inter(s, pin)); bearing.append(round(inter(s, ring) / ring.val().Volume(), 3))
        return {
            'solids': len(s.val().Solids()),
            'mount_x_envelope': inter(s, env),
            'post_holes': holes,
            'pin_through_each_hole_hits': clear,
            'floor_under_each_washer_fraction': bearing,
            'screw_head_room_under_case': PAD_H - FLOOR,
            'outline': (round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)),
        }
    # the legs must grip: rail pushed against either leg
    push = RAIL_CLR - 0.01
    return {
        'solids': len(s.val().Solids()),
        'saddle_x_envelope': inter(s, env),
        'saddle_x_rail_nominal': inter(s, stub),
        'saddle_x_rail_pushed_out': inter(s, stub.translate((-push, 0, 0))),
        'saddle_x_rail_pushed_in': inter(s, stub.translate((push, 0, 0))),
        'legs_x': (X_LEG_OUT, X_LEG_IN),
        'leg_bottom_below_rail_top': RAIL_H,
        'floor_underside_above_rail_top': -SADDLE_DROP,
        'case_bottom_above_rail_top': PAD_H - SADDLE_DROP,
        'top_above_rail_top': round(bb.zmax - SADDLE_DROP, 2),
        'bottom_below_rail_top': round(SADDLE_DROP - bb.zmin, 2),
        'strap_slots': {'out': SLOT_OUT, 'in': SLOT_IN, 'y': STRAP_Y, 'size': STRAP_SLOT},
        'esc_holes': ESC_HOLES,
        'outline': (round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)),
    }


def assembly(s):
    a = cq.Assembly(name='vesc_tub_bracket_on_rail')
    a.add(rail_stub(), name='tub_rail', color=cq.Color(0.3, 0.55, 0.3))
    a.add(s, name='saddle', color=cq.Color(0.9, 0.45, 0.1))
    a.add(envelope(), name='fsesc67_envelope', color=cq.Color(0.35, 0.35, 0.35))
    return a


if __name__ == '__main__':
    s = saddle()
    bb = s.val().BoundingBox()
    cq.exporters.export(s, os.path.join(OUT, N + '.step'))
    cq.exporters.export(s, os.path.join(OUT, N + '.stl'), tolerance=0.02, angularTolerance=0.1)
    assembly(s).save(os.path.join(OUT, N + '_assembly.step'))
    vol = s.val().Volume()
    print(f'saddle {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm, {vol/1000:.1f} cm3 '
          f'({vol/1000*1.24:.0f} g PLA solid, ~{vol/1000*1.24*0.6:.0f} g at 60% effective), '
          f'rail {RAIL_W} wide, grip {RAIL_H}, SADDLE_DROP {SADDLE_DROP}')
    for k, v in checks(s).items():
        print(f'  {k}: {v}')
    render([(s, (0.9, 0.45, 0.1))], os.path.join(OUT, N + '.png'), views=[(28, -55), (-20, 125)],
           title='FSESC 6.7 PRO tub saddle')
    shaded = [(rail_stub(), (0.3, 0.55, 0.3)), (s, (0.9, 0.45, 0.1)), (envelope(), (0.35, 0.35, 0.35))]
    render(shaded, os.path.join(OUT, N + '_assembly.png'), views=[(28, -55), (28, 125)],
           title='FSESC 6.7 PRO tub saddle on a tub side rail (green = rail, grey = case envelope)')
    lines_png([q for q, _ in shaded], os.path.join(OUT, N + '_assembly_lines.svg'), os.path.join(OUT, N + '_assembly_lines.png'))
    lines_png([s], os.path.join(OUT, N + '_lines.svg'), os.path.join(OUT, N + '_lines.png'))
