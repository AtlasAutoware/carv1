"""Whole-board layout: every mount of the set on the REAL Baseplate, above and below, with
the Traxxas Slash 2WD chassis modelled underneath as keep-outs.

    python3 board_layout.py                 # regenerates the under-board variants, then the layout
    UNDER_CLEARANCE=40 python3 board_layout.py
    SKIP_REGEN=1 python3 board_layout.py    # layout only
    PLATE=v2 python3 board_layout.py        # Baseplate v2 (baseplate_v2.py) instead of the STL:
                                            # all nine mounts placed, outputs out/board_layout_v2_*
                                            # (six on clipless, the cable tidy on the plate edge, the
                                            # VESC on a tub saddle, the LiPo frame as the Omni alternative);
                                            # lidar in front on B1+B2, mast behind it on A3+B3, and the
                                            # report includes the lidar's occluded sectors and the
                                            # camera's line of sight over the lidar per pitch step
    PLATE=v2 ATTEMPT=mastB3C3 python3 board_layout.py   # the centred mast pairs, to show why not

Board frame: +x forward, +y left, +z up, origin at the plate's centre, plate top at z=0.
FRONT_END decides which end of templates/Baseplate.stl is the front (see LAYOUT.md).

Placement is the PLACEMENTS table below: one row per mount, with the holes it claims. The
script builds the mounts with their own scripts' builders (env vars set before import,
because those scripts read their parameters at import time), moves them into the board
frame, checks that no hole is claimed twice, that every foot sits in a real hole, and runs
pairwise intersections over every solid (mounts, feet, clipless pieces, component
envelopes, chassis keep-outs, board). Chassis numbers that are not in the Traxxas manual are
ESTIMATES and are parameters (all in the CHASSIS dict, overridable by env var of the same
name).
"""
import os, sys, math, json, subprocess, itertools, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
OUT = os.path.join(HERE, 'out')
os.makedirs(OUT, exist_ok=True)

# ---- layout parameters ------------------------------------------------------------------
PLATE = os.environ.get('PLATE', 'stl')                # 'stl': templates/Baseplate.stl; 'v2': baseplate_v2.py
UNDER_CLEARANCE = float(os.environ.get('UNDER_CLEARANCE', 45.0))   # plate underside to tub rim top
PLATE_T = float(os.environ.get('PLATE_T', 2.0 if PLATE == 'stl' else 3.175))   # v2 is cut from 1/8 in ply
FRONT_END = os.environ.get('FRONT_END', 'xmin')       # which STL end is the car's front
EXTRA_HOLES = int(os.environ.get('EXTRA_HOLES', 1))   # 1: model the four new holes (see LAYOUT.md)
OUTLET_Y = float(os.environ.get('OUTLET_Y', 0.0))     # Omni AC outlet offset along its edge, ESTIMATE
if PLATE == 'stl':
    PITCH_X, PITCH_Y = 50.0, 47.5                      # measured from the STL below (exact)
else:
    import baseplate_v2 as _BV2
    PITCH_X, PITCH_Y = _BV2.PITCH, _BV2.PITCH_Y         # 40 along x, 41 across (see baseplate_v2.py)
SFX = ('' if PLATE == 'stl' else '_v2') + os.environ.get('TAG', '')   # output file suffix (TAG for variants)
if os.environ.get('ATTEMPT', 'final') != 'final':
    SFX += '_' + os.environ['ATTEMPT']

# every mount script reads env at import: set the shared numbers first
os.environ['PEG_PITCH'] = str(PITCH_X)
os.environ['PITCH_Y'] = str(PITCH_Y)
# lidar scan plane: 10 mm over the tallest above-board item, which is the cup's finger tops
# (34.3 + 1.13 = 35.43): deck goes one step up (LIDAR_SCAN_CLEARANCE 45.5 -> Z_DECK 15).
# On the 3.175 plate the mounts sit on the plate (ABOVE_Z 0), fingers 34.3, scan 44.8.
os.environ.setdefault('LIDAR_SCAN_CLEARANCE', '45.5')
if PLATE == 'v2':
    # v2 placement needs: a four-post 100 mm tidy, and the small-board plate at PLATE_OFFSET
    # 10.75 (the most aft the plate's own rim rule allows; with rot 180 a LARGER offset moves
    # the plate AFT: front end 134.25 instead of 136.5 at the default 8.5, away from the front
    # tyre's lock sweep). All parametric, all >= the 3 mm wall rule. (The plinth's BAR_END=3.5
    # of the superseded mast-in-front layout is no longer needed: the plinth is now on
    # columns 1-2 with nothing ahead of its bar; default 5.)
    os.environ.setdefault('L', '100'); os.environ.setdefault('N_POST', '4')
    os.environ.setdefault('EDGE', '1')          # cable tidy: edge variant, bolted through the zip holes
    os.environ.setdefault('PLATE_OFFSET', '10.75')
# camera mast and lidar plinth holes on v2 (LAYOUT.md section 19): lidar in front on B1+B2,
# mast behind it on A3+B3. ATTEMPT=mastB3C3 / mastB4C4 show why the centred mast positions
# the owner asked for first do not fit (Jetson tray under C3, Omni cradle over column 4).
MAST_HOLES = {'mastB3C3': ('B3', 'C3'), 'mastB4C4': ('B4', 'C4')}.get(os.environ.get('ATTEMPT', 'final'), ('A3', 'B3'))
LIDAR_HOLES_V2 = ('B1', 'B2')
# VESC tub saddle placement (v2): tray rear end x. The case's rear face (x0 + 3.5) is then
# 0.25 ahead of the Jetson tray's end at 28.5; the tray's front end (x0 + 99) is 3.75 behind the
# steering linkage model at 128 and 2.25 into the servo model's rear top corner (LAYOUT.md "Final set")
SADDLE_X0 = float(os.environ.get('SADDLE_X0', 25.25))

if not os.environ.get('SKIP_REGEN'):
    # regenerate the under-board variants (and refresh the upright ones at the real pitch)
    regen = [
        ('jetson_orin_nano_mount.py', {'RETAIN': 'snap', 'SNAP_ROWS': '1'}),
        ('vesc_fsesc67_mount.py', {'RETAIN': 'snap'}),
        ('small_board_plate_mount.py', {'RETAIN': 'snap', 'SNAP_FEET': '1'}),
        ('liteon_45w_brick_mount.py', {'RETAIN': 'snap'}),
    ]
    if PLATE == 'v2':
        regen = [('baseplate_v2.py', {}),
                 ('jetson_orin_nano_mount.py', {'RETAIN': 'snap', 'SNAP_ROWS': '1'}),
                 ('cable_tidy_mount.py', {'EDGE': '1'}),
                 ('vesc_tub_bracket_mount.py', {}),
                 ('rplidar_c1_mount.py', {'OUT_SUFFIX': '_v2'}),
                 ('camera_mast_mount.py', {'OUT_SUFFIX': '_v2', 'LIDAR_X': '60', 'LIDAR_Y': '-20.5'}),   # feet at PITCH_Y 41, lidar one column ahead
                 ('small_board_plate_mount.py', {'RETAIN': 'recess', 'OUT_SUFFIX': '_v2'}),
                 ('liteon_45w_brick_mount.py', {'RETAIN': 'recess', 'PITCH_Y': str(2 * PITCH_Y), 'FEET_X_OFFSET': '13.5', 'OUT_SUFFIX': '_v2'}),
                 # these three were missing from the list, so out/omni20_cradle.stl was the
                 # 49 mm pitch part and went to the printer for a 40 mm plate
                 ('omni20_mount.py', {'OUT_SUFFIX': '_v2'}),
                 ('vesc_fsesc67_mount.py', {'RETAIN': 'recess', 'OUT_SUFFIX': '_v2'}),
                 ('lipo_smc_9000_mount.py', {'OUT_SUFFIX': '_v2'})]
        # (FEET_X_OFFSET 13.5 = column 8 behind the brick centre; asserted against the computed value below)
    for script, env in regen:
        e = dict(os.environ); e.update(env)
        print(f'== {script} {env}')
        r = subprocess.run([sys.executable, os.path.join(HERE, script)], env=e, capture_output=True, text=True)
        print('\n'.join(l for l in r.stdout.splitlines() if not l.startswith(' ') or 'x_' in l or 'depth' in l))
        if r.returncode:
            print(r.stderr); sys.exit(1)

import numpy as np
import trimesh
import cadquery as cq
from shapely.geometry import Polygon
from clipless import foot, clipless_piece, RIM_TOP, HOLE, FOOT_FLANGE, FOOT_CLR
from render import render

# mount modules: the under-board ones with RETAIN=snap, the rest recess
os.environ['RETAIN'] = 'snap'; os.environ['SNAP_ROWS'] = '1'
os.environ.setdefault('SNAP_FEET', '1')
import jetson_orin_nano_mount as JET
import vesc_fsesc67_mount as VESC
if PLATE == 'stl':
    import small_board_plate_mount as SBP      # hanging, one foot
os.environ['RETAIN'] = 'recess'
if PLATE == 'v2':
    import small_board_plate_mount as SBP      # upright, two feet: it goes above the v2 board
import omni20_mount as OMNI
import rplidar_c1_mount as LIDAR
import camera_mast_mount as MAST
import cable_tidy_mount as TIDY
if PLATE == 'v2':
    import vesc_tub_bracket_mount as VTB       # VESC tray on a tub-rail saddle (loads its own recess copy of VESC)
from jetson_orin_nano_mount import box, inter, lines_png
from snap_retain import hang, piece_from_above, UNDER_Z

ABOVE_Z = max(RIM_TOP - 12.85 - PLATE_T, 0.0)         # mount underside above the plate top (1.13)


# ---- 1. the real baseplate ---------------------------------------------------------------
# read_baseplate() (trimesh section of templates/Baseplate.stl: outline, holes, cutouts,
# clamp holes, clamp blocks) now lives in baseplate_v2.py, which the v2 plate is built from.
from baseplate_v2 import read_baseplate
import baseplate_v2 as BV2

BP = read_baseplate()
CX, CY = BP['centre']


def to_board(x, y):
    """STL (x, y) -> board frame. FRONT_END='xmin': the x=0 end is the front, so board
    x = C - x_stl and board y = C - y_stl (a 180 degree turn, keeps z up)."""
    if FRONT_END == 'xmin':
        return (CX - x, CY - y)
    return (x - CX, y - CY)


if PLATE == 'stl':
    HOLES = sorted(set(tuple(round(v, 3) for v in to_board(*h)) for h in BP['holes']), key=lambda h: (-h[0], -h[1]))
    COLS = sorted(set(h[0] for h in HOLES), reverse=True)          # c1 (front) .. c5 (rear)
    ROWS = sorted(set(h[1] for h in HOLES), reverse=True)          # L, C, R
    ROWN = {ROWS[0]: 'L', ROWS[1]: 'C', ROWS[2]: 'R'}
    HOLE_NAME = {h: f'c{COLS.index(h[0]) + 1}{ROWN[h[1]]}' for h in HOLES}
else:
    # v2: named holes A1..D8 from baseplate_v2 (rows A..D = car left..right, columns 1..6, 8)
    HOLE_NAME = {v: k for k, v in BV2.HOLES.items()}
    HOLES = list(HOLE_NAME)
    COLS = [BV2.COLS[c] for c in sorted(BV2.COLS)]
    ROWS = [BV2.ROWS[r] for r in 'ABCD']
CUTOUTS = [(to_board(*c), w, h) for (c, w, h) in BP['cutouts']]
SMALL = [(to_board(*c), d) for (c, d) in BP['small']]
BRACKETS = []
for (xr, yr, zr) in BP['brackets']:
    (bx0, by0), (bx1, by1) = to_board(xr[0], yr[0]), to_board(xr[1], yr[1])
    BRACKETS.append((min(bx0, bx1), max(bx0, bx1), min(by0, by1), max(by0, by1), zr))
L_PLATE, W_PLATE = BP['size']
XL, XR = -L_PLATE / 2, L_PLATE / 2       # plate x extent (centred on the outline)
YL, YR = -W_PLATE / 2, W_PLATE / 2

# new holes this layout needs (see LAYOUT.md): cup pair behind the cradle, mast pair ahead
# of c1. Named n1.. ; EXTRA_HOLES=0 leaves them out (and the cup and mast then have no holes).
# (STL plate only; v2 has no extra holes.)
NEW_HOLES = {}
if EXTRA_HOLES and PLATE == 'stl':
    yC = ROWS[1]
    NEW_HOLES = {'n1L': (143.0, yC + PITCH_Y), 'n1C': (143.0, yC), 'n1R': (143.0, yC - PITCH_Y),
                 'n6L': (-180.0, yC + OUTLET_Y + PITCH_Y / 2), 'n6R': (-180.0, yC + OUTLET_Y - PITCH_Y / 2)}
ALL_HOLES = {v: k for k, v in HOLE_NAME.items()}
ALL_HOLES.update(NEW_HOLES)
for k, v in list(ALL_HOLES.items()):
    ALL_HOLES[k] = (round(v[0], 3), round(v[1], 3))


# ---- 2. board solids --------------------------------------------------------------------
def board_solid(hole_names):
    """The plate: outline, the 15 holes, the holes in `hole_names` that are new, the four
    cutouts and the four small holes. Top face at z=0. For v2, baseplate_v2.plate_solid()."""
    if PLATE == 'v2':
        return BV2.plate_solid(PLATE_T)
    p = cq.Workplane('XY').rect(L_PLATE, W_PLATE).extrude(-PLATE_T)
    for (x, y) in HOLES + [NEW_HOLES[n] for n in hole_names if n in NEW_HOLES]:
        p = p.cut(box(x - HOLE / 2, x + HOLE / 2, y - HOLE / 2, y + HOLE / 2, -PLATE_T - 1, 1))
    for ((x, y), w, h) in CUTOUTS:
        p = p.cut(box(x - w / 2, x + w / 2, y - h / 2, y + h / 2, -PLATE_T - 1, 1))
    for ((x, y), d) in SMALL:
        p = p.cut(cq.Workplane('XY').workplane(offset=-PLATE_T - 1).center(x, y).circle(d / 2).extrude(PLATE_T + 2))
    return p


def bracket_solids():
    """The clamp blocks under the plate from the STL, hung from the plate underside."""
    out = []
    for (x0, x1, y0, y1, (z0, z1)) in BRACKETS:
        out.append(box(x0, x1, y0, y1, -PLATE_T + z0, -PLATE_T + z1))
    return out


# ---- 3. chassis keep-outs (Traxxas Slash 2WD 58034) ---------------------------------------
# Manual (traxxas.com/media/productattach/C-58234-8/2/58234-8-om-en-r00.pdf): battery
# hold-down 25 mm / 23 mm sides, Titan 12T 550 motor, XL-5 ESC 1.23 x 2.18 x 0.75 in.
# Product specs (traxxas.com, 58034 Slash 2WD): wheelbase 335 mm, length 568, width 296,
# height 214, ground clearance 89 mm (3.5 in), 2.16 kg. Everything else is an ESTIMATE.
CHASSIS = dict(
    WHEELBASE=335.0, TRACK=296.0,
    FRONT_TOWER_X=None, REAR_TOWER_X=None,        # from the STL bracket centres (see below)
    FRONT_AXLE_OFF=10.0,                          # front axle ahead of the front tower, ESTIMATE
    REAR_AXLE_OFF=-4.0,                           # rear axle behind the rear tower, ESTIMATE
    TUB_L=(-150.0, 150.0), TUB_W=100.0, TUB_DEPTH=28.0, TUB_WALL=3.0,
    TOWER_T=4.0, TOWER_W=120.0, TOWER_TOP=-16.0,  # tower top edge below the plate underside (clamp grip)
    POST_D=7.0, POST_ABOVE=25.0,                  # body posts through the cutouts, above the plate top
    POSTS='both' if PLATE == 'stl' else 'front',  # which body posts are fitted ('both', 'front', 'none');
                                                  # v2: the rear pair comes off (the Lite-On cup is where they were)
    SHOCK_Y=42.0, SHOCK_CAP_D=20.0, SHOCK_LEN=90.0,
    BATT=(-120.0, 45.0, 3.0, 50.0, 6.0),          # x0, x1, y0, y1, hold-down height above the rim;
                                                  # 165 x 50 mm bay (traxxas.com 58034 "battery compartment
                                                  # 165 x 50 x 23 mm"), front end and side are ESTIMATES
    GEARBOX=(-165.0, -100.0, -30.0, 22.0, 20.0),  # x0, x1, y0, y1, top above the rim (spur cover;
                                                  # must be under the plate's clamp brackets, which reach 24 mm down)
    MOTOR_X=-120.0, MOTOR_SIDE=-1, MOTOR_Y_IN=-22.0, MOTOR_D=36.0, MOTOR_L=60.0, MOTOR_AXIS_Z=-2.0,
    SERVO=(122.0, 142.0, -20.0, 20.0, 2.0),       # transverse standard servo, top above the rim
    LINKAGE=(128.0, 175.0, -40.0, 40.0, 15.0),    # bellcrank / servo saver / drag link sweep
    TYRE_D=110.0, TYRE_W=42.0, TYRE_TOP_ABOVE_RIM=34.0, BUMP=30.0, LOCK_DEG=30.0,
)
for k in CHASSIS:
    if k in os.environ:
        if isinstance(CHASSIS[k], str):
            CHASSIS[k] = os.environ[k]
        else:
            CHASSIS[k] = type(CHASSIS[k])(json.loads(os.environ[k])) if not isinstance(CHASSIS[k], tuple) \
                else tuple(json.loads(os.environ[k]))
# the towers are where the STL brackets clamp
fb = [b for b in BRACKETS if (b[0] + b[1]) / 2 > 0][0]
rb = [b for b in BRACKETS if (b[0] + b[1]) / 2 < 0][0]
CHASSIS['FRONT_TOWER_X'] = (fb[0] + fb[1]) / 2
CHASSIS['REAR_TOWER_X'] = (rb[0] + rb[1]) / 2
Z_RIM = -PLATE_T - UNDER_CLEARANCE
FRONT_AXLE = CHASSIS['FRONT_TOWER_X'] + CHASSIS['FRONT_AXLE_OFF']
REAR_AXLE = CHASSIS['REAR_TOWER_X'] + CHASSIS['REAR_AXLE_OFF']
CHASSIS['WHEELBASE_MODEL'] = FRONT_AXLE - REAR_AXLE


def chassis_solids():
    C = CHASSIS
    S = {}
    x0, x1 = C['TUB_L']; w = C['TUB_W']; d = C['TUB_DEPTH']; t = C['TUB_WALL']
    tub = box(x0, x1, -w / 2, w / 2, Z_RIM - d, Z_RIM)
    tub = tub.cut(box(x0 + t, x1 - t, -w / 2 + t, w / 2 - t, Z_RIM - d + t, Z_RIM + 1))
    S['tub'] = tub
    bx0, bx1, by0, by1, bh = C['BATT']
    S['battery_tray'] = box(bx0, bx1, by0, by1, Z_RIM - d + t, Z_RIM + bh)
    gx0, gx1, gy0, gy1, gh = C['GEARBOX']
    S['gearbox_spur'] = box(gx0, gx1, gy0, gy1, Z_RIM - d, Z_RIM + gh)
    # motor: transverse can on the MOTOR_SIDE, pinion end toward the centreline
    yi = C['MOTOR_Y_IN'] * (1 if C['MOTOR_SIDE'] < 0 else -1)
    yo = yi + C['MOTOR_SIDE'] * C['MOTOR_L']
    S['motor'] = cq.Workplane('XY').add(cq.Solid.makeCylinder(
        C['MOTOR_D'] / 2, abs(yo - yi), cq.Vector(C['MOTOR_X'], min(yi, yo), Z_RIM + C['MOTOR_AXIS_Z']), cq.Vector(0, 1, 0)))
    sx0, sx1, sy0, sy1, sh = C['SERVO']
    S['steering_servo'] = box(sx0, sx1, sy0, sy1, Z_RIM - d + t, Z_RIM + sh)
    lx0, lx1, ly0, ly1, lh = C['LINKAGE']
    S['steering_linkage'] = box(lx0, lx1, ly0, ly1, Z_RIM, Z_RIM + lh)
    for name, tx in (('front_tower', C['FRONT_TOWER_X']), ('rear_tower', C['REAR_TOWER_X'])):
        S[name] = box(tx - C['TOWER_T'] / 2, tx + C['TOWER_T'] / 2, -C['TOWER_W'] / 2, C['TOWER_W'] / 2,
                      Z_RIM - d, -PLATE_T + C['TOWER_TOP'])
        for sy in (-1, 1):
            S[f'{name}_shock_{"L" if sy > 0 else "R"}'] = (
                cq.Workplane('XY', origin=(tx, sy * C['SHOCK_Y'], -PLATE_T + C['TOWER_TOP'] - 4))
                .circle(C['SHOCK_CAP_D'] / 2).extrude(-C['SHOCK_LEN']))
    for i, ((x, y), w_, h_) in enumerate(CUTOUTS):
        if C['POSTS'] == 'none' or (C['POSTS'] == 'front' and x < 0) or (C['POSTS'] == 'rear' and x > 0):
            continue
        S[f'body_post_{i}'] = cq.Workplane('XY', origin=(x, y, -PLATE_T + C['TOWER_TOP'])) \
            .circle(C['POST_D'] / 2).extrude(-C['TOWER_TOP'] + PLATE_T + C['POST_ABOVE'])
    # wheels: tyre at ride height swept to full bump; front also swept to full lock
    r, tw = C['TYRE_D'] / 2, C['TYRE_W']
    zc = Z_RIM + C['TYRE_TOP_ABOVE_RIM'] - r
    for name, ax, steer in (('front', FRONT_AXLE, True), ('rear', REAR_AXLE, False)):
        for sy in (-1, 1):
            yc = sy * (C['TRACK'] / 2 - tw / 2)
            def tyre(dz, ang):
                t_ = cq.Workplane('XY').add(cq.Solid.makeCylinder(r, tw, cq.Vector(ax, yc - tw / 2, zc + dz), cq.Vector(0, 1, 0)))
                return t_.rotate((ax, yc, 0), (ax, yc, 1), ang) if ang else t_
            env = tyre(0, 0)
            for dz in (C['BUMP'] / 2, C['BUMP']):
                env = env.union(tyre(dz, 0))
            if steer:
                for ang in (-C['LOCK_DEG'], C['LOCK_DEG']):
                    for dz in (0, C['BUMP']):
                        env = env.union(tyre(dz, ang))
            S[f'tyre_{name}_{"L" if sy > 0 else "R"}'] = env
    return S


# ---- 4. mounts -----------------------------------------------------------------------------
def rotz(s, deg):
    return s.rotate((0, 0, 0), (0, 0, 1), deg)


def place_above(s, cx, cy, deg=0.0):
    return rotz(s, deg).translate((cx, cy, ABOVE_Z))


def place_below(s, cx, cy, deg=0.0):
    # hang() puts the contact face at UNDER_Z (-3.13, the rim top of a piece inserted from
    # above through a 2 mm plate); on a thicker plate the rim ends inside the plate and the
    # mount rests on the plate underside instead, PLATE_T - 3.13 lower.
    return hang(rotz(s, deg), dz=-max(PLATE_T + UNDER_Z, 0.0)).translate((cx, cy, 0))


def piece_above(x, y):
    return clipless_piece().translate((x, y, -PLATE_T - 12.85))


def place_tub(s, cx, cy, deg=0.0):
    """A part built in a tray frame whose rail-top line is at z = VTB.SADDLE_DROP, fixed to the
    tub: the rail top goes to Z_RIM (it moves with UNDER_CLEARANCE, the plate does not)."""
    return rotz(s, deg).translate((cx, cy, Z_RIM - VTB.SADDLE_DROP))


# the cup builder needs FEET_X_OFFSET before import: brick behind the cradle on the new pair
# (STL: n6L/n6R; v2: column 8, feet 80 mm apart on rows A and C so the cup centre sits on
# row B, the cradle's row, with the outlet at the pack's mid-edge)
os.environ['RETAIN'] = 'recess'
if PLATE == 'stl':
    _cradle_x = (ALL_HOLES['c4C'][0] + ALL_HOLES['c5C'][0]) / 2
    _cup_col_x = NEW_HOLES.get('n6L', (-180.0, 0))[0]
else:
    _cradle_x = (ALL_HOLES['B5'][0] + ALL_HOLES['B6'][0]) / 2
    _cup_col_x = ALL_HOLES['B8'][0]
    # which column-8 pair is centred on the cup: rows are 40 apart, so the cup centre must be
    # on a row (pitch 80) or halfway between two (pitch 40)
    CUP_Y = ALL_HOLES['B5'][1] + OUTLET_Y
    _pairs = {('A8', 'C8'): 2 * PITCH_Y, ('B8', 'D8'): 2 * PITCH_Y, ('A8', 'B8'): PITCH_Y,
              ('B8', 'C8'): PITCH_Y, ('C8', 'D8'): PITCH_Y}
    _fit = [(p, pitch) for p, pitch in _pairs.items()
            if abs((ALL_HOLES[p[0]][1] + ALL_HOLES[p[1]][1]) / 2 - CUP_Y) < 1e-6]
    assert _fit, f'no column-8 row pair is centred on the cup at y={CUP_Y} (OUTLET_Y={OUTLET_Y}); use a multiple of PITCH_Y/2'
    CUP_ROWS, _cup_pitch = _fit[0]
    os.environ['PITCH_Y'] = str(_cup_pitch)
_brick_c = _cradle_x - (OMNI.OMNI[0] / 2 + 0.5 + 85.0 / 2)
os.environ['FEET_X_OFFSET'] = str(round(-(_cup_col_x - _brick_c), 2))
import liteon_45w_brick_mount as LITE
os.environ['PITCH_Y'] = str(PITCH_Y)


ATTEMPT = os.environ.get('ATTEMPT', 'final')   # 'final', or 'jetson' / 'vesc' / 'tidy' to
                                                # show why those do not fit (see LAYOUT.md);
                                                # v2: 'final' or 'vesc' (see build_mounts_v2)


def build_mounts_v2():
    """Placement on Baseplate v2 (LAYOUT.md section 19). Same dict format as
    build_mounts(). Above: lidar plinth in FRONT on B1+B2, mast behind it on A3+B3, cradle on
    B5+B6, cup on A8+C8, small-board plate on D1+D2. Below: Jetson tray on D4+D5. Off the
    grid: cable tidy on the plate edge, VESC on the tub saddle. ATTEMPT=tidy / vesc adds the
    clipless cable tidy (C1+C2 below) / VESC tray (C2+C3 below) where they come closest;
    ATTEMPT=mastB3C3 / mastB4C4 put the mast on the centred pairs, to show the numbers."""
    H = ALL_HOLES
    M = {}
    mid = lambda a, b: ((H[a][0] + H[b][0]) / 2, (H[a][1] + H[b][1]) / 2)

    # lidar plinth: FRONT, row B, columns 1 and 2 (feet along x), connector notch aft, so
    # nothing on the board crosses its scan plane in the forward half. Body x 63.95..126.05,
    # y -10.55..51.55; the front cutouts start at x 160, the front posts are 25 mm tall (scan
    # plane 44.8), the tyres at |y| > 81.
    cx, cy = mid(*LIDAR_HOLES_V2)
    M['rplidar_c1'] = dict(side='above', holes=list(LIDAR_HOLES_V2), centre=(cx, cy), rot=0,
                           parts=[('plinth', place_above(LIDAR.plinth(), cx, cy))],
                           feet=[place_above(f, cx, cy) for f in LIDAR.feet()],
                           env=[('lidar', place_above(LIDAR.envelope(), cx, cy))])
    # camera mast: BEHIND the lidar on column 3, feet across on rows A and B (centre (35, 41)):
    # its bar (x 14.75..55.25, y 0.75..81) stops 0.25 mm behind the plinth's bar and its four
    # legs cross the scan plane 47 to 91 mm behind the lidar axis, all in the rear half. The
    # centred pairs do not work: a from-below piece at C3 has its flange under the plate where
    # the Jetson tray's front-inboard corner is (x 18.5..28.5, y -37..-18.5), and a mast on
    # column 4 has its bar 27 mm into the Omni cradle (front at x 2.1). Camera forward,
    # channel open to +y (the mast slides on from the left edge).
    cx, cy = mid(*MAST_HOLES)
    m, sd = MAST.mast(), MAST.saddle()
    M['camera_mast'] = dict(side='above', holes=list(MAST_HOLES), centre=(cx, cy), rot=0,
                            parts=[('mast', place_above(m, cx, cy)), ('head', place_above(sd, cx, cy))],
                            feet=[place_above(f, cx, cy) for f in MAST.feet()],
                            env=[('camera', place_above(MAST.envelope(), cx, cy))])
    # Omni cradle: row B, columns 5 and 6, AC outlet edge to the rear
    cx, cy = mid('B5', 'B6')
    pack = (cq.Workplane('XY').rect(*OMNI.OMNI[:2]).extrude(OMNI.OMNI[2]).edges('|Z').fillet(8)
            .translate((0, 0, OMNI.FLOOR)))
    M['omni20'] = dict(side='above', holes=['B5', 'B6'], centre=(cx, cy), rot=0,
                       parts=[('cradle', place_above(OMNI.cradle(), cx, cy))],
                       feet=[place_above(f, cx, cy) for f in OMNI.feet()],
                       env=[('omni20_pack', place_above(pack, cx, cy))])
    # Lite-On cup: column 8, turned 180 so the prongs face the pack. Its centre y is the
    # cradle's row plus OUTLET_Y, so the feet pair (across the car) must be the pair of
    # column-8 rows centred there: CUP_ROWS below (PITCH_Y 40 or 80, set before LITE import).
    bx, by = _brick_c, cy + OUTLET_Y
    assert abs(float(os.environ['FEET_X_OFFSET']) - 13.5) < 1e-6, os.environ['FEET_X_OFFSET']
    assert abs(by - CUP_Y) < 1e-6
    M['liteon_45w'] = dict(side='above', holes=list(CUP_ROWS), centre=(bx, by), rot=180,
                           parts=[('cup', place_above(LITE.cup(), bx, by, 180))],
                           feet=[place_above(f, bx, by, 180) for f in LITE.feet()],
                           env=[('brick', place_above(LITE.envelope(), bx, by, 180))])
    # small-board plate: ABOVE, row D (car right), columns 1 and 2, turned 180 so its long end
    # points aft (PLATE_OFFSET 10.75: x 34.25..134.25, front end behind the front tyre's
    # full-lock sweep; its clipless flanges under the plate end at x 58.5, clear of the Jetson
    # tray; inboard edge y -31.5, 21 mm from the plinth body)
    cx, cy = mid('D1', 'D2')
    pl, _ = SBP.plate()
    M['small_board_plate'] = dict(side='above', holes=['D1', 'D2'], centre=(cx, cy), rot=180,
                                  parts=[('small_board_plate', place_above(pl, cx, cy, 180))],
                                  feet=[place_above(f, cx, cy, 180) for f in SBP.feet()],
                                  env=[('pca9685_hub', place_above(SBP.envelope(), cx, cy, 180))])
    # Jetson tray: BELOW, row D, columns 4 and 5, turned 180 so the connector edge is outboard
    cx, cy = mid('D4', 'D5')
    M['jetson_orin_nano'] = dict(side='below', holes=['D4', 'D5'], centre=(cx, cy), rot=180,
                                 parts=[('jetson_tray', place_below(JET.tray(), cx, cy, 180))],
                                 feet=[place_below(f, cx, cy, 180) for f in JET.feet()],
                                 env=[('jetson_kit', place_below(JET.envelope(), cx, cy, 180))])
    # cable tidy, EDGE variant: no clipless. The rail hangs off the right edge (inboard face
    # 0.25 off y -88.75), 100 long at x -112..-12 (behind the small-board plate, above the
    # Jetson tray's overhang, ahead of the rear tyre's bump sweep), bolted through the zip holes
    # at x -105 and -145 (the ones at -65, -25, 15 have the Jetson tray's floor under them, 55
    # and 95 the small-board plate over them). Rail coordinates: pads at x -43 and -83, plate
    # edge at y +W/2 + 0.25.
    assert TIDY.EDGE, 'v2 layout needs the EDGE=1 cable tidy'
    cx = -105.0 - TIDY.BRACKET_X[0]
    cy = YL - TIDY.EDGE_GAP - TIDY.W / 2
    M['cable_tidy'] = dict(side='above', holes=[], centre=(cx, cy), rot=0,
                           parts=[('rail', place_above(TIDY.rail(), cx, cy))], feet=[],
                           env=[('cables', place_above(TIDY.cables(), cx, cy)),
                                ('m5_fasteners', place_above(TIDY.fasteners(), cx, cy))])
    for bx in TIDY.BRACKET_X:
        assert (round(cx + bx, 3), -BV2.ZIP_Y) in [(round(x, 3), round(y, 3)) for (x, y) in BV2.ZIP_HOLES], \
            f'tidy pad at x {cx + bx} is not on a zip hole'
    # VESC on the tub saddle: tray turned 90 (its +x, the XT90 end, to the car's left), rail on
    # the tub's right wall (outer face y -50), rear end SADDLE_X0 (0.25 ahead of the Jetson tray)
    cy = -CHASSIS['TUB_W'] / 2 - VTB.RAIL_X_OUT
    cx = SADDLE_X0 + VTB.OW / 2
    M['vesc_tub_bracket'] = dict(side='tub', holes=[], centre=(cx, cy), rot=90,
                                 parts=[('saddle', place_tub(VTB.saddle(), cx, cy, 90))], feet=[],
                                 env=[('vesc_case', place_tub(VTB.envelope(), cx, cy, 90))])
    if ATTEMPT == 'tidy':
        # the old clipless tidy (100 mm, four posts) BELOW on C1+C2, lip outboard (turned 180):
        # its pieces on the plate top hit the small-board plate's inboard 6.5 mm (LAYOUT.md 17).
        # Needs EDGE=0 in the environment (the edge rail has no feet).
        cx, cy = mid('C1', 'C2')
        M['cable_tidy_C1C2'] = dict(side='below', holes=['C1', 'C2'], centre=(cx, cy), rot=180,
                                    parts=[('rail', place_below(TIDY.rail(), cx, cy, 180))],
                                    feet=[place_below(f, cx, cy, 180) for f in TIDY.feet()],
                                    env=[('cables', place_below(TIDY.cables(), cx, cy, 180))])
    if ATTEMPT == 'vesc':
        # VESC tray below on C2+C3, phase leads aft: the closest the clipless tray gets, LAYOUT.md 17
        cx, cy = mid('C2', 'C3')
        M['vesc_fsesc67_C2C3'] = dict(side='below', holes=['C2', 'C3'], centre=(cx, cy), rot=180,
                                      parts=[('vesc_tray', place_below(VESC.tray(), cx, cy, 180))],
                                      feet=[place_below(f, cx, cy, 180) for f in VESC.feet()],
                                      env=[('vesc_case', place_below(VESC.envelope(), cx, cy, 180))])
    return M


def build_mounts():
    """Returns {name: dict(parts=[(label, solid)], feet, env, holes, side, centre, rot)}."""
    M = {}
    yC, yL, yR = ROWS[1], ROWS[0], ROWS[2]
    mast_rows = ('n1C', 'n1L') if ATTEMPT != 'jetson' else ('n1C', 'n1R')
    lidar_holes = ('c1R', 'c2R') if ATTEMPT != 'jetson' else ('c1L', 'c2L')

    # -- camera mast, front, feet across on the new pair (channel open toward the L side)
    m, s = MAST.mast(), MAST.saddle()
    cx = 143.0
    cy = (ALL_HOLES[mast_rows[0]][1] + ALL_HOLES[mast_rows[1]][1]) / 2
    rot = 0 if mast_rows[1] == 'n1L' else 180
    M['camera_mast'] = dict(side='above', holes=list(mast_rows), centre=(cx, cy), rot=rot,
                            parts=[('mast', place_above(m, cx, cy, rot)), ('head', place_above(s, cx, cy, rot))],
                            feet=[place_above(f, cx, cy, rot) for f in MAST.feet()],
                            env=[('camera', place_above(MAST.envelope(), cx, cy, rot))])
    if rot:   # a 180 turn would point the camera backwards: turn the mast, not the camera side
        M['camera_mast']['rot'] = 0
        M['camera_mast']['parts'] = [('mast', place_above(m, cx, cy)), ('head', place_above(s, cx, cy))]
        M['camera_mast']['feet'] = [place_above(f, cx, cy) for f in MAST.feet()]
        M['camera_mast']['env'] = [('camera', place_above(MAST.envelope(), cx, cy))]
    # -- lidar plinth, feet along x on the front pair of a side row
    cx = (ALL_HOLES[lidar_holes[0]][0] + ALL_HOLES[lidar_holes[1]][0]) / 2
    cy = ALL_HOLES[lidar_holes[0]][1]
    p = LIDAR.plinth()
    M['rplidar_c1'] = dict(side='above', holes=list(lidar_holes), centre=(cx, cy), rot=0,
                           parts=[('plinth', place_above(p, cx, cy))],
                           feet=[place_above(f, cx, cy) for f in LIDAR.feet()],
                           env=[('lidar', place_above(LIDAR.envelope(), cx, cy))])
    # -- Omni cradle, feet along x on c4C + c5C, AC outlet edge facing the rear
    cx, cy = _cradle_x, yC
    c = OMNI.cradle()
    pack = (cq.Workplane('XY').rect(*OMNI.OMNI[:2]).extrude(OMNI.OMNI[2]).edges('|Z').fillet(8)
            .translate((0, 0, OMNI.FLOOR)))
    M['omni20'] = dict(side='above', holes=['c4C', 'c5C'], centre=(cx, cy), rot=0,
                       parts=[('cradle', place_above(c, cx, cy))],
                       feet=[place_above(f, cx, cy) for f in OMNI.feet()],
                       env=[('omni20_pack', place_above(pack, cx, cy))])
    # -- Lite-On cup behind the cradle on the new pair n6L + n6R, turned 180 so the prongs face +x
    bx, by = _brick_c, yC + OUTLET_Y
    cup = LITE.cup()
    M['liteon_45w'] = dict(side='above', holes=['n6L', 'n6R'], centre=(bx, by), rot=180,
                           parts=[('cup', place_above(cup, bx, by, 180))],
                           feet=[place_above(f, bx, by, 180) for f in LITE.feet()],
                           env=[('brick', place_above(LITE.envelope(), bx, by, 180))])

    if ATTEMPT == 'jetson':
        # Jetson under the board on c2R + c3R (channel open to the rear, connectors inboard)
        cx, cy = (ALL_HOLES['c2R'][0] + ALL_HOLES['c3R'][0]) / 2, yR
        t = JET.tray()
        M['jetson_orin_nano'] = dict(side='below', holes=['c2R', 'c3R'], centre=(cx, cy), rot=0,
                                     parts=[('jetson_tray', place_below(t, cx, cy))],
                                     feet=[place_below(f, cx, cy) for f in JET.feet()],
                                     env=[('jetson_kit', place_below(JET.envelope(), cx, cy))])
    elif ATTEMPT == 'vesc':
        # VESC under the board on c2L + c3L, turned 180 (channel opens forward, phase leads forward)
        cx, cy = (ALL_HOLES['c2L'][0] + ALL_HOLES['c3L'][0]) / 2, yL
        v = VESC.tray()
        M['vesc_fsesc67'] = dict(side='below', holes=['c2L', 'c3L'], centre=(cx, cy), rot=180,
                                 parts=[('vesc_tray', place_below(v, cx, cy, 180))],
                                 feet=[place_below(f, cx, cy, 180) for f in VESC.feet()],
                                 env=[('vesc_case', place_below(VESC.envelope(), cx, cy, 180))])
    elif ATTEMPT == 'tidy':
        # cable tidy under the board on c1L + c2L, lip outboard (turned 180)
        cx, cy = (ALL_HOLES['c1L'][0] + ALL_HOLES['c2L'][0]) / 2, yL
        r = TIDY.rail()
        M['cable_tidy'] = dict(side='below', holes=['c1L', 'c2L'], centre=(cx, cy), rot=180,
                               parts=[('rail', place_below(r, cx, cy, 180))],
                               feet=[place_below(f, cx, cy, 180) for f in TIDY.feet()],
                               env=[('cables', place_below(TIDY.cables(), cx, cy, 180))])
    else:
        # small-board plate under the board on one foot at c2L (SNAP_FEET=1)
        cx, cy = ALL_HOLES['c2L']
        pl, _ = SBP.plate()
        M['small_board_plate'] = dict(side='below', holes=['c2L'], centre=(cx, cy), rot=0,
                                      parts=[('small_board_plate', place_below(pl, cx, cy))],
                                      feet=[place_below(f, cx, cy) for f in SBP.feet()],
                                      env=[('pca9685_hub', place_below(SBP.envelope(), cx, cy))])
    return M


# ---- 5. checks --------------------------------------------------------------------------
def bbox(s):
    b = s.val().BoundingBox()
    return (b.xmin, b.xmax, b.ymin, b.ymax, b.zmin, b.zmax)


def bb_overlap(a, b, tol=0.0):
    return not (a[1] < b[0] - tol or b[1] < a[0] - tol or a[3] < b[2] - tol or b[3] < a[2] - tol
                or a[5] < b[4] - tol or b[5] < a[4] - tol)


def hole_checks(M):
    claims = {}
    problems = []
    for name, m in M.items():
        for h in m['holes']:
            if h not in ALL_HOLES:
                problems.append(f'{name}: hole {h} does not exist')
                continue
            claims.setdefault(h, []).append(name)
        # every foot's peg must be inside a claimed hole (foot bbox in plan inside the hole square)
        for f in m['feet']:
            b = bbox(f)
            fx, fy = (b[0] + b[1]) / 2, (b[2] + b[3]) / 2
            ok = False
            for h in m['holes']:
                hx, hy = ALL_HOLES.get(h, (1e9, 1e9))
                if abs(fx - hx) < 2.5 and abs(fy - hy) < 2.5:
                    ok = True
            if not ok:
                problems.append(f'{name}: foot at ({fx:.2f}, {fy:.2f}) is not in one of its holes {m["holes"]}')
    for h, names in claims.items():
        if len(names) > 1:
            problems.append(f'hole {h} claimed by {names}')
    # every printed part must still be one solid where it is built (a feature cut through a
    # finger's root, say, shows up here and nowhere else)
    for name, m in M.items():
        for lab, s in m['parts']:
            n = len(s.val().Solids())
            if n != 1:
                problems.append(f'{name}:{lab} is {n} solids')
    return claims, problems


def all_solids(M, chassis, board, brackets, used_holes):
    """(label, group, solid) for everything that can collide."""
    S = []
    for name, m in M.items():
        for lab, s in m['parts']:
            S.append((f'{name}:{lab}', 'mount', s))
        for i, f in enumerate(m['feet']):
            S.append((f'{name}:foot{i}', 'foot', f))
        for lab, s in m['env']:
            S.append((f'{name}:{lab}', 'envelope', s))
        for h in m['holes']:
            x, y = ALL_HOLES[h]
            piece = piece_above(x, y) if m['side'] == 'above' else piece_from_above().translate((x, y, 0))
            S.append((f'{name}:clipless_{h}', 'clipless', piece))
    for k, s in chassis.items():
        S.append((f'chassis:{k}', 'chassis', s))
    S.append(('board:plate', 'board', board))
    for i, b in enumerate(brackets):
        S.append((f'board:bracket{i}', 'board', b))
    return S


SKIP_SAME = {frozenset(p) for p in [('foot', 'clipless'), ('mount', 'foot'), ('envelope', 'mount'),
                                     ('envelope', 'foot'), ('envelope', 'clipless')]}


def pairwise(S):
    """Intersection volume of every pair whose bounding boxes overlap, except pairs that
    nest by design inside one mount (part/feet/pockets/envelope), chassis-chassis,
    board-board and the clamp brackets against the towers they clamp."""
    boxes = [bbox(s) for _, _, s in S]
    rows = []
    t0 = time.time()
    for i, j in itertools.combinations(range(len(S)), 2):
        (la, ga, sa), (lb, gb, sb) = S[i], S[j]
        same = la.split(':')[0] == lb.split(':')[0]
        if same and frozenset((ga, gb)) in SKIP_SAME:
            continue
        if {ga, gb} == {'chassis'} or {ga, gb} == {'board'}:
            continue
        if 'bracket' in la + lb and 'tower' in la + lb and 'shock' not in la + lb:
            continue
        if {la, lb} == {'omni20:omni20_pack', 'liteon_45w:brick'}:
            continue        # the brick's prongs are inside the pack's outlet by design (plug engagement)
        if not bb_overlap(boxes[i], boxes[j]):
            continue
        v = inter(sa, sb)
        if v > 1e-3:
            rows.append((la, lb, round(v, 1)))
    return rows, time.time() - t0


def clearance_report(M, chassis):
    """Per under-board mount: the deepest point and its margin to each chassis solid it is
    above, for UNDER_CLEARANCE 45 / 40 / 35 (the chassis solids just move with Z_RIM)."""
    rep = {}
    for name, m in M.items():
        if m['side'] != 'below':
            continue
        solids = [s for _, s in m['parts']] + [s for _, s in m['env']] + m['feet']
        zmin = min(bbox(s)[4] for s in solids)
        rep[name] = {'depth_below_plate_top': round(-zmin, 2)}
        for uc in (45.0, 40.0, 35.0):
            dz = uc - UNDER_CLEARANCE
            worst = None
            for k, cs in chassis.items():
                if k.startswith('tyre') or 'tower' in k or 'post' in k:
                    continue
                cb = bbox(cs)
                cb = (cb[0], cb[1], cb[2], cb[3], cb[4] - dz, cb[5] - dz)
                for s in solids:
                    b = bbox(s)
                    if b[0] < cb[1] and b[1] > cb[0] and b[2] < cb[3] and b[3] > cb[2]:
                        # plan overlap: vertical margin = mount bottom - chassis top
                        margin = b[4] - cb[5]
                        if worst is None or margin < worst[1]:
                            worst = (k, round(margin, 2))
            rep[name][f'UC{int(uc)}'] = worst if worst else ('nothing below', None)
    return rep


def tub_report(M, chassis, board):
    """Per tub-fixed mount: the plate comes down toward it as UNDER_CLEARANCE shrinks. For
    45 / 40 / 35: margin to the plate underside, and the intersection volume with every
    plate-fixed solid (mounts, feet, pieces, envelopes, plate, clamps) after shifting the
    saddle up by the difference."""
    rep = {}
    fixed = []
    for name, m in M.items():
        if m['side'] == 'tub':
            continue
        fixed += [(f'{name}:{l}', s) for l, s in m['parts'] + m['env']] + [(f'{name}:foot{i}', f) for i, f in enumerate(m['feet'])]
        for h in m['holes']:
            x, y = ALL_HOLES[h]
            fixed.append((f'{name}:clipless_{h}', piece_above(x, y) if m['side'] == 'above' else piece_from_above().translate((x, y, 0))))
    fixed.append(('board:plate', board))
    for name, m in M.items():
        if m['side'] != 'tub':
            continue
        solids = [s for _, s in m['parts']] + [s for _, s in m['env']]
        zmax = max(bbox(s)[5] for s in solids)
        rep[name] = {'top_above_rim': round(zmax - Z_RIM, 2), 'bottom_below_rim': round(Z_RIM - min(bbox(s)[4] for s in solids), 2)}
        for uc in (45.0, 40.0, 35.0):
            dz = UNDER_CLEARANCE - uc
            hits = {}
            for lab, fs in fixed:
                fb = bbox(fs)
                for s in solids:
                    b = bbox(s)
                    b = (b[0], b[1], b[2], b[3], b[4] + dz, b[5] + dz)
                    if bb_overlap(b, fb):
                        v = inter(s.translate((0, 0, dz)), fs)
                        if v > 1e-3:
                            hits[lab] = round(hits.get(lab, 0) + v, 1)
            rep[name][f'UC{int(uc)}'] = {'plate_underside_margin': round(-PLATE_T - (zmax + dz), 2), 'hits_mm3': hits}
    return rep


# ---- 6. main -------------------------------------------------------------------------------
if __name__ == '__main__':
    print(f'baseplate STL: {L_PLATE:.1f} x {W_PLATE:.1f} x {BP["thickness"]:.3f} mm (using PLATE_T={PLATE_T}), '
          f'{len(HOLES)} holes, front = STL {FRONT_END} end')
    print('columns x:', COLS, ' rows y:', ROWS)
    print('cutouts (board frame):', [(tuple(round(v, 2) for v in c), round(w, 2), round(h, 2)) for c, w, h in CUTOUTS])
    print('small holes:', [(tuple(round(v, 2) for v in c), round(d, 3)) for c, d in SMALL])
    print('brackets:', [tuple(round(v, 2) for v in b[:4]) + (b[4],) for b in BRACKETS])
    print('new holes:', NEW_HOLES)
    print(f'towers at x = {CHASSIS["FRONT_TOWER_X"]:.1f} / {CHASSIS["REAR_TOWER_X"]:.1f}, axles {FRONT_AXLE:.1f} / '
          f'{REAR_AXLE:.1f} (wheelbase {CHASSIS["WHEELBASE_MODEL"]:.1f} vs spec {CHASSIS["WHEELBASE"]})')

    M = build_mounts() if PLATE == 'stl' else build_mounts_v2()
    claims, problems = hole_checks(M)
    print('\nhole assignment:')
    for name, m in M.items():
        print(f'  {name:20s} {m["side"]:5s} holes {m["holes"]}  centre ({m["centre"][0]:.2f}, {m["centre"][1]:.2f}) rot {m["rot"]}')
    print('unused holes:', [h for h in ALL_HOLES if h not in claims])
    print('problems:', problems or 'none')

    used = [h for m in M.values() for h in m['holes']]
    board = board_solid(used)
    brackets = bracket_solids()
    chassis = chassis_solids()
    S = all_solids(M, chassis, board, brackets, used)
    print(f'\n{len(S)} solids; pairwise intersections (bbox-prefiltered)...')
    rows, dt = pairwise(S)
    print(f'  {len(rows)} intersecting pairs in {dt:.0f} s')
    for la, lb, v in sorted(rows, key=lambda r: -r[2]):
        print(f'  {v:9.1f} mm3  {la}  x  {lb}')

    rep = clearance_report(M, chassis)
    print('\nunder-board clearance (mount bottom minus chassis top, mm, for UNDER_CLEARANCE 45/40/35):')
    for k, v in rep.items():
        print(f'  {k}: {v}')
    trep = tub_report(M, chassis, board)
    if trep:
        print('\ntub-fixed mounts vs the plate and its mounts for UNDER_CLEARANCE 45/40/35:')
        for k, v in trep.items():
            print(f'  {k}: {v}')

    # scan-plane clearance above the board
    scan_z = ABOVE_Z + LIDAR.Z_DECK + LIDAR.LIDAR_SCAN_H
    tallest = []
    for name, m in M.items():
        if m['side'] == 'above' and name != 'camera_mast':
            for lab, s in m['parts'] + m['env']:
                tallest.append((round(bbox(s)[5], 2), f'{name}:{lab}'))
    tallest.sort(reverse=True)
    print(f'\nlidar scan plane {scan_z:.2f} above the board top; tallest above-board items: {tallest[:4]}')

    # what crosses the scan plane, as the lidar sees it: (a) every solid other than the lidar's
    # own, sliced at scan_z, as bearing sectors from the lidar axis; (b) the mast legs
    # analytically (camera_mast_mount.scan_occlusion) as a cross-check
    lidar_c = M['rplidar_c1']['centre']
    occlusion = []
    slab = box(XL - 200, XR + 200, YL - 200, YR + 200, scan_z - 0.05, scan_z + 0.05)
    for lab, g, s in S:
        if lab.startswith('rplidar_c1') or g == 'board':
            continue
        b = bbox(s)
        if b[4] > scan_z or b[5] < scan_z:
            continue
        cut = s.intersect(slab)
        for sol in cut.val().Solids() if cut.val().Volume() > 1e-6 else []:
            vs = [(v.X - lidar_c[0], v.Y - lidar_c[1]) for v in sol.Vertices()]
            angs = [math.degrees(math.atan2(y, x)) for x, y in vs]
            rng = min(math.hypot(x, y) for x, y in vs)
            # angular span (handle the +-180 wrap by working around the mean bearing)
            mean = math.degrees(math.atan2(sum(math.sin(math.radians(a)) for a in angs), sum(math.cos(math.radians(a)) for a in angs)))
            rel = [((a - mean + 180) % 360) - 180 for a in angs]
            occlusion.append(dict(solid=lab, range=round(rng, 1), bearing=round(mean, 1), width=round(max(rel) - min(rel), 1),
                                  sector=(round(mean + min(rel), 1), round(mean + max(rel), 1))))
    occlusion.sort(key=lambda o: o['bearing'])
    fwd = [o for o in occlusion if abs(o['bearing']) - o['width'] / 2 < 90]
    print(f'\nin the scan plane, from the lidar axis {lidar_c} (bearing from dead ahead, +left; C1 datasheet angle = 360 - bearing):')
    for o in occlusion:
        print(f"  {o['solid']:28s} range {o['range']:6.1f}  bearing {o['bearing']:7.1f}  width {o['width']:5.1f}  sector {o['sector']}")
    print('  forward half (-90..+90):', 'CLEAR' if not fwd else fwd)
    # (the mast module's z convention is part z + RIM_PROUD = "above the board top" for a
    # mount sitting on the rim; here the mounts sit ABOVE_Z above the board, so convert)
    mast_c = M['camera_mast']['centre']
    to_mast_z = lambda z_board: z_board - ABOVE_Z + MAST.RIM_PROUD
    rel_lidar = (lidar_c[0] - mast_c[0], lidar_c[1] - mast_c[1])
    legs = MAST.scan_occlusion(rel_lidar, to_mast_z(scan_z))
    print('  (analytic legs:', [(l['leg'], l['range'], l['bearing'], l['width']) for l in legs], ')')

    # camera line of sight over the lidar: pitch steps 0..-20, depth V FOV 65 (RGB 55)
    lidar_top = ABOVE_Z + LIDAR.Z_DECK + LIDAR.LIDAR_BASE_H + LIDAR.LIDAR_HEAD_H
    los = {}
    for fov in (MAST.CAM_VFOV, 55.0):
        los[fov] = MAST.lidar_in_view(rel_lidar, to_mast_z(lidar_top), vfov=fov)
        for r in los[fov]:                       # back to board coordinates for the report
            r['lens_x'] = round(r['lens_x'] + mast_c[0], 1)
            r['lens_z'] = round(r['lens_z'] - MAST.RIM_PROUD + ABOVE_Z, 1)
    print(f'\ncamera over the lidar (lens {mast_c[0] + MAST.lens_xyz(0)[0]:.1f} x, lidar top {lidar_top:.1f}, V FOV {MAST.CAM_VFOV:.0f}):')
    for r in los[MAST.CAM_VFOV]:
        print(f"  pitch {r['pitch']:4d}: lens ({r['lens_x']:.1f}, {r['lens_z']:.1f}), lidar edge dx {r['lidar_edge_dx']}, dz {r['lidar_edge_dz']}, "
              f"elevation {r['lidar_elev']}, image bottom {r['image_bottom']}, lidar {r['margin_below_image']} deg below the image")
    print(f"  lidar top enters the depth image at pitch {los[MAST.CAM_VFOV][0]['enters_at_pitch']}, the RGB image (V 55) at {los[55.0][0]['enters_at_pitch']}")

    # ---- exports ----
    a = cq.Assembly(name='board_layout')
    colours = {'mount': (0.9, 0.45, 0.1), 'foot': (0.2, 0.5, 0.9), 'envelope': (0.35, 0.35, 0.35),
               'clipless': (0.63, 0.63, 0.63), 'chassis': (0.3, 0.55, 0.3), 'board': (0.55, 0.55, 0.6)}
    for lab, g, s in S:
        a.add(s, name=lab.replace(':', '_'), color=cq.Color(*colours[g]))
    a.save(os.path.join(OUT, f'board_layout{SFX}_assembly.step'))
    json.dump({'holes': ALL_HOLES, 'claims': claims, 'placements': {k: {'side': v['side'], 'holes': v['holes'],
              'centre': v['centre'], 'rot': v['rot']} for k, v in M.items()}, 'interference': rows,
               'clearance': rep, 'tub': trep, 'chassis': CHASSIS, 'UNDER_CLEARANCE': UNDER_CLEARANCE,
               'scan_plane_z': scan_z, 'scan_occlusion': occlusion, 'scan_forward_half_clear': not fwd,
               'mast_legs_analytic': legs, 'camera_line_of_sight': {str(k): v for k, v in los.items()}},
              open(os.path.join(OUT, f'board_layout{SFX}_report.json'), 'w'), indent=1, default=str)

    # ---- renders ----
    from PIL import Image, ImageDraw, ImageFont

    def ortho(parts, path, view, labels, title, scale=3.0, pad=30):
        """Orthographic painter's-algorithm render. view: 'top' (x right, y up), 'bottom'
        (seen from below: x right, y down), 'side' (x right, z up, from -y), 'front' (y
        left-to-right reversed, z up, from +x)."""
        import numpy as np
        light = np.array([0.4, -0.6, 0.7]); light /= np.linalg.norm(light)
        proj = {'top': (0, 1, 2, 1, 1, 1), 'bottom': (0, 1, 2, 1, -1, -1),
                'side': (0, 2, 1, 1, 1, -1), 'front': (1, 2, 0, -1, 1, 1)}[view]
        a, b, d, sa, sb, sd = proj
        tris = []
        for obj, col in parts:
            shape = obj.val() if hasattr(obj, 'val') else obj
            behind = 1e4 if (view == 'bottom' and col == colours['chassis']) else 0.0   # chassis under the mounts
            verts, faces = shape.tessellate(0.3, 0.3)
            v = np.array([(q.x, q.y, q.z) for q in verts]); f = np.array(faces)
            if len(f) == 0:
                continue
            P = v[f]
            n = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0]); n /= (np.linalg.norm(n, axis=1, keepdims=True) + 1e-9)
            shade = 0.5 + 0.5 * np.clip(np.abs(n @ light), 0, 1)
            depth = P[:, :, d].mean(1) * sd - behind
            for k in range(len(f)):
                c = tuple(int(255 * min(1, col[i] * shade[k])) for i in range(3))
                tris.append((depth[k], sa * P[k][:, a], sb * P[k][:, b], c))
        tris.sort(key=lambda t: t[0])
        xs = np.concatenate([t[1] for t in tris]); ys = np.concatenate([t[2] for t in tris])
        x0, x1, y0, y1 = xs.min(), xs.max(), ys.min(), ys.max()
        W, H = int((x1 - x0) * scale) + 2 * pad, int((y1 - y0) * scale) + 2 * pad + 40
        im = Image.new('RGB', (W, H), 'white'); dr = ImageDraw.Draw(im)
        tf = lambda x, y: (pad + (x - x0) * scale, 40 + pad + (y1 - y) * scale)
        for _, px, py, c in tris:
            dr.polygon([tf(px[i], py[i]) for i in range(3)], fill=c)
        try:
            font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 13)
            fontb = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 14)
        except Exception:
            font = fontb = ImageFont.load_default()
        dr.text((10, 10), title, fill='black', font=fontb)
        for (x, y, text, colr) in labels:
            X, Y = tf(sa * x, sb * y)
            w = dr.textlength(text, font=font)
            dr.rectangle([X - w / 2 - 3, Y - 9, X + w / 2 + 3, Y + 9], fill=(255, 255, 255))
            dr.text((X - w / 2, Y - 8), text, fill=colr, font=font)
        im.save(path)
        return path

    hole_labels = [(x, y, n, (0, 0, 160)) for n, (x, y) in ALL_HOLES.items() if n in used or n in HOLE_NAME.values()]
    def mount_labels(side):
        L = []
        for name, m in M.items():
            if m['side'] == side or (side == 'below' and m['side'] == 'tub'):
                L.append((m['centre'][0], m['centre'][1], f'{name} [{",".join(m["holes"]) or ("edge, M5 x2" if side == "above" else "tub rail")}]', (140, 40, 0)))
        return L

    above = [(s, colours[g]) for lab, g, s in S if (g == 'board' or 'post' in lab or lab.split(':')[0] in M and M[lab.split(':')[0]]['side'] == 'above')]
    below = [(s, colours[g]) for lab, g, s in S if (g == 'board' or g == 'chassis' or lab.split(':')[0] in M and M[lab.split(':')[0]]['side'] in ('below', 'tub'))]
    ortho(above, os.path.join(OUT, f'board_layout{SFX}_top.png'), 'top', hole_labels + mount_labels('above'),
          f'above the board, +x (front) to the right, +y (left) up; grey = component envelopes, green = body posts')
    ortho(below, os.path.join(OUT, f'board_layout{SFX}_bottom.png'), 'bottom', hole_labels + mount_labels('below'),
          f'below the board (seen from underneath: +x right, car right (-y) up); green = chassis keep-outs at UNDER_CLEARANCE {UNDER_CLEARANCE:.0f}')
    ortho([(s, colours[g]) for lab, g, s in S], os.path.join(OUT, f'board_layout{SFX}_side.png'), 'side', [],
          f'side view from the right (-y), +x to the right; UNDER_CLEARANCE {UNDER_CLEARANCE:.0f} mm')
    ortho([(s, colours[g]) for lab, g, s in S], os.path.join(OUT, f'board_layout{SFX}_front.png'), 'front', [],
          f'front view (from +x), car left on the left')
    lines_png([s for lab, g, s in S if g != 'chassis' or 'post' in lab],
              os.path.join(OUT, f'board_layout{SFX}_lines.svg'), os.path.join(OUT, f'board_layout{SFX}_lines.png'))
    print('\nwritten:', sorted(f for f in os.listdir(OUT) if f.startswith(f'board_layout{SFX}_')))
