"""Whole-board layout of the NEW car (Traxxas Slash 4x4, 6822 chassis, "KA2246-R00") on
Baseplate v2 (baseplate_v2_newcar.py), above and below, with the chassis modelled
underneath as keep-outs (vertical steering servo in the front bulkhead, battery left, motor
right-rear). Same frame and the same checks as ../board_layout.py: +x forward, +y left,
+z up, origin at the plate centre, plate top at z = 0.

    python3 board_layout_newcar.py
    UNDER_CLEARANCE=40 TAG=_uc40 python3 board_layout_newcar.py
    BANK_L=161.8 BANK_MASS=506 TAG=_blade100 python3 board_layout_newcar.py   # Baseus Blade 100 W instead of the Blade HD
    CAM_HEIGHT=150 TAG=_cam150 python3 board_layout_newcar.py

Mounts (PLACEMENTS below): TiM561 plinth above on A1+B1+A2+B2, camera mast above on
A3+B3 directly behind it, Jetson tray (the old set's `_under` snap tray, used upright)
above on D2+D3, power bank tray below on B5+C5+B6+C6, cable tidy on the right edge, VESC
saddle on the chassis's right rail. The reused mounts are built by the parent directory's
scripts, imported with the environment they need; nothing in ../out is rewritten. The
parts this layout needs that are not already in ../out are exported to out/ here.
"""
import os, sys, math, json, itertools, time
HERE = os.path.dirname(os.path.abspath(__file__))
PARENT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, PARENT)
OUT = os.path.join(HERE, 'out')
os.makedirs(OUT, exist_ok=True)

UNDER_CLEARANCE = float(os.environ.get('UNDER_CLEARANCE', 45.0))   # plate underside to chassis rim top, MEASURE
PLATE_T = float(os.environ.get('PLATE_T', 3.175))
_L = os.environ.get('LAYOUT', 'under')
TAG = os.environ.get('TAG', '') + ('' if _L == 'under' else '_above') + ('' if os.environ.get('VESC', 'saddle' if _L == 'under' else 'rear') == ('saddle' if _L == 'under' else 'rear') else '_vesc' + os.environ['VESC'])
CAM_HEIGHT = float(os.environ.get('CAM_HEIGHT', 170.0))            # see the MD: 150 puts the lidar hood in the image at 0 pitch
SADDLE_X0 = float(os.environ.get('SADDLE_X0', 25.25))

import baseplate_v2_newcar as BV2
PITCH_X, PITCH_Y = BV2.PITCH, BV2.PITCH_Y

# shared numbers every mount script reads at import
os.environ['PEG_PITCH'] = str(PITCH_X)
os.environ['PITCH_Y'] = str(PITCH_Y)
os.environ['CAM_HEIGHT'] = str(CAM_HEIGHT)
os.environ.setdefault('L', '100'); os.environ.setdefault('N_POST', '4'); os.environ.setdefault('EDGE', '1')

import numpy as np
import cadquery as cq
from clipless import clipless_piece, RIM_TOP, HOLE
from jetson_orin_nano_mount import box, inter, lines_png
from snap_retain import hang, piece_from_above, UNDER_Z
import tim561_mount as TIM
import battery_bank_underslung as BANK

# lidar geometry the mast module needs (mast on A3+B3 at (35, 41), lidar feet centre (95, 41))
LIDAR_C = ((BV2.HOLES['A1'][0] + BV2.HOLES['A2'][0]) / 2, (BV2.HOLES['A1'][1] + BV2.HOLES['B1'][1]) / 2)
MAST_C = (BV2.HOLES['A3'][0], LIDAR_C[1])
ABOVE_Z = max(RIM_TOP - 12.85 - PLATE_T, 0.0)          # 0 on the 3.175 plate
SCAN_Z = ABOVE_Z + TIM.Z_DECK + TIM.LIDAR_SCAN_H         # 76.46 above the plate top
LIDAR_TOP = ABOVE_Z + TIM.Z_DECK + TIM.LIDAR_H
os.environ['LIDAR_X'] = str(LIDAR_C[0] - MAST_C[0]); os.environ['LIDAR_Y'] = str(LIDAR_C[1] - MAST_C[1])
os.environ['LIDAR_SCAN_Z'] = str(SCAN_Z); os.environ['LIDAR_TOP_Z'] = str(LIDAR_TOP)
import camera_mast_mount as MAST
import importlib.util


def load_copy(script, name, env):
    """A private copy of a parent-directory mount module, imported with its own environment
    (the scripts read their parameters at import time; jetson_orin_nano_mount is already in
    sys.modules as the recess variant because everything imports box/inter from it)."""
    saved = {k: os.environ.get(k) for k in env}
    os.environ.update(env)
    spec = importlib.util.spec_from_file_location(name, os.path.join(PARENT, script))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    for k, v in saved.items():
        if v is None:
            os.environ.pop(k, None)
        else:
            os.environ[k] = v
    return mod


JET = load_copy('jetson_orin_nano_mount.py', 'jetson_under', {'RETAIN': 'snap', 'SNAP_ROWS': '1'})   # the `_under` two-foot snap tray
import cable_tidy_mount as TIDY
import vesc_tub_bracket_mount as VTB
VESC_REAR = load_copy('vesc_fsesc67_mount.py', 'vesc_recess_p41', {'RETAIN': 'recess', 'PEG_PITCH': str(PITCH_Y), 'FINGER_X': '18.0'})
LAYOUT = os.environ.get('LAYOUT', 'under')      # 'under': bank hangs under the plate, Jetson above (the owner's request)
                                                # 'above': bank upright above at the rear, Jetson hangs under (fallback)
VESC = os.environ.get('VESC', 'saddle' if LAYOUT == 'under' else 'rear')   # 'saddle': the old car's tub-rail saddle (front right with
                                                # LAYOUT=under, rear right with 'above'); 'rear': FSESC tray above on B8+C8, turned 90


# ---- chassis keep-outs: Traxxas Slash 4x4 (68086-4 family, 6822 chassis) -------------------
# Manufacturer numbers (traxxas.com specifications via hobbytown.com): wheelbase 324, width
# 296, ground clearance 72, height 193, 2.64 kg. Exploded view 68086-4 page 2 (rcscrapyard.net
# mirror): steering servo 2075 inverted, output shaft DOWN, on the front bulkhead 6830; servo
# saver 6845 below it. Battery lengthwise on the left, ESC/receiver side on the right, centre
# driveshaft down the middle, motor transverse at the rear next to the transmission. Every
# millimetre below that is not the wheelbase/width is an ESTIMATE and a parameter (env var of
# the same name, JSON value). z is relative to the chassis rim top Z_RIM.
CHASSIS = dict(
    WHEELBASE=324.0, TRACK=296.0,
    FRONT_AXLE_OFF=10.0, REAR_AXLE_OFF=-4.0,
    TUB_L=(-150.0, 150.0), TUB_W=100.0, TUB_DEPTH=24.0, TUB_WALL=3.0,   # side wall height; the saddle's legs grip 18 of it
    TOWER_T=4.0, TOWER_W=120.0, TOWER_TOP=-16.0,
    POST_D=7.0, POST_ABOVE=25.0, POSTS='front',    # rear posts off: the bank tray on columns 5-6 covers them on either face
    SHOCK_Y=42.0, SHOCK_CAP_D=20.0, SHOCK_LEN=90.0,
    BATT=(-75.0, 65.0, 15.0, 62.0, 6.0),           # left bay outboard of the shaft tunnel, x0 x1 y0 y1, hold-down above the rim
    GEARBOX=(-160.0, -110.0, -25.0, 25.0, 22.0),   # centre transmission + slipper, top above the rim
    MOTOR_X=-125.0, MOTOR_SIDE=-1, MOTOR_Y_IN=-25.0, MOTOR_D=36.0, MOTOR_L=60.0, MOTOR_AXIS_Z=5.0,
    SHAFT=(-110.0, 128.0, -8.0, 8.0, -8.0),         # centre driveshaft, top below the rim (the saddle floor reaches rim -6)
    # vertical servo in the front bulkhead, right of centre: 20 mm case along x, 56 mm ear span
    # across, case top (its "bottom" face, it is inverted) 25 above the rim
    SERVO=(128.0, 150.0, -53.0, 3.0, 25.0),
    LINKAGE=(140.0, 185.0, -40.0, 40.0, 12.0),      # bellcranks and servo saver, below the servo body
    TYRE_D=110.0, TYRE_W=42.0, TYRE_TOP_ABOVE_RIM=14.0, BUMP=30.0, LOCK_DEG=30.0,
    # tyre top at ride = 110 - 72 ground clearance - 24 tub depth = 14 above the rim
)
for k in CHASSIS:
    if k in os.environ:
        v = json.loads(os.environ[k]) if not isinstance(CHASSIS[k], str) else os.environ[k]
        CHASSIS[k] = tuple(v) if isinstance(CHASSIS[k], tuple) else type(CHASSIS[k])(v)
fb = [b for b in BV2.BRACKETS if (b[0] + b[1]) / 2 > 0][0]
rb = [b for b in BV2.BRACKETS if (b[0] + b[1]) / 2 < 0][0]
CHASSIS['FRONT_TOWER_X'] = (fb[0] + fb[1]) / 2
CHASSIS['REAR_TOWER_X'] = (rb[0] + rb[1]) / 2
Z_RIM = -PLATE_T - UNDER_CLEARANCE
FRONT_AXLE = CHASSIS['FRONT_TOWER_X'] + CHASSIS['FRONT_AXLE_OFF']
REAR_AXLE = CHASSIS['REAR_TOWER_X'] + CHASSIS['REAR_AXLE_OFF']
CHASSIS['WHEELBASE_MODEL'] = FRONT_AXLE - REAR_AXLE
ALL_HOLES = {k: (round(v[0], 3), round(v[1], 3)) for k, v in BV2.HOLES.items()}
XL, XR, YL, YR = -BV2.L_PLATE / 2, BV2.L_PLATE / 2, -BV2.W_PLATE / 2, BV2.W_PLATE / 2


def chassis_solids():
    C = CHASSIS
    S = {}
    x0, x1 = C['TUB_L']; w = C['TUB_W']; d = C['TUB_DEPTH']; t = C['TUB_WALL']
    tub = box(x0, x1, -w / 2, w / 2, Z_RIM - d, Z_RIM)
    S['tub'] = tub.cut(box(x0 + t, x1 - t, -w / 2 + t, w / 2 - t, Z_RIM - d + t, Z_RIM + 1))
    bx0, bx1, by0, by1, bh = C['BATT']
    S['battery_tray'] = box(bx0, bx1, by0, by1, Z_RIM - d + t, Z_RIM + bh)
    gx0, gx1, gy0, gy1, gh = C['GEARBOX']
    S['gearbox'] = box(gx0, gx1, gy0, gy1, Z_RIM - d, Z_RIM + gh)
    yi = C['MOTOR_Y_IN'] * (1 if C['MOTOR_SIDE'] < 0 else -1)
    yo = yi + C['MOTOR_SIDE'] * C['MOTOR_L']
    S['motor'] = cq.Workplane('XY').add(cq.Solid.makeCylinder(
        C['MOTOR_D'] / 2, abs(yo - yi), cq.Vector(C['MOTOR_X'], min(yi, yo), Z_RIM + C['MOTOR_AXIS_Z']), cq.Vector(0, 1, 0)))
    sx0, sx1, sy0, sy1, sh = C['SHAFT']
    S['centre_shaft'] = box(sx0, sx1, sy0, sy1, Z_RIM - d + t, Z_RIM + sh)
    sx0, sx1, sy0, sy1, sh = C['SERVO']
    S['steering_servo'] = box(sx0, sx1, sy0, sy1, Z_RIM - d + t, Z_RIM + sh)
    lx0, lx1, ly0, ly1, lh = C['LINKAGE']
    S['steering_linkage'] = box(lx0, lx1, ly0, ly1, Z_RIM - d, Z_RIM + lh)
    for name, tx in (('front_tower', C['FRONT_TOWER_X']), ('rear_tower', C['REAR_TOWER_X'])):
        S[name] = box(tx - C['TOWER_T'] / 2, tx + C['TOWER_T'] / 2, -C['TOWER_W'] / 2, C['TOWER_W'] / 2,
                      Z_RIM - d, -PLATE_T + C['TOWER_TOP'])
        for sy in (-1, 1):
            S[f'{name}_shock_{"L" if sy > 0 else "R"}'] = (
                cq.Workplane('XY', origin=(tx, sy * C['SHOCK_Y'], -PLATE_T + C['TOWER_TOP'] - 4))
                .circle(C['SHOCK_CAP_D'] / 2).extrude(-C['SHOCK_LEN']))
    for i, ((x, y), w_, h_) in enumerate(BV2.CUTOUTS):
        if C['POSTS'] == 'none' or (C['POSTS'] == 'front' and x < 0) or (C['POSTS'] == 'rear' and x > 0):
            continue
        S[f'body_post_{i}'] = cq.Workplane('XY', origin=(x, y, -PLATE_T + C['TOWER_TOP'])) \
            .circle(C['POST_D'] / 2).extrude(-C['TOWER_TOP'] + PLATE_T + C['POST_ABOVE'])
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


# ---- placement -------------------------------------------------------------------------------
def rotz(s, deg):
    return s.rotate((0, 0, 0), (0, 0, 1), deg)


def place_above(s, cx, cy, deg=0.0):
    return rotz(s, deg).translate((cx, cy, ABOVE_Z))


def place_below(s, cx, cy, deg=0.0):
    return hang(rotz(s, deg), dz=-max(PLATE_T + UNDER_Z, 0.0)).translate((cx, cy, 0))


def piece_above(x, y):
    return clipless_piece().translate((x, y, -PLATE_T - 12.85))


def place_tub(s, cx, cy, deg=0.0):
    return rotz(s, deg).translate((cx, cy, Z_RIM - VTB.SADDLE_DROP))


def build_mounts():
    H = ALL_HOLES
    M = {}
    mid = lambda *names: (sum(H[n][0] for n in names) / len(names), sum(H[n][1] for n in names) / len(names))
    # TiM561: front, four feet on rows A and B, columns 1 and 2, connectors aft
    holes = ['A1', 'B1', 'A2', 'B2']
    cx, cy = mid(*holes)
    M['tim561'] = dict(side='above', holes=holes, centre=(cx, cy), rot=0,
                       parts=[('plinth', place_above(TIM.plinth(), cx, cy))],
                       feet=[place_above(f, cx, cy) for f in TIM.feet()],
                       env=[('lidar', place_above(TIM.envelope(), cx, cy))])
    # camera mast directly behind the lidar on A3+B3, camera forward, channel open to +y
    cx, cy = mid('A3', 'B3')
    m, sd = MAST.mast(), MAST.saddle()
    M['camera_mast'] = dict(side='above', holes=['A3', 'B3'], centre=(cx, cy), rot=0,
                            parts=[('mast', place_above(m, cx, cy)), ('head', place_above(sd, cx, cy))],
                            feet=[place_above(f, cx, cy) for f in MAST.feet()],
                            env=[('camera', place_above(MAST.envelope(), cx, cy))])
    # Jetson: the old set's two-foot snap tray on D2+D3, turned 180 so the connector edge is
    # outboard (right). LAYOUT=under: upright ABOVE the plate (the bank has the underside);
    # LAYOUT=above: hanging BELOW, as on the old car.
    jh = ['D2', 'D3']
    cx, cy = mid(*jh)
    pl = place_above if LAYOUT == 'under' else place_below
    M['jetson_orin_nano'] = dict(side='above' if LAYOUT == 'under' else 'below', holes=jh, centre=(cx, cy), rot=180,
                                 parts=[('jetson_tray', pl(JET.tray(), cx, cy, 180))],
                                 feet=[pl(f, cx, cy, 180) for f in JET.feet()],
                                 env=[('jetson_kit', pl(JET.envelope(), cx, cy, 180))])
    # power bank on columns 5-6, ports forward: hanging (LAYOUT=under) or upright above (LAYOUT=above).
    # Columns 4-5 would clear the transmission, but the mast's A3/B3 pieces have their flanges
    # under the plate out to x 51.5, inside a tray that starts at x 45; columns 5-6 is the only
    # pair whose tray (x -131.45..9.45) has no from-below flange over it
    holes = ['B5', 'C5', 'B6', 'C6']
    cx, cy = mid(*holes)
    pl = place_below if LAYOUT == 'under' else place_above
    M['battery_bank'] = dict(side='below' if LAYOUT == 'under' else 'above', holes=holes, centre=(cx, cy), rot=0,
                             parts=[('bank_tray', pl(BANK.tray(), cx, cy))],
                             feet=[pl(f, cx, cy) for f in BANK.feet()],
                             env=[('power_bank', pl(BANK.envelope(), cx, cy))])
    # cable tidy, edge variant on the right edge, bolted through the zip holes at x -105 and -145
    assert TIDY.EDGE
    cx = -105.0 - TIDY.BRACKET_X[0]
    cy = YL - TIDY.EDGE_GAP - TIDY.W / 2
    M['cable_tidy'] = dict(side='above', holes=[], centre=(cx, cy), rot=0,
                           parts=[('rail', place_above(TIDY.rail(), cx, cy))], feet=[],
                           env=[('cables', place_above(TIDY.cables(), cx, cy)),
                                ('m5_fasteners', place_above(TIDY.fasteners(), cx, cy))])
    for bx in TIDY.BRACKET_X:
        assert (round(cx + bx, 3), -BV2.ZIP_Y) in [(round(x, 3), round(y, 3)) for (x, y) in BV2.ZIP_HOLES]
    if VESC == 'saddle':
        # VESC on the chassis's right rail, as on the old car: ahead of the bank (LAYOUT=under,
        # x 25.25..124.25) or behind the hanging Jetson tray (LAYOUT=above, x -105..-6)
        x0 = float(os.environ.get('SADDLE_X0_ABOVE', -105.0)) if LAYOUT == 'above' else SADDLE_X0
        cy = -CHASSIS['TUB_W'] / 2 - VTB.RAIL_X_OUT
        cx = x0 + VTB.OW / 2
        M['vesc_tub_bracket'] = dict(side='tub', holes=[], centre=(cx, cy), rot=90,
                                     parts=[('saddle', place_tub(VTB.saddle(), cx, cy, 90))], feet=[],
                                     env=[('vesc_case', place_tub(VTB.envelope(), cx, cy, 90))])
    else:
        # VESC tray (recess, two feet at pitch 41) ABOVE on B8+C8, turned 90 so its feet lie
        # across the car: case centred on the rear column, phase leads to the right (-y)
        cx, cy = mid('B8', 'C8')
        M['vesc_fsesc67_rear'] = dict(side='above', holes=['B8', 'C8'], centre=(cx, cy), rot=90,
                                      parts=[('vesc_tray', place_above(VESC_REAR.tray(), cx, cy, 90))],
                                      feet=[place_above(f, cx, cy, 90) for f in VESC_REAR.feet()],
                                      env=[('vesc_case', place_above(VESC_REAR.envelope(), cx, cy, 90))])
    return M


# ---- checks (as ../board_layout.py) -----------------------------------------------------------
def bbox(s):
    b = s.val().BoundingBox()
    return (b.xmin, b.xmax, b.ymin, b.ymax, b.zmin, b.zmax)


def bb_overlap(a, b):
    return not (a[1] < b[0] or b[1] < a[0] or a[3] < b[2] or b[3] < a[2] or a[5] < b[4] or b[5] < a[4])


def hole_checks(M):
    claims, problems = {}, []
    for name, m in M.items():
        for h in m['holes']:
            if h not in ALL_HOLES:
                problems.append(f'{name}: hole {h} does not exist'); continue
            claims.setdefault(h, []).append(name)
        for f in m['feet']:
            b = bbox(f)
            fx, fy = (b[0] + b[1]) / 2, (b[2] + b[3]) / 2
            if not any(abs(fx - ALL_HOLES[h][0]) < 2.5 and abs(fy - ALL_HOLES[h][1]) < 2.5 for h in m['holes'] if h in ALL_HOLES):
                problems.append(f'{name}: foot at ({fx:.2f}, {fy:.2f}) is not in one of its holes {m["holes"]}')
        for lab, s in m['parts']:
            n = len(s.val().Solids())
            if n != 1:
                problems.append(f'{name}:{lab} is {n} solids')
    for h, names in claims.items():
        if len(names) > 1:
            problems.append(f'hole {h} claimed by {names}')
    return claims, problems


def all_solids(M, chassis, board, brackets):
    S = []
    for name, m in M.items():
        S += [(f'{name}:{lab}', 'mount', s) for lab, s in m['parts']]
        S += [(f'{name}:foot{i}', 'foot', f) for i, f in enumerate(m['feet'])]
        S += [(f'{name}:{lab}', 'envelope', s) for lab, s in m['env']]
        for h in m['holes']:
            x, y = ALL_HOLES[h]
            piece = piece_above(x, y) if m['side'] == 'above' else piece_from_above().translate((x, y, 0))
            S.append((f'{name}:clipless_{h}', 'clipless', piece))
    S += [(f'chassis:{k}', 'chassis', s) for k, s in chassis.items()]
    S.append(('board:plate', 'board', board))
    S += [(f'board:bracket{i}', 'board', b) for i, b in enumerate(brackets)]
    return S


SKIP_SAME = {frozenset(p) for p in [('foot', 'clipless'), ('mount', 'foot'), ('envelope', 'mount'),
                                     ('envelope', 'foot'), ('envelope', 'clipless')]}


def pairwise(S):
    boxes = [bbox(s) for _, _, s in S]
    rows, t0 = [], time.time()
    for i, j in itertools.combinations(range(len(S)), 2):
        (la, ga, sa), (lb, gb, sb) = S[i], S[j]
        same = la.split(':')[0] == lb.split(':')[0]
        if same and frozenset((ga, gb)) in SKIP_SAME:
            continue
        if {ga, gb} == {'chassis'} or {ga, gb} == {'board'}:
            continue
        if 'bracket' in la + lb and 'tower' in la + lb and 'shock' not in la + lb:
            continue
        if not bb_overlap(boxes[i], boxes[j]):
            continue
        v = inter(sa, sb)
        if v > 1e-3:
            rows.append((la, lb, round(v, 1)))
    return rows, time.time() - t0


def gaps(S, pairs):
    """Axis-aligned bounding-box gap for named pairs (closest-approach table)."""
    D = {lab: bbox(s) for lab, _, s in S}
    out = {}
    for a, b in pairs:
        if a not in D or b not in D:
            continue
        A, B = D[a], D[b]
        dx = max(B[0] - A[1], A[0] - B[1]); dy = max(B[2] - A[3], A[2] - B[3]); dz = max(B[4] - A[5], A[4] - B[5])
        out[f'{a} | {b}'] = {'dx': round(dx, 2), 'dy': round(dy, 2), 'dz': round(dz, 2), 'gap': round(max(dx, dy, dz), 2)}
    return out


def clearance_report(M, chassis):
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
                cb = bbox(cs); cb = (cb[0], cb[1], cb[2], cb[3], cb[4] - dz, cb[5] - dz)
                for s in solids:
                    b = bbox(s)
                    if b[0] < cb[1] and b[1] > cb[0] and b[2] < cb[3] and b[3] > cb[2]:
                        margin = b[4] - cb[5]
                        if worst is None or margin < worst[1]:
                            worst = (k, round(margin, 2))
            rep[name][f'UC{int(uc)}'] = worst if worst else ('nothing below', None)
    return rep


def tub_report(M, board):
    rep = {}
    fixed = [('board:plate', board)]
    for name, m in M.items():
        if m['side'] == 'tub':
            continue
        fixed += [(f'{name}:{l}', s) for l, s in m['parts'] + m['env']] + [(f'{name}:foot{i}', f) for i, f in enumerate(m['feet'])]
        for h in m['holes']:
            x, y = ALL_HOLES[h]
            fixed.append((f'{name}:clipless_{h}', piece_above(x, y) if m['side'] == 'above' else piece_from_above().translate((x, y, 0))))
    for name, m in M.items():
        if m['side'] != 'tub':
            continue
        solids = [s for _, s in m['parts']] + [s for _, s in m['env']]
        zmax = max(bbox(s)[5] for s in solids)
        rep[name] = {'top_above_rim': round(zmax - Z_RIM, 2)}
        for uc in (45.0, 40.0, 35.0):
            dz = UNDER_CLEARANCE - uc
            hits = {}
            for lab, fs in fixed:
                fb_ = bbox(fs)
                for s in solids:
                    b = bbox(s); b = (b[0], b[1], b[2], b[3], b[4] + dz, b[5] + dz)
                    if bb_overlap(b, fb_):
                        v = inter(s.translate((0, 0, dz)), fs)
                        if v > 1e-3:
                            hits[lab] = round(hits.get(lab, 0) + v, 1)
            rep[name][f'UC{int(uc)}'] = {'plate_underside_margin': round(-PLATE_T - (zmax + dz), 2), 'hits_mm3': hits}
    return rep


def camera_los(cam_height):
    """Pitch at which the TiM561's hood enters the bottom of the depth (V 65) and RGB (V 55)
    images, for the mast at MAST_C and the lidar at LIDAR_C. Candidates: hood top front rim,
    hood base front rim, housing front-top edge. Lens position from the mast module at the
    given CAM_HEIGHT (its lens_xyz moves with the pitch)."""
    xc = LIDAR_C[0] + (TIM.X_REAR + TIM.X_FRONT) / 2
    cands = {'hood_top_front': (xc + TIM.HOOD_R1, LIDAR_TOP),
             'hood_base_front': (xc + TIM.HOOD_R0, ABOVE_Z + TIM.Z_DECK + TIM.LIDAR_BODY_H),
             'housing_front_top': (LIDAR_C[0] + TIM.X_FRONT, ABOVE_Z + TIM.Z_DECK + TIM.LIDAR_BODY_H)}
    z_axis = cam_height - MAST.RIM_PROUD - MAST.Z_PAD - MAST.CAM_LENS_Z
    def lens(p):
        x, z = MAST.X_CAM_FRONT, MAST.Z_PAD + MAST.CAM_LENS_Z
        a = math.radians(p)
        return (MAST_C[0] + x * math.cos(a) + z * math.sin(a), ABOVE_Z + z_axis - x * math.sin(a) + z * math.cos(a))
    out = {}
    for fov in (MAST.CAM_VFOV, 55.0):
        rows = []
        def margin(p):
            lx, lz = lens(p)
            worst = None
            for k, (x, z) in cands.items():
                e = math.degrees(math.atan2(z - lz, x - lx))
                m = (-p - fov / 2) - e
                if worst is None or m < worst[0]:
                    worst = (m, k, e, round(x - lx, 1), round(z - lz, 1))
            return worst
        lo, hi = 0.0, 89.0
        enters = 0.0 if margin(lo)[0] <= 0 else None
        if enters is None and margin(hi)[0] <= 0:
            for _ in range(40):
                mid_ = (lo + hi) / 2
                lo, hi = (mid_, hi) if margin(mid_)[0] > 0 else (lo, mid_)
            enters = round((lo + hi) / 2, 1)
        for p in (0, 5, 10, 15, 20):
            m, k, e, dx, dz = margin(p)
            rows.append(dict(pitch=-p, critical=k, dx=dx, dz=dz, elev=round(e, 1), image_bottom=round(-p - fov / 2, 1), margin=round(m, 1)))
        out[str(fov)] = dict(enters_at_pitch=None if enters is None else -enters, rows=rows)
    return out


# ---- renders (ortho painter's algorithm, as ../board_layout.py) -----------------------------------
COLOURS = {'mount': (0.9, 0.45, 0.1), 'foot': (0.2, 0.5, 0.9), 'envelope': (0.35, 0.35, 0.35),
           'clipless': (0.63, 0.63, 0.63), 'chassis': (0.3, 0.55, 0.3), 'board': (0.55, 0.55, 0.6)}


def ortho(parts, path, view, labels, title, scale=3.0, pad=30):
    from PIL import Image, ImageDraw, ImageFont
    light = np.array([0.4, -0.6, 0.7]); light /= np.linalg.norm(light)
    proj = {'top': (0, 1, 2, 1, 1, 1), 'bottom': (0, 1, 2, 1, -1, -1), 'side': (0, 2, 1, 1, 1, -1), 'front': (1, 2, 0, -1, 1, 1)}[view]
    a, b, d, sa, sb, sd = proj
    tris = []
    for obj, col in parts:
        shape = obj.val() if hasattr(obj, 'val') else obj
        behind = 0.0
        if view == 'bottom':                       # seen from below: plate farthest, chassis behind the mounts
            behind = 2e4 if col == COLOURS['board'] else (1e4 if col == COLOURS['chassis'] else 0.0)
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


if __name__ == '__main__':
    print(f'plate {BV2.L_PLATE:.1f} x {BV2.W_PLATE:.1f} x {PLATE_T}, towers at x {CHASSIS["FRONT_TOWER_X"]:.1f} / {CHASSIS["REAR_TOWER_X"]:.1f}, '
          f'axles {FRONT_AXLE:.1f} / {REAR_AXLE:.1f} (wheelbase {CHASSIS["WHEELBASE_MODEL"]:.1f} vs spec {CHASSIS["WHEELBASE"]}); '
          f'UNDER_CLEARANCE {UNDER_CLEARANCE}, CAM_HEIGHT {CAM_HEIGHT}')
    probs = BV2.check_webs()
    assert not probs, probs
    M = build_mounts()
    claims, problems = hole_checks(M)
    print('\nhole assignment:')
    for name, m in M.items():
        print(f'  {name:18s} {m["side"]:5s} holes {m["holes"]}  centre ({m["centre"][0]:.2f}, {m["centre"][1]:.2f}) rot {m["rot"]}')
    print('unused holes:', [h for h in ALL_HOLES if h not in claims])
    print('problems:', problems or 'none')

    board = BV2.plate_solid(PLATE_T)
    brackets = BV2.bracket_solids(PLATE_T)
    chassis = chassis_solids()
    S = all_solids(M, chassis, board, brackets)
    print(f'\n{len(S)} solids; pairwise intersections (bbox-prefiltered)...')
    rows, dt = pairwise(S)
    print(f'  {len(rows)} intersecting pairs in {dt:.0f} s')
    for la, lb, v in sorted(rows, key=lambda r: -r[2]):
        print(f'  {v:9.1f} mm3  {la}  x  {lb}')
    ext = {lab: [round(v, 2) for v in bbox(s)] for lab, g, s in S if g in ('mount', 'envelope')}
    print('\nextents (x0 x1 y0 y1 z0 z1):')
    for k, v in ext.items():
        print(f'  {k:32s} {v}')
    rep = clearance_report(M, chassis)
    print('\nunder-board clearance (mount bottom minus chassis top, mm, for UNDER_CLEARANCE 45/40/35):')
    for k, v in rep.items():
        print(f'  {k}: {v}')
    trep = tub_report(M, board)
    print('\ntub-fixed mounts vs the plate and its mounts for UNDER_CLEARANCE 45/40/35:')
    for k, v in trep.items():
        print(f'  {k}: {v}')
    close = gaps(S, [('jetson_orin_nano:jetson_tray', 'vesc_tub_bracket:saddle'), ('jetson_orin_nano:jetson_kit', 'chassis:tub'),
                     ('vesc_tub_bracket:saddle', 'chassis:motor'), ('vesc_tub_bracket:saddle', 'battery_bank:clipless_C5'), ('vesc_tub_bracket:saddle', 'battery_bank:clipless_C6'),
                     ('vesc_fsesc67_rear:vesc_tray', 'battery_bank:bank_tray'), ('vesc_fsesc67_rear:vesc_tray', 'chassis:body_post_0'), ('vesc_fsesc67_rear:vesc_tray', 'chassis:body_post_1'),
                     ('vesc_fsesc67_rear:vesc_tray', 'cable_tidy:rail'), ('battery_bank:bank_tray', 'cable_tidy:rail'), ('battery_bank:bank_tray', 'camera_mast:mast'),
                     ('jetson_orin_nano:jetson_kit', 'chassis:centre_shaft'), ('jetson_orin_nano:jetson_tray', 'chassis:steering_servo'),
                     ('tim561:plinth', 'camera_mast:mast'), ('tim561:lidar', 'camera_mast:mast'), ('camera_mast:mast', 'jetson_orin_nano:jetson_tray'),
                     ('jetson_orin_nano:jetson_tray', 'chassis:tyre_front_R'), ('jetson_orin_nano:jetson_kit', 'chassis:tyre_front_R'),
                     ('battery_bank:bank_tray', 'board:bracket2'), ('battery_bank:power_bank', 'board:bracket2'), ('battery_bank:bank_tray', 'board:bracket3'),
                     ('battery_bank:bank_tray', 'vesc_tub_bracket:saddle'), ('battery_bank:bank_tray', 'chassis:rear_tower'),
                     ('battery_bank:bank_tray', 'cable_tidy:m5_fasteners'), ('battery_bank:bank_tray', 'chassis:battery_tray'),
                     ('battery_bank:bank_tray', 'chassis:gearbox'), ('battery_bank:bank_tray', 'chassis:motor'),
                     ('vesc_tub_bracket:saddle', 'chassis:steering_servo'), ('vesc_tub_bracket:saddle', 'jetson_orin_nano:clipless_D2'),
                     ('vesc_tub_bracket:saddle', 'jetson_orin_nano:clipless_D3'), ('vesc_tub_bracket:vesc_case', 'jetson_orin_nano:clipless_D2'),
                     ('tim561:clipless_B1', 'chassis:steering_servo'), ('tim561:clipless_B1', 'chassis:steering_linkage'),
                     ('camera_mast:clipless_B3', 'battery_bank:bank_tray'), ('camera_mast:mast', 'battery_bank:clipless_B5'),
                     ('tim561:lidar', 'chassis:tyre_front_L'), ('tim561:plinth', 'chassis:body_post_2'), ('tim561:plinth', 'chassis:body_post_3'),
                     ('cable_tidy:rail', 'chassis:tyre_rear_R'), ('jetson_orin_nano:jetson_tray', 'tim561:plinth')])
    print('\nclosest approaches (bbox gaps, mm; the largest of dx/dy/dz is the separating direction):')
    for k, v in close.items():
        print(f'  {k:70s} gap {v["gap"]:7.2f}  (dx {v["dx"]}, dy {v["dy"]}, dz {v["dz"]})')

    # scan plane: what crosses it, as bearings from the lidar's scan axis
    lidar_axis = (LIDAR_C[0] + (TIM.X_REAR + TIM.X_FRONT) / 2, LIDAR_C[1])
    slab = box(XL - 250, XR + 250, YL - 250, YR + 250, SCAN_Z - 0.05, SCAN_Z + 0.05)
    occlusion = []
    for lab, g, s in S:
        if lab.startswith('tim561') or g == 'board':
            continue
        b = bbox(s)
        if b[4] > SCAN_Z or b[5] < SCAN_Z:
            continue
        cut = s.intersect(slab)
        for sol in (cut.val().Solids() if cut.val().Volume() > 1e-6 else []):
            vs = [(v.X - lidar_axis[0], v.Y - lidar_axis[1]) for v in sol.Vertices()]
            angs = [math.degrees(math.atan2(y, x)) for x, y in vs]
            rng = min(math.hypot(x, y) for x, y in vs)
            mean = math.degrees(math.atan2(sum(math.sin(math.radians(a)) for a in angs), sum(math.cos(math.radians(a)) for a in angs)))
            rel = [((a - mean + 180) % 360) - 180 for a in angs]
            occlusion.append(dict(solid=lab, range=round(rng, 1), bearing=round(mean, 1), width=round(max(rel) - min(rel), 1),
                                  sector=(round(mean + min(rel), 1), round(mean + max(rel), 1)),
                                  in_blind_sector=abs(abs(mean) - 180) + (max(rel) - min(rel)) / 2 <= 45.0,
                                  in_forward_half=abs(mean) - (max(rel) - min(rel)) / 2 < 90))
    occlusion.sort(key=lambda o: o['bearing'])
    tallest = sorted([(round(bbox(s)[5], 2), lab) for lab, g, s in S if g in ('mount', 'envelope', 'chassis') and not lab.startswith('tim561')], reverse=True)
    print(f'\nlidar scan plane {SCAN_Z:.2f} above the plate top (lidar top {LIDAR_TOP:.2f}); tallest other items: {tallest[:6]}')
    print(f'in the scan plane, from the lidar axis {tuple(round(v, 1) for v in lidar_axis)} (bearing from dead ahead, +left; the TiM561 blind sector is |bearing| > 135):')
    for o in occlusion:
        print(f"  {o['solid']:28s} range {o['range']:6.1f}  bearing {o['bearing']:7.1f}  width {o['width']:5.1f}  sector {o['sector']}  "
              f"{'BLIND (free)' if o['in_blind_sector'] else 'VISIBLE'}")
    visible = [o for o in occlusion if not o['in_blind_sector']]
    print('  visible occlusions (outside the rear 90 degrees):', visible or 'NONE')
    to_mast_z = lambda z: z - ABOVE_Z + MAST.RIM_PROUD
    legs = MAST.scan_occlusion((lidar_axis[0] - MAST_C[0], lidar_axis[1] - MAST_C[1]), to_mast_z(SCAN_Z))
    print('  (analytic legs, 0.75 x 6 mm half-width:', [(l['leg'], l['range'], l['bearing'], l['width']) for l in legs], ')')
    legs_blind = all(abs(abs(l['bearing']) - 180) + l['width'] / 2 <= 45 for l in legs)
    print('  all four legs inside the blind sector (analytic):', legs_blind)

    los = {h: camera_los(h) for h in (150.0, CAM_HEIGHT, 190.0)}
    print(f'\ncamera line of sight over the TiM561 (lens x {MAST_C[0] + MAST.lens_xyz(0)[0]:.1f}, lidar hood top {LIDAR_TOP:.1f}):')
    for h, r in los.items():
        print(f"  CAM_HEIGHT {h:.0f}: hood enters the depth image (V {MAST.CAM_VFOV:.0f}) at pitch {r[str(MAST.CAM_VFOV)]['enters_at_pitch']}, "
              f"the RGB image (V 55) at {r['55.0']['enters_at_pitch']}; at 0 pitch the margin is {r[str(MAST.CAM_VFOV)]['rows'][0]['margin']} deg "
              f"({r[str(MAST.CAM_VFOV)]['rows'][0]['critical']}, dx {r[str(MAST.CAM_VFOV)]['rows'][0]['dx']}, dz {r[str(MAST.CAM_VFOV)]['rows'][0]['dz']})")

    # exports: assembly, report, the reused parts this layout needs that are not in ../out
    a = cq.Assembly(name='newcar_layout')
    for lab, g, s in S:
        a.add(s, name=lab.replace(':', '_'), color=cq.Color(*COLOURS[g]))
    a.save(os.path.join(OUT, f'newcar_layout{TAG}_assembly.step'))
    json.dump({'holes': ALL_HOLES, 'claims': claims, 'placements': {k: {'side': v['side'], 'holes': v['holes'], 'centre': v['centre'], 'rot': v['rot']} for k, v in M.items()},
               'interference': rows, 'extents': ext, 'clearance': rep, 'tub': trep, 'closest': close, 'chassis': CHASSIS,
               'UNDER_CLEARANCE': UNDER_CLEARANCE, 'CAM_HEIGHT': CAM_HEIGHT, 'LAYOUT': LAYOUT, 'VESC': VESC, 'scan_plane_z': SCAN_Z, 'lidar_top_z': LIDAR_TOP,
               'scan_occlusion': occlusion, 'legs_analytic': legs, 'legs_all_in_blind_sector': legs_blind,
               'camera_line_of_sight': {str(k): v for k, v in los.items()}, 'plate_shifts': {'rear': BV2.REAR_SHIFT, 'front': BV2.FRONT_SHIFT}},
              open(os.path.join(OUT, f'newcar_layout{TAG}_report.json'), 'w'), indent=1, default=str)
    if not os.environ.get('TAG'):
        m, sd = MAST.mast(), MAST.saddle()
        cq.exporters.export(m, os.path.join(OUT, 'camera_mast_newcar.step'))
        cq.exporters.export(m, os.path.join(OUT, 'camera_mast_newcar.stl'), tolerance=0.02, angularTolerance=0.1)
        cq.exporters.export(sd, os.path.join(OUT, 'camera_mast_head_newcar.stl'), tolerance=0.02, angularTolerance=0.1)
        for k, v in MAST.checks(m, sd).items():
            print(f'  mast check {k}: {v}')
        vt = VESC_REAR.tray()                     # the VESC=rear option's tray (recess, feet at 41)
        cq.exporters.export(vt, os.path.join(OUT, 'vesc_fsesc67_p41.step'))
        cq.exporters.export(vt, os.path.join(OUT, 'vesc_fsesc67_p41.stl'), tolerance=0.02, angularTolerance=0.1)
        print('  vesc rear tray checks:', {k: v for k, v in VESC_REAR.checks(vt).items() if 'x_' in k or k == 'solids'})

    hole_labels = [(x, y, n, (0, 0, 160)) for n, (x, y) in ALL_HOLES.items()]
    def mount_labels(side):
        return [(m['centre'][0], m['centre'][1], f'{name} [{",".join(m["holes"]) or ("edge, M5 x2" if side == "above" else "chassis rail")}]', (140, 40, 0))
                for name, m in M.items() if m['side'] == side or (side == 'below' and m['side'] == 'tub')]
    above = [(s, COLOURS[g]) for lab, g, s in S if (g == 'board' or 'post' in lab or (lab.split(':')[0] in M and M[lab.split(':')[0]]['side'] == 'above'))]
    below = [(s, COLOURS[g]) for lab, g, s in S if (g == 'board' or g == 'chassis' or (lab.split(':')[0] in M and M[lab.split(':')[0]]['side'] in ('below', 'tub')))]
    ortho(above, os.path.join(OUT, f'newcar_layout{TAG}_top.png'), 'top', hole_labels + mount_labels('above'),
          f'new car ({LAYOUT}, VESC {VESC}), above the board: +x (front) right, +y (car left) up; grey = component envelopes, green = front body posts')
    ortho(below, os.path.join(OUT, f'newcar_layout{TAG}_bottom.png'), 'bottom', hole_labels + mount_labels('below'),
          f'new car ({LAYOUT}, VESC {VESC}), below the board (from underneath: +x right, car right (-y) up); green = Slash 4x4 keep-outs at UNDER_CLEARANCE {UNDER_CLEARANCE:.0f}')
    ortho([(s, COLOURS[g]) for lab, g, s in S], os.path.join(OUT, f'newcar_layout{TAG}_side.png'), 'side', [],
          f'new car, side view from the right (-y), +x right; UNDER_CLEARANCE {UNDER_CLEARANCE:.0f}, CAM_HEIGHT {CAM_HEIGHT:.0f}')
    ortho([(s, COLOURS[g]) for lab, g, s in S], os.path.join(OUT, f'newcar_layout{TAG}_front.png'), 'front', [], 'new car, front view (from +x), car left on the left')
    lines_png([s for lab, g, s in S if g != 'chassis' or 'post' in lab],
              os.path.join(OUT, f'newcar_layout{TAG}_lines.svg'), os.path.join(OUT, f'newcar_layout{TAG}_lines.png'))
    print('\nwritten:', sorted(f for f in os.listdir(OUT) if f.startswith(f'newcar_layout{TAG}_')))
