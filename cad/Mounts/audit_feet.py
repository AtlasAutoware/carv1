"""Can every foot in the set actually be installed, and does every part's foot pitch match
the plate it is meant for?

For each printed part this builds the model with the same environment board_layout.py
uses, then for every foot sweeps a box the size of the foot's bounding box 200 mm in each
of five directions (+z, +x, -x, +y, -y) and measures how much of the part stands in the
way. A recess foot needs +z clear; a slide-channel foot needs one horizontal direction
clear. If no direction is clear, the foot cannot be put in and the part is unusable no
matter what the interference checks said.

    PLATE=v2 python3 audit_feet.py        (default)
    PLATE=stl python3 audit_feet.py       the original 15-hole plate
"""
import os, sys, importlib, subprocess, json
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, 'newcar'))
PLATE = os.environ.get('PLATE', 'v2')
PX, PY = (40.0, 41.0) if PLATE == 'v2' else (50.0, 47.5)

# (label, module, env, builder, feet fn, plate it is for)
SET = [
    ('omni20_cradle',          'omni20_mount',            {},                                              'cradle', 'feet', PLATE),
    ('liteon_45w_brick_v2',    'liteon_45w_brick_mount',  {'RETAIN': 'recess', 'PITCH_Y': str(2 * PY), 'FEET_X_OFFSET': '13.5'}, 'cup', 'feet', PLATE),
    ('jetson_orin_nano_under', 'jetson_orin_nano_mount',  {'RETAIN': 'snap', 'SNAP_ROWS': '1'},           'tray', 'feet', PLATE),
    ('small_board_plate_v2',   'small_board_plate_mount', {'RETAIN': 'recess'},                            'plate', 'feet', PLATE),
    ('rplidar_c1_v2',          'rplidar_c1_mount',        {},                                              'plinth', 'feet', PLATE),
    ('camera_mast_v2',         'camera_mast_mount',       {'LIDAR_X': '60', 'LIDAR_Y': '-20.5'},           'mast', 'feet', PLATE),
    ('cable_tidy_edge',        'cable_tidy_mount',        {'EDGE': '1'},                                   'rail', None, None),
    ('vesc_tub_bracket',       'vesc_tub_bracket_mount',  {},                                              'saddle', None, None),
    ('vesc_fsesc67',           'vesc_fsesc67_mount',      {'RETAIN': 'recess'},                            'tray', 'feet', PLATE),
    ('lipo_smc_9000',          'lipo_smc_9000_mount',     {},                                              'frame', 'feet', PLATE),
    ('power_board_mount',      'power_board_mount',       {},                                              'plate', 'feet', PLATE),
    ('tim561',                 'tim561_mount',            {},                                              'plinth', 'feet', 'v2'),
    ('battery_bank_underslung','battery_bank_underslung', {},                                              'tray', 'feet', 'v2'),
]

SNAP_BUMP_MAX = 120.0   # mm3: what two snap-tongue bumps present to a foot sliding past them (intended)
DIRS = {'+z': (0, 0, 1), '+x': (1, 0, 0), '-x': (-1, 0, 0), '+y': (0, 1, 0), '-y': (0, -1, 0)}


def run_one(label, mod, env, builder, feet_fn, plate):
    """Runs in a fresh interpreter so each module sees its own environment."""
    code = f'''
import os, sys, json
os.environ.update({{'PEG_PITCH': '{PX}', 'PITCH_Y': '{PY}'}})
os.environ.update({json.dumps(env)})
sys.path.insert(0, {HERE!r}); sys.path.insert(0, {os.path.join(HERE, 'newcar')!r})
import cadquery as cq
from clipless import FOOT_FLANGE_T
import {mod} as M
part = M.{builder}()
if isinstance(part, tuple): part = part[0]
out = {{'solids': len(part.val().Solids())}}
if {feet_fn!r}:
    feet = getattr(M, {feet_fn!r})()
    out['feet'] = [tuple(round(v, 2) for v in (f.val().BoundingBox().center.x, f.val().BoundingBox().center.y)) for f in feet]
    res = []
    for f in feet:
        # The foot is a flange box over a peg box, so its exact swept volume along a
        # direction is those two boxes each extended 200 mm from their own face. This
        # is a T-shaped path: the flange runs along the channel ledges, the peg between
        # them, which a single bounding-box sweep would get wrong.
        b = f.val().BoundingBox()
        # flange: the slab at the top of the foot; peg: everything below it
        zf = b.zmax - FOOT_FLANGE_T
        flange = (b.xmin, b.xmax, b.ymin, b.ymax, zf, b.zmax)
        pb = f.val().intersect(cq.Workplane('XY').box(b.xlen + 1, b.ylen + 1, b.zlen).translate(((b.xmin + b.xmax) / 2, (b.ymin + b.ymax) / 2, zf - b.zlen / 2)).val()).BoundingBox()
        peg = (pb.xmin, pb.xmax, pb.ymin, pb.ymax, pb.zmin, pb.zmax)
        best = None
        for name, (dx, dy, dz) in {DIRS!r}.items():
            L = 200.0
            swept = None
            for (x0, x1, y0, y1, z0, z1) in (flange, peg):
                if dx: x0, x1 = (x1, x1 + L) if dx > 0 else (x0 - L, x0)
                if dy: y0, y1 = (y1, y1 + L) if dy > 0 else (y0 - L, y0)
                if dz: z0, z1 = (z1, z1 + L) if dz > 0 else (z0 - L, z0)
                bx = cq.Workplane('XY').box(x1 - x0, y1 - y0, z1 - z0).translate(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))
                swept = bx if swept is None else swept.union(bx)
            it = part.val().intersect(swept.val())
            v = it.Volume()
            where = None
            if v > 0.5:
                ib = it.BoundingBox()
                where = [round(ib.xmin, 1), round(ib.xmax, 1), round(ib.ymin, 1), round(ib.ymax, 1), round(ib.zmin, 1), round(ib.zmax, 1)]
            if best is None or v < best[1]: best = (name, round(v, 1), where)
        res.append(best)
    out['install'] = res
print('JSON' + json.dumps(out))
'''
    r = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True)
    line = [l for l in r.stdout.splitlines() if l.startswith('JSON')]
    if not line:
        return {'error': (r.stderr.strip().splitlines() or ['no output'])[-1]}
    return json.loads(line[0][4:])


def pitch_of(feet):
    xs = sorted(set(round(x, 1) for x, _ in feet)); ys = sorted(set(round(y, 1) for _, y in feet))
    dx = [round(b - a, 1) for a, b in zip(xs, xs[1:])]; dy = [round(b - a, 1) for a, b in zip(ys, ys[1:])]
    return dx, dy


if __name__ == '__main__':
    print(f'plate {PLATE}: hole pitch {PX} along x, {PY} across')
    bad = 0
    for label, mod, env, builder, feet_fn, plate in SET:
        r = run_one(label, mod, env, builder, feet_fn, plate)
        if 'error' in r:
            print(f'{label:26s} ERROR {r["error"]}'); bad += 1; continue
        line = f'{label:26s} solids={r["solids"]}'
        ok = r['solids'] == 1
        if feet_fn:
            dx, dy = pitch_of(r['feet'])
            pitch_ok = all(abs(d - PX) < 0.05 or abs(d - 2 * PX) < 0.05 for d in dx) and \
                       all(abs(d - PY) < 0.05 or abs(d - 2 * PY) < 0.05 for d in dy)
            blocked = [i for i, (_, v, _w) in enumerate(r['install']) if v > SNAP_BUMP_MAX]
            line += f'  feet={len(r["feet"])} pitch x{dx} y{dy} {"ok" if pitch_ok else "WRONG PITCH"}'
            line += '  install=' + ','.join(f'{d}:{v:.0f}' + (f'@x{w[0]}..{w[1]},y{w[2]}..{w[3]},z{w[4]}..{w[5]}' if (v > SNAP_BUMP_MAX and w) else '') for d, v, w in r['install'])
            line += '  ' + ('BLOCKED feet ' + str(blocked) if blocked else 'installable')
            ok = ok and pitch_ok and not blocked
        else:
            line += '  (no feet: bolts/clamps)'
        print(('PASS ' if ok else 'FAIL ') + line)
        bad += 0 if ok else 1
    print(f'\n{bad} part(s) failed')
    RC = 1 if bad else 0


# ---- what is actually on disk -----------------------------------------------------------
# The model can be right and the exported file wrong (the Omni cradle: model rebuilt at 40
# inside board_layout.py, STL on disk still the 49 mm part, and that is what got sliced).
# Measure the peg through-holes in the STL itself for the recess parts.
def stl_pitch(path, floor_z=1.0):
    import trimesh, numpy as np
    m = trimesh.load(path)
    s = m.section(plane_origin=[0, 0, floor_z], plane_normal=[0, 0, 1])
    if s is None: return None
    p, _ = s.to_planar()
    cs = []
    for poly in p.polygons_full:
        for ring in poly.interiors:
            xs, ys = np.array(ring.coords).T
            w, h = xs.max() - xs.min(), ys.max() - ys.min()
            if 18 < w < 26 and 18 < h < 26:
                cs.append((round(float((xs.min() + xs.max()) / 2), 1), round(float((ys.min() + ys.max()) / 2), 1)))
    return sorted(cs)


if __name__ == '__main__' and os.environ.get('STL', '1') == '1':
    print('\nSTL files on disk (recess parts, peg holes at z=1):')
    for name in ['omni20_cradle', 'omni20_cradle_v2', 'liteon_45w_brick_v2', 'small_board_plate_v2',
                 'vesc_fsesc67', 'vesc_fsesc67_v2', 'lipo_smc_9000', 'lipo_smc_9000_v2',
                 'power_board_mount']:
        path = os.path.join(HERE, 'out', name + '.stl')
        if not os.path.exists(path): print(f'  {name:24s} missing'); continue
        cs = stl_pitch(path, 6.0 if 'vesc' in name else 1.0)
        if not cs: print(f'  {name:24s} no peg holes found at z=1'); continue
        dx, dy = pitch_of(cs)
        print(f'  {name:24s} holes {cs}  pitch x{dx} y{dy}')
    sys.exit(RC)
