"""Tessellate the Baseplate v2 layout (mounts, feet, clipless pieces, component envelopes,
chassis keep-out model, plate) into one JSON for the browser viewer, view3d.html."""
import os, sys, json, base64, struct
os.environ.setdefault('PLATE', 'v2'); os.environ['SKIP_REGEN'] = '1'
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import board_layout as BL

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out')
M = BL.build_mounts_v2()
used = [h for m in M.values() for h in m['holes']]
S = BL.all_solids(M, BL.chassis_solids(), BL.board_solid(used), BL.bracket_solids(), used)

def b64(a): return base64.b64encode(np.ascontiguousarray(a).tobytes()).decode()
parts = []; tri_total = 0
for lab, grp, s in S:
    shape = s.val() if hasattr(s, 'val') else s
    tol = 0.35 if grp in ('chassis', 'envelope', 'board') else 0.15
    v, t = shape.tessellate(tol, 0.3)
    V = np.array([(p.x, p.y, p.z) for p in v], np.float32); T = np.array(t, np.uint32)
    parts.append({'label': lab, 'group': grp, 'n': len(T), 'v': b64(V), 't': b64(T)}); tri_total += len(T)
meta = {'plate': BL.PLATE, 'holes': {k: list(v) for k, v in BL.ALL_HOLES.items()},
        'placements': {k: {'side': m['side'], 'holes': m['holes']} for k, m in M.items()}}
json.dump({'meta': meta, 'parts': parts}, open(os.path.join(OUT, 'scene_v2.json'), 'w'))
print(len(parts), 'parts,', tri_total, 'triangles,', os.path.getsize(os.path.join(OUT, 'scene_v2.json')) // 1024, 'kB')
