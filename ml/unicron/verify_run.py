#!/usr/bin/env python3
"""Check every output of a finished shard_run.py run (run with the ML venv's python).

    python ml/unicron/verify_run.py ROOT
"""
import glob, hashlib, json, os, subprocess, sys
import numpy as np, onnxruntime as ort

ROOT = os.path.abspath(sys.argv[1]); C = json.load(open(f'{ROOT}/shard_state/config.json'))
ok = True


def check(cond, msg):
    global ok
    print(('  ok    ' if cond else '  FAIL  ') + msg, flush=True); ok = ok and bool(cond)


nmaps = len(C['maps'].split(',')); frames = 0
print('demo sets')
for d, n, seed in C['sets']:
    s = json.load(open(f'{ROOT}/{d}/summary.json')); E = [json.loads(ln) for ln in open(f'{ROOT}/{d}/episodes.jsonl')]
    files = glob.glob(f'{ROOT}/{d}/*.npz'); frames += sum(e['frames'] for e in E)
    check(s['episodes'] == len(E) == len(files) == n * nmaps,
          f'{d}: {len(E)} episodes, {len(files)} episode files (expected {n * nmaps}), expert success {s["success_rate"]:.2f}')
    check(len({(e['task_id'], e['ep_seed']) for e in E}) == len(E), f'{d}: no episode twice')
    bad = 0
    for f in files:
        try:
            z = np.load(f); assert len(z['act']) > 0 and z['front'].shape[1:] == (96, 128, 3)
        except Exception:
            bad += 1
    check(bad == 0, f'{d}: every episode file loads ({bad} bad)')
    left = glob.glob(f'{ROOT}/{d}/*.tmp*') + glob.glob(f'{ROOT}/{d}/parts/*.partial*') + glob.glob(f'{ROOT}/{d}/parts/*.tmp*')
    check(not left, f'{d}: no temp or partial files left {left[:3]}')
print('data cache')
cache = json.load(open(f'{ROOT}/{C["runs"]}/cache.json')); meta = json.load(open(cache['dir'] + '/meta.json'))
check(meta['n'] + meta['dropped_stops'] == frames,
      f'{meta["n"]} training steps + {meta["dropped_stops"]} dropped stop frames = {frames} recorded frames')
sizes = sum(os.path.getsize(f'{cache["dir"]}/{p["name"]}.front.u8') for p in meta['parts'])
check(sizes == meta['n'] * int(np.prod(meta['front_shape'])), f'frame files hold exactly {meta["n"]} rows')
print('seeds')
for s in C['seeds']:
    run = f'{ROOT}/{C["runs"]}/route_s{s}'
    L = [json.loads(ln) for ln in open(f'{run}/log.jsonl')]
    res = [r['resumed_at_batch'] for r in L if 'resumed_at_batch' in r]
    check([r['epoch'] for r in L] == list(range(C['epochs'])),
          f'seed {s}: epochs logged {[r["epoch"] for r in L]}' + (f', resumed mid-epoch at batch {res}' if res else ''))
    sess = ort.InferenceSession(f'{run}/student.onnx', providers=['CPUExecutionProvider'])
    out = sess.run(['action'], {'front': np.zeros((2, 3, 96, 128), np.float32), 'bev': np.zeros((2, 1, 96, 96), np.float32),
                                'state': np.zeros((2, 5), np.float32), 'ids': np.zeros((2, 24), np.int64)})[0]
    check(out.shape == (2, 2) and np.isfinite(out).all(), f'seed {s}: student.onnx loads and runs')
    for name, _, _ in C['evals']:
        e = json.load(open(f'{run}/{name}/summary.json'))
        exp = C['neval'] * (nmaps + len(C['eval_extra_maps'].split(',')))
        check(e['episodes'] == exp, f'seed {s} {name}: {e["episodes"]}/{exp} episodes, success {e["success_rate"]:.2f}')
print('selection')
ch = json.load(open(f'{ROOT}/{C["runs"]}/route/choice.json'))
md5 = lambda p: hashlib.md5(open(p, 'rb').read()).hexdigest()
check(md5(f'{ROOT}/{C["runs"]}/route/student.onnx') == md5(f'{ROOT}/{ch["chosen"]}/student.onnx'),
      f'route/student.onnx is the chosen seed {ch["chosen"]}')
sys.path.insert(0, f'{ROOT}/ml'); import shard_run                     # noqa: E402
t = shard_run.systemctl('is-active', shard_run.unit(ROOT) + '.timer').stdout.strip()
check(t != 'active', f'supervisor timer switched off after the run ({t})')
state = json.load(open(f'{ROOT}/shard_state/state.json'))
re = {n: v['attempts'] for n, v in state.items() if v.get('attempts', 0) > 1}
print(f'shards that were restarted: {re}')
print('ALL CHECKS PASSED' if ok else 'SOME CHECKS FAILED')
sys.exit(0 if ok else 1)
