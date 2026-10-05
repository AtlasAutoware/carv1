#!/usr/bin/env python3
"""sim_rollout: closed-loop evaluation and on-policy data collection for the student policy.

Everything runs in the ROS-free sim (f1tenth_gym_ros/sim_core.py + goal_core.py), with the
network fed through f1tenth_gym_ros/policy_io.py -- the same code the car's policy_bridge
uses -- so a number measured here is a number about the deployed input pipeline, not about
a training loader.

Modes
  eval    run a policy (ONNX file or 'expert') on a fixed, seeded task set; write per-episode
          results + a summary JSON.
  collect record (observation, expert label) pairs while a mixture of expert and student
          drives (DAgger; beta = probability the expert's action is executed at a tick).
          beta=1 is plain expert demonstration.

    python3 ml/sim_rollout.py eval --policy models/student.onnx --maps levine,Spielberg_map \
        --n 60 --seed 1000 --out runs/eval/baseline
    python3 ml/sim_rollout.py collect --policy expert --beta 1 --maps levine,Spielberg_map,comp_track \
        --n 300 --seed 0 --out data/demos

Restartable and shardable (for long runs on machines where processes get killed):
  * every finished episode is appended to a journal, and a run started again with the same
    arguments skips the episodes already done; episode files and results are written to a temp
    file and renamed, so a SIGKILL never leaves a torn file;
  * --shard K/N runs every N-th episode starting at K and writes <out>/parts/part-K-of-N.jsonl;
    `merge --out <out>` joins the N parts into episodes.jsonl + summary.json (same files as an
    unsharded run). The episodes and their seeds are the same either way.
"""
import argparse, fcntl, glob, hashlib, json, math, os, random, re, sys, threading, time
from concurrent.futures import ProcessPoolExecutor, as_completed
from concurrent.futures.process import BrokenProcessPool
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'f1tenth_gym_ros')); sys.path.insert(0, os.path.join(REPO, 'tools'))
from sim_core import SimMap, bicycle_step, render_fpv, load_map          # noqa: E402
from goal_core import Planner, pure_pursuit, describe_route              # noqa: E402
import policy_io as PIO                                                  # noqa: E402

PARA = [lambda s: s,
        lambda s: s.replace('Go straight', 'Drive straight ahead').replace('go straight', 'keep going straight'),
        lambda s: s.replace('to the end and stop', 'until you reach the goal, then stop'),
        lambda s: 'Please ' + s[0].lower() + s[1:],
        lambda s: s.replace('Turn', 'Take a').replace('turn ', 'take a ').replace('a left', 'left turn').replace('a right', 'right turn'),
        lambda s: s + '. Avoid the walls.']            # identical to tools/sim_generate.py

WHEELBASE, MAX_STEER, BEAMS, CAM = 0.33, 0.4, 540, (640, 480, 460.5)
SCAN_MAX = PIO.BEV_EXTENT + 1.0
GOAL_TOL = 0.5          # m: "reached"
STOP_V = 0.25           # m/s: "stopped" once inside GOAL_TOL
_MAPS = {}


def get_map(name):
    if name not in _MAPS:
        occ, res, origin = load_map(os.path.join(REPO, 'maps', f'{name}.yaml'))
        _MAPS[name] = (SimMap(occ, res, origin), Planner(occ, res, origin))
    return _MAPS[name]


def _sample_map(args):
    """The tasks of one map (map index mi in the list): its own RNG streams, so maps are independent."""
    name, mi, n_per_map, seed, min_dist, max_dist = args
    rng = np.random.default_rng(seed * 1000 + mi); prng = random.Random(seed * 1000 + mi)
    _, pl = get_map(name); k = tries = 0; tasks = []
    while k < n_per_map and tries < n_per_map * 10:
        tries += 1
        s = pl.sample_free(rng, 1)[0]; g = pl.sample_free_near(rng, s, min_dist, max_dist)
        if g is None: continue
        path = pl.plan(s, g)
        if path is None or len(path) < 4: continue
        instr = prng.choice(PARA)(describe_route(path))
        tasks.append({'map': name, 'start': list(s), 'goal': list(g), 'path': [list(p) for p in path],
                      'instruction': instr, 'task_id': f'{name}:{seed}:{k}'})
        k += 1
    return tasks


def sample_tasks(maps, n_per_map, seed, min_dist=3.0, max_dist=25.0, workers=1):
    """Deterministic (start, goal, path, instruction) set. Same seed -> same tasks.
    workers > 1 plans the maps in parallel processes (same tasks, same order)."""
    jobs = [(name, mi, n_per_map, seed, min_dist, max_dist) for mi, name in enumerate(maps)]
    if workers > 1 and len(jobs) > 1:
        with ProcessPoolExecutor(min(workers, len(jobs)), initializer=_exit_with_parent) as ex:
            per_map = list(ex.map(_sample_map, jobs))
    else:
        per_map = [_sample_map(j) for j in jobs]
    return [t for ts in per_map for t in ts]


def get_tasks(maps, n_per_map, seed):
    """sample_tasks(), cached in data/_tasks/: every shard of a sharded run needs the same task
    list, and route planning takes ~0.6 s a task (minutes per set), too slow to repeat in every
    shard. The first shard plans (one process per map) while the others wait on the lock. The
    key includes the map files, so editing a map invalidates the cache."""
    stats = [[os.path.basename(f), os.path.getsize(f), int(os.path.getmtime(f))]
             for m in maps for f in sorted(glob.glob(os.path.join(REPO, 'maps', m + '.*')))]
    key = hashlib.sha1(json.dumps([maps, n_per_map, seed, stats]).encode()).hexdigest()[:16]
    d = os.path.join(REPO, 'data', '_tasks'); os.makedirs(d, exist_ok=True)
    fn = os.path.join(d, key + '.json')
    if not os.path.isfile(fn):
        with open(fn + '.lock', 'w') as lk:
            fcntl.flock(lk, fcntl.LOCK_EX)              # shards started together: one plans, the rest wait
            if not os.path.isfile(fn):
                _write_lines(fn, [json.dumps(sample_tasks(maps, n_per_map, seed, workers=len(maps)))])
    return json.load(open(fn))


def path_len(P):
    P = np.asarray(P); return float(np.hypot(*np.diff(P, axis=0).T).sum())


def perturb_obs(front, scan, kind, rng):
    """Sim-side proxies for the sim-to-real gap. Not a substitute for the real car."""
    if kind in ('lidar', 'both'):
        scan = scan + rng.normal(0, 0.03, scan.shape).astype(np.float32)
        scan[rng.random(scan.shape) < 0.19] = 0.0          # C1 on the bench: 581/720 bins valid
    if kind in ('camera', 'both'):
        f = front.astype(np.float32)
        f = f * rng.uniform(0.6, 1.4) + rng.uniform(-40, 40, 3)[None, None, :]
        f += rng.normal(0, 12, f.shape)
        front = np.clip(f, 0, 255).astype(np.uint8)
    if kind == 'nocam':
        front = np.zeros_like(front)
    return front, scan


class OnnxPolicy:
    def __init__(self, path):
        import onnxruntime as ort
        so = ort.SessionOptions(); so.intra_op_num_threads = 1; so.inter_op_num_threads = 1
        self.sess = ort.InferenceSession(path, so, providers=['CPUExecutionProvider'])
        self.order = PIO.action_order_of(self.sess)
        self.lidar_x, self.cam_x = PIO.sensor_geom(PIO.model_config(self.sess))   # where training had the sensors
        if os.environ.get('LEGACY_SWAP') == '1':            # reproduce the pre-9/23 policy_bridge bug
            self.order = self.order[::-1]

    def __call__(self, front, bev, state, ids):
        out = self.sess.run(['action'], PIO.make_feed(front, bev, state, ids))[0][0]
        return PIO.split_action(out, self.order)          # (speed, steer)


def rollout(task, policy, beta=0.0, record=False, seed=0, perturb=None, dt=0.02,
            sensor_hz=10.0, max_speed=1.5, time_mult=2.0, hint_fn=None):
    """One closed-loop episode. policy: OnnxPolicy or None (= expert). Returns (result, data)."""
    sm, _ = get_map(task['map']); path = [tuple(p) for p in task['path']]
    rng = np.random.default_rng(seed); P = np.asarray(path)
    cum = np.concatenate([[0.0], np.hypot(*np.diff(P, axis=0).T).cumsum()]); L = float(cum[-1])
    sx, sy = path[0]; th0 = math.atan2(path[1][1] - sy, path[1][0] - sx)
    st = np.array([sx, sy, th0, 0.0]); ids = PIO.text_ids(task['instruction'])
    angles = -math.pi + 2 * math.pi * np.arange(BEAMS) / BEAMS
    inc = 2 * math.pi / BEAMS
    timeout = time_mult * L / 0.8 + 8.0
    act = (0.0, 0.0); steer_applied = 0.0; t = 0.0; next_tick = 0.0
    rec = {'front': [], 'bev': [], 'state': [], 'act': [], 'scan': []}
    res = {'task_id': task['task_id'], 'map': task['map'], 'route_m': round(L, 2), 'collided': False,
           'reached': False, 'stopped_at_goal': False, 'progress': 0.0, 'time_s': 0.0,
           'steer_err': 0.0, 'speed_err': 0.0, 'ticks': 0, 'student_ticks': 0}
    se = sp = 0.0; n = 0; hold = 0.0
    while t < timeout:
        if t >= next_tick - 1e-9:
            next_tick += 1.0 / sensor_hz
            # 7 m is enough: the policy only sees the scan through the 6 m BEV raster
            scan = sm.raycast(st[0], st[1], st[2] + angles, SCAN_MAX)
            scan = np.clip(scan + rng.normal(0, 0.01, BEAMS), 0, SCAN_MAX).astype(np.float32)
            rgb = render_fpv(sm, st[0], st[1], st[2], CAM[0], CAM[1], CAM[2])
            front = PIO.front_image(np.ascontiguousarray(rgb[:, :, ::-1]))   # the car feeds BGR
            if perturb: front, scan = perturb_obs(front, scan, perturb, rng)
            bev = PIO.bev_image(scan, -math.pi, inc)
            wz = st[3] / WHEELBASE * math.tan(steer_applied)
            # no_route: the car-side hint has no route, so policy_bridge would publish zero speed.
            # (Until 10/5 this flag shared the name `hold` with the stop timer below, which reset
            # the timer every tick, so episodes never ended at the goal and ran to the timeout.)
            if hint_fn is None:
                hx, hy = PIO.route_hint(st[:3], path); no_route = False
            else:                                   # car-side hint (ml/car_route_check.py)
                h = hint_fn(st, t); no_route = h is None; hx, hy = (0.0, 0.0) if no_route else h
            state = np.array([st[3], wz, hx, hy, wz], np.float32)   # [2:4] = route hint (masked out of older models)
            ev, es, edone, _ = pure_pursuit(st[:3], path, wheelbase=WHEELBASE)
            if record:
                rec['front'].append(front); rec['bev'].append(bev); rec['state'].append(state)
                rec['scan'].append(scan.astype(np.float16))
                rec['act'].append((ev, es))
            use_expert = policy is None or rng.random() < beta
            if use_expert:
                act = (ev, es)
            else:
                v, s = policy(front, bev, state, ids)
                act = (float(np.clip(v, 0.0, max_speed)), float(np.clip(s, -MAX_STEER, MAX_STEER)))
                if no_route: act = (0.0, act[1])    # policy_bridge publishes zero speed without a route
                se += abs(act[1] - es); sp += abs(act[0] - ev); n += 1
                res['student_ticks'] += 1
            res['ticks'] += 1
        steer_applied = act[1]
        nxt = bicycle_step(st, act[0], act[1], WHEELBASE, dt)
        t += dt
        if sm.occupied(nxt[0], nxt[1]):
            res['collided'] = True; break
        st = nxt
        d = np.hypot(P[:, 0] - st[0], P[:, 1] - st[1]); i = int(np.argmin(d))
        if d[i] < 1.0: res['progress'] = max(res['progress'], float(cum[i] / max(L, 1e-6)))
        dg = math.hypot(P[-1, 0] - st[0], P[-1, 1] - st[1])
        if dg < GOAL_TOL:
            res['reached'] = True; res['progress'] = 1.0
            # keep going until the car has actually stopped and held it for a second: the
            # expert's stop (speed 0 at the goal) must be in the data, or "...and stop" is
            # an instruction the student has never seen demonstrated
            if abs(st[3]) < STOP_V:
                res['stopped_at_goal'] = True; hold += dt
                if hold > 1.0: break
    res['time_s'] = round(t, 2)
    res['success'] = bool(res['reached'] and not res['collided'])
    if n: res['steer_err'] = round(se / n, 4); res['speed_err'] = round(sp / n, 4)
    data = None
    if record and rec['act']:
        data = {'front': np.asarray(rec['front'], np.uint8), 'bev': np.asarray(rec['bev'], np.uint8),
                'state': np.asarray(rec['state'], np.float32), 'act': np.asarray(rec['act'], np.float32),
                'scan': np.asarray(rec['scan'], np.float16),
                'ids': np.repeat(np.asarray(ids, np.int64)[None], len(rec['act']), 0)}
    return res, data


def rollout_car(task, policy, car, beta=0.0, record=False, seed=0, dt=0.02, sensor_hz=10.0, time_mult=2.0):
    """One closed-loop episode under car conditions (ml/car_model.py): the lidar and camera where
    the car has them and as old as there, the car-side route hint (noisy, lagged pose; no hint
    inside goal_tol), policy_bridge's clips, AEB and speed mapping, motor and servo dynamics, and a
    collision is any part of the car body touching a wall. policy: OnnxPolicy or None (= expert).
    Recorded data has the same arrays as rollout()'s plus 'geom' = (lidar_x, cam_x): the clean
    sensor readings (training adds its own noise), the state with the car-side hint, and the
    expert's action for the TRUE state as the label."""
    import car_model as CM
    c = car; WB = c['wheelbase']
    sm, _ = get_map(task['map']); path = [tuple(p) for p in task['path']]
    rng = np.random.default_rng(seed); P = np.asarray(path)
    cum = np.concatenate([[0.0], np.hypot(*np.diff(P, axis=0).T).cumsum()]); L = float(cum[-1])
    sx, sy = path[0]; th0 = math.atan2(path[1][1] - sy, path[1][0] - sx)
    st = np.array([sx, sy, th0, 0.0])
    ids = PIO.text_ids(task['instruction'] if c['instruction'] == 'task' else c['instruction'])
    angles = -math.pi + 2 * math.pi * np.arange(BEAMS) / BEAMS
    inc = 2 * math.pi / BEAMS
    timeout = time_mult * L / 0.8 + 8.0
    hist = CM.History(max(c['lidar_lat'], c['cam_lat'], c['pose_lag']) + 2 * dt)
    hint = CM.Hint(path, c, rng); motor = CM.Motor(c, dt); servo = CM.Servo(c, dt)
    dart = CM.OU(c['dart'], c['dart_tau'], 1.0 / sensor_hz, rng)
    # policy_bridge (10/5) centres the raster where the model's training had the lidar
    bev_dx = c['lidar_x'] - policy.lidar_x if (c['bridge_shift'] and policy is not None) else 0.0
    steer_now = 0.0; v_tgt = 0.0; s_cmd = 0.0; t = 0.0; next_tick = 0.0
    rec = {'front': [], 'bev': [], 'state': [], 'act': [], 'scan': []}
    res = {'task_id': task['task_id'], 'map': task['map'], 'route_m': round(L, 2), 'collided': False,
           'reached': False, 'stopped_at_goal': False, 'progress': 0.0, 'time_s': 0.0,
           'steer_err': 0.0, 'speed_err': 0.0, 'ticks': 0, 'student_ticks': 0, 'aeb_ticks': 0, 'max_speed': 0.0}
    se = sp = 0.0; n = 0; at_goal = 0.0
    while t < timeout:
        hist.add(t, st)
        if t >= next_tick - 1e-9:
            next_tick += 1.0 / sensor_hz
            sl = hist.at(t - c['lidar_lat']); sc = hist.at(t - c['cam_lat'])
            scan = sm.raycast(sl[0] + c['lidar_x'] * math.cos(sl[2]), sl[1] + c['lidar_x'] * math.sin(sl[2]),
                              sl[2] + angles, SCAN_MAX)
            scan = np.clip(scan + rng.normal(0, 0.01, BEAMS), 0, SCAN_MAX).astype(np.float32)
            rgb = render_fpv(sm, sc[0] + c['cam_x'] * math.cos(sc[2]), sc[1] + c['cam_x'] * math.sin(sc[2]), sc[2],
                             CAM[0], CAM[1], CAM[2])
            front = PIO.front_image(np.ascontiguousarray(rgb[:, :, ::-1]))   # the car feeds BGR
            # what the car's policy gets: the C1's empty bins and range noise, and the camera as configured
            seen = scan.copy()
            if c['lidar_noise'] > 0: seen = seen + rng.normal(0, c['lidar_noise'], BEAMS).astype(np.float32)
            if c['lidar_drop'] > 0: seen[rng.random(BEAMS) < c['lidar_drop']] = 0.0
            if c['cam'] == 'render': fseen = front
            elif c['cam'] == 'perturb': fseen = perturb_obs(front, seen, 'camera', rng)[0]
            else: fseen = np.zeros_like(front)
            h = hint(hist.at(t - c['pose_lag']))
            if h is None:            # inside goal_tol: policy_bridge publishes zero speed and steering
                v_cmd = s_cmd = 0.0
            else:
                wz = st[3] / WB * math.tan(steer_now)
                state = np.array([st[3], wz, h[0], h[1], wz], np.float32)
                ev, es, _, _ = pure_pursuit(st[:3], path, wheelbase=WB, v_max=c['expert_vmax'])
                if record:
                    rec['front'].append(front); rec['bev'].append(PIO.bev_image(scan, -math.pi, inc))
                    rec['state'].append(state); rec['scan'].append(scan.astype(np.float16)); rec['act'].append((ev, es))
                student = not (policy is None or rng.random() < beta)
                if student:
                    v, s = policy(fseen, PIO.bev_image(seen, -math.pi, inc, dx=bev_dx), state, ids)
                    res['student_ticks'] += 1
                else:
                    v, s = ev, es + dart()
                v_cmd, s_cmd = CM.bridge(v, s, seen, inc, c)
                if v > 0.0 and math.isfinite(s) and v_cmd == 0.0: res['aeb_ticks'] += 1
                if student:
                    se += abs(s_cmd - es); sp += abs(v_cmd - ev); n += 1
            v_tgt = CM.wheel_target(v_cmd, c)
            res['ticks'] += 1
        st2 = st.copy(); st2[3] = motor.step(st[3], v_tgt)
        steer_now = servo.step(steer_now, s_cmd)
        nxt = bicycle_step(st2, st2[3], steer_now, WB, dt)     # speed already set: its speed tracking is a no-op
        t += dt
        if CM.body_hits(sm, nxt) if c['footprint'] else sm.occupied(nxt[0], nxt[1]):
            res['collided'] = True; break
        st = nxt; res['max_speed'] = max(res['max_speed'], float(st[3]))
        d = np.hypot(P[:, 0] - st[0], P[:, 1] - st[1]); i = int(np.argmin(d))
        if d[i] < 1.0: res['progress'] = max(res['progress'], float(cum[i] / max(L, 1e-6)))
        dg = math.hypot(P[-1, 0] - st[0], P[-1, 1] - st[1])
        if dg < GOAL_TOL:
            res['reached'] = True; res['progress'] = 1.0
            if abs(st[3]) < STOP_V:
                res['stopped_at_goal'] = True; at_goal += dt
                if at_goal > 1.0: break
    res['time_s'] = round(t, 2); res['max_speed'] = round(res['max_speed'], 3)
    res['success'] = bool(res['reached'] and not res['collided'])
    if n: res['steer_err'] = round(se / n, 4); res['speed_err'] = round(sp / n, 4)
    data = None
    if record and rec['act']:
        data = {'front': np.asarray(rec['front'], np.uint8), 'bev': np.asarray(rec['bev'], np.uint8),
                'state': np.asarray(rec['state'], np.float32), 'act': np.asarray(rec['act'], np.float32),
                'scan': np.asarray(rec['scan'], np.float16),
                'ids': np.repeat(np.asarray(ids, np.int64)[None], len(rec['act']), 0),
                'geom': np.array([c['lidar_x'], c['cam_x']], np.float32)}
    return res, data


_POLICY = None


def _exit_with_parent():
    """Pool workers: exit as soon as the parent process is gone (SIGKILLed, say). An orphaned
    worker would otherwise keep the shard's lock file held and the shard would look busy."""
    ppid = os.getppid()

    def watch():
        while os.getppid() == ppid: time.sleep(2)
        os._exit(1)
    threading.Thread(target=watch, daemon=True).start()


def _init(policy_path):
    global _POLICY
    _exit_with_parent()
    _POLICY = None if policy_path == 'expert' else OnnxPolicy(policy_path)


def _write_lines(path, lines):
    """Text file written atomically (temp file + fsync + rename)."""
    tmp = f'{path}.tmp.{os.getpid()}'
    with open(tmp, 'w') as f:
        f.write(''.join(ln + '\n' for ln in lines)); f.flush(); os.fsync(f.fileno())
    os.replace(tmp, path)


def _episode_file(out_dir, task, seed):
    return os.path.join(out_dir, task['task_id'].replace(':', '_') + f'_s{seed}.npz')


def _save_npz(fn, data):
    """Atomic (temp file + fsync + rename), so a kill mid-write never leaves a torn .npz for the
    training cache to trip over. Compressed: rendered frames and rasters compress well, which
    keeps the disk writes down. np.load reads it exactly like an uncompressed one."""
    tmp = f'{fn}.tmp.{os.getpid()}'
    with open(tmp, 'wb') as f:
        np.savez_compressed(f, **data); f.flush(); os.fsync(f.fileno())
    os.replace(tmp, fn)


def _work(args):
    task, beta, record, seed, perturb, out_dir = args[:6]
    car = args[6] if len(args) > 6 else None
    if car:
        res, data = rollout_car(task, _POLICY, car, beta=beta, record=record, seed=seed)
    else:
        res, data = rollout(task, _POLICY, beta=beta, record=record, seed=seed, perturb=perturb)
    res['ep_seed'] = seed
    if data is not None and out_dir:
        fn = _episode_file(out_dir, task, seed); _save_npz(fn, data)
        res['file'] = os.path.basename(fn); res['frames'] = int(len(data['act']))
    return res


def _key(r):
    return f"{r['task_id']}|{r.get('ep_seed')}"


def _read_journal(path, sig, out_dir):
    """Episodes that an interrupted run of the same job already finished, {key: result}. Empty if
    there is no journal or it belongs to a different job (other policy file, maps, seed, ...).
    A line torn by a kill mid-write is dropped, and so is an episode whose .npz is missing."""
    try:
        lines = open(path).read().splitlines()
    except FileNotFoundError:
        return {}
    try:
        if json.loads(lines[0]).get('journal') != sig: return {}
    except (IndexError, ValueError, AttributeError):
        return {}
    done = {}
    for ln in lines[1:]:
        try: r = json.loads(ln)
        except ValueError: continue
        if 'file' in r and not os.path.isfile(os.path.join(out_dir, r['file'])): continue
        done[_key(r)] = r
    return done


def summarize(results):
    k = len(results) or 1
    f = lambda key: float(np.mean([r[key] for r in results])) if results else 0.0
    ci = lambda key: float(1.96 * np.std([float(r[key]) for r in results]) / math.sqrt(k))
    return {'episodes': len(results), 'success_rate': f('success'), 'success_ci95': ci('success'),
            'collision_rate': f('collided'), 'stopped_at_goal_rate': f('stopped_at_goal'),
            'mean_progress': f('progress'), 'mean_steer_err_rad': f('steer_err'),
            'mean_speed_err_mps': f('speed_err')}


def write_summary(out, results, policy, maps, seed, beta, perturb, secs, car=None):
    results.sort(key=lambda r: (r['task_id'], r.get('ep_seed', 0)))
    _write_lines(os.path.join(out, 'episodes.jsonl'), [json.dumps(r) for r in results])
    summ = summarize(results); summ.update({'policy': policy, 'maps': maps, 'seed': seed, 'beta': beta,
                                            'perturb': perturb, 'secs': round(secs, 1)})
    if car: summ['car'] = car
    by_map = {m: summarize([r for r in results if r['map'] == m]) for m in maps.split(',')}
    summ['by_map'] = by_map
    _write_lines(os.path.join(out, 'summary.json'), [json.dumps(summ, indent=1)])
    print(json.dumps({k: v for k, v in summ.items() if k != 'by_map'}))
    for m, s in by_map.items():
        print(f"  {m:16s} success {s['success_rate']:.2f}  collide {s['collision_rate']:.2f}  progress {s['mean_progress']:.2f}")


PART_RE = re.compile(r'^part-(\d+)-of-(\d+)\.jsonl$')


def merge(out):
    """Join the N parts of a sharded run (<out>/parts/part-K-of-N.jsonl) into episodes.jsonl and
    summary.json. Exits non-zero if a part is missing or the parts are from different jobs."""
    pdir = os.path.join(out, 'parts'); parts = {}
    for f in glob.glob(os.path.join(pdir, 'part-*.jsonl')):
        m = PART_RE.match(os.path.basename(f))
        if m: parts[(int(m.group(1)), int(m.group(2)))] = f
    Ns = {N for _, N in parts}
    if len(Ns) != 1: sys.exit(f'merge: expected the parts of one sharding in {pdir}, found {sorted(parts)}')
    N = Ns.pop(); missing = [k for k in range(N) if (k, N) not in parts]
    if missing: sys.exit(f'merge: parts {missing} of {N} are not finished yet')
    results, sig0, secs = [], None, 0.0
    for k in range(N):
        lines = open(parts[(k, N)]).read().splitlines()
        head = json.loads(lines[0]); sig = dict(head['journal']); sig.pop('shard', None)
        if sig0 is None: sig0 = sig
        elif sig != sig0: sys.exit(f'merge: part {k} is from a different job:\n  {sig}\n  {sig0}')
        secs += head.get('secs', 0.0); results += [json.loads(ln) for ln in lines[1:]]
    keys = [_key(r) for r in results]
    if len(set(keys)) != len(keys): sys.exit('merge: an episode appears in more than one part')
    write_summary(out, results, sig0['policy'], sig0['maps'], sig0['seed'], sig0['beta'], sig0['perturb'], secs,
                  sig0.get('car'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['eval', 'collect', 'merge'])
    ap.add_argument('--policy', default='expert'); ap.add_argument('--beta', type=float, default=0.0)
    ap.add_argument('--maps', default='levine,Spielberg_map'); ap.add_argument('--n', type=int, default=50)
    ap.add_argument('--seed', type=int, default=1000); ap.add_argument('--repeats', type=int, default=1)
    ap.add_argument('--perturb', default=None, choices=[None, 'lidar', 'camera', 'both', 'nocam'])
    ap.add_argument('--workers', type=int, default=16); ap.add_argument('--out', required=True)
    ap.add_argument('--shard', default='0/1', help='K/N: run only episodes K, K+N, K+2N, ... and write them to '
                    '<out>/parts/part-K-of-N.jsonl; `merge --out <out>` then writes episodes.jsonl + summary.json')
    ap.add_argument('--car', default=None, help='run under car conditions (ml/car_model.py): a preset, '
                    'e.g. car (the car with today\'s bridge), car_fixed, car_train')
    ap.add_argument('--car-set', action='append', default=[], metavar='KEY=VALUE',
                    help='override one car_model setting (JSON value), e.g. --car-set speed_map=\'"inverse"\'')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    if a.mode == 'merge':
        merge(a.out); return
    k, N = (int(x) for x in a.shard.split('/'))
    assert 0 <= k < N, f'bad --shard {a.shard}'
    car = None
    if a.car:
        import car_model
        if a.perturb: sys.exit('--perturb and --car do not mix: the car preset says what the sensors do')
        car = car_model.resolve(a.car, a.car_set); car['preset'] = a.car
    elif a.car_set:
        sys.exit('--car-set needs --car')
    tasks = get_tasks(a.maps.split(','), a.n, a.seed)
    record = a.mode == 'collect'
    jobs = [(t, a.beta, record, a.seed * 7919 + r * 104729 + i, a.perturb, a.out if record else None)
            + ((car,) if car else ()) for r in range(a.repeats) for i, t in enumerate(tasks)][k::N]
    t0 = time.time()
    pol = os.path.abspath(a.policy) if a.policy != 'expert' else 'expert'
    if pol != 'expert' and not os.path.isfile(pol):
        # otherwise every pool worker dies in _init and the pool respawns them forever
        sys.exit(f'policy file not found: {pol}')
    # Resume: finished episodes go to a journal (one fsync'd line each); a run started again with
    # the same arguments, after a SIGKILL say, keeps those and runs only the rest.
    pfile = None if pol == 'expert' else [os.path.getsize(pol), int(os.path.getmtime(pol))]
    sig = {'mode': a.mode, 'policy': pol, 'policy_file': pfile, 'maps': a.maps, 'n': a.n, 'seed': a.seed,
           'repeats': a.repeats, 'beta': a.beta, 'perturb': a.perturb, 'shard': a.shard}
    if car: sig['car'] = car
    if N == 1:
        jpath = os.path.join(a.out, 'episodes.partial.jsonl')
    else:
        pdir = os.path.join(a.out, 'parts'); os.makedirs(pdir, exist_ok=True)
        jpath = os.path.join(pdir, f'part-{k}-of-{N}.partial.jsonl')
    done = _read_journal(jpath, sig, a.out)
    _write_lines(jpath, [json.dumps({'journal': sig})] + [json.dumps(r) for r in done.values()])  # drops a torn line
    todo = [j for j in jobs if f"{j[0]['task_id']}|{j[3]}" not in done]
    if record:   # temp files of episodes a killed run did not finish
        for j in todo:
            for f in glob.glob(_episode_file(a.out, j[0], j[3]) + '.tmp.*'): os.remove(f)
    print(f'{len(jobs)} episodes: {len(done)} already done, {len(todo)} to run', flush=True)
    results = list(done.values())
    if todo:
        try:
            with open(jpath, 'a') as jf, ProcessPoolExecutor(min(a.workers, len(todo)), initializer=_init,
                                                             initargs=(pol,)) as ex:
                for fut in as_completed([ex.submit(_work, j) for j in todo]):
                    r = fut.result(); results.append(r)
                    jf.write(json.dumps(r) + '\n'); jf.flush(); os.fsync(jf.fileno())
        except BrokenProcessPool:
            # a worker was killed (SIGKILL, OOM killer): everything finished so far is in the journal
            print(f'a worker process died; {len(results)} of {len(jobs)} episodes are saved, '
                  f'run again to resume', file=sys.stderr, flush=True)
            sys.exit(75)
    secs = time.time() - t0
    if N > 1:
        results.sort(key=lambda r: (r['task_id'], r.get('ep_seed', 0)))
        _write_lines(os.path.join(pdir, f'part-{k}-of-{N}.jsonl'),
                     [json.dumps({'journal': sig, 'secs': round(secs, 1)})] + [json.dumps(r) for r in results])
        os.remove(jpath)
        print(f'part {k}/{N} done: {len(results)} episodes, {secs:.0f} s', flush=True)
        return
    write_summary(a.out, results, a.policy, a.maps, a.seed, a.beta, a.perturb, secs, car)
    os.remove(jpath)


if __name__ == '__main__':
    main()
