#!/usr/bin/env python3
"""shard_run: the extended route-hint run (the recipe of ml/ws_route_extended.sh) as small,
restartable shards, for a shared machine where any process can be killed at any moment
(unicron, the CSL's 6-GPU node). Standard library only, so it runs under the system python3;
the shards themselves run with the ML venv (config key `py`).

The run is a set of shards with dependencies:
  collect:<set>:<k>     expert demos, every N-th episode of a demo set (sim_rollout --shard k/N)  CPU
  merge:<set>           that set's episodes.jsonl + summary.json, from its parts                    CPU
  cache                 the training data cache, built once for every seed                          CPU
  train:s<seed>         one seed (train_policy.py; checkpoint every few minutes)                    1 GPU
  eval:s<seed>:<name>   closed-loop evals of that seed's student.onnx                               CPU
  select                seed chosen on the selection set -> <runs>/route/{student.onnx,choice.json}
Shards that do not depend on each other run side by side (the seeds on separate GPUs) within a
CPU-worker and GPU budget.

Why a SIGKILL costs at most a few minutes:
  * a shard is done when its output exists, and every output is written to a temp file and
    renamed into place, so a kill never leaves a half-written output that looks finished;
  * a killed shard is simply started again and resumes: episodes it already recorded are kept,
    and training continues from its last checkpoint (`ckpt_mins` apart);
  * each running shard holds its own lock file, so one left running by a killed orchestrator is
    recognised and never started twice;
  * the orchestrator itself runs as a systemd user service (Restart=on-failure, so a SIGKILL
    brings it back 15 s later) with a timer that starts it every 2 minutes if it is not running.
    The account must linger (`loginctl enable-linger`) so the user manager runs without a login.
    The unit files go in $XDG_RUNTIME_DIR/systemd/user, not ~/.config: on the CSL the home is
    Kerberos NFS, unreadable without a ticket. (cron would be the classic choice, but crontab
    entries of this account never ran on unicron when tested, Oct 2026.) A shard killed again and again is
    restarted with growing delays (10 s doubling to 10 min), so the run never fights whoever
    keeps killing it in a tight loop.
What it does not survive: a reboot, or a kill of every process of the account including the
systemd user manager. Nothing is lost then either; `install` again and it carries on.

    python3 ml/shard_run.py install --root DIR --set py=/path/to/venv/bin/python [--set key=json ...]
    python3 ml/shard_run.py status  --root DIR
    python3 ml/shard_run.py stop    --root DIR      # stops the shards (training checkpoints first) and the timer
DIR holds the code (DIR/ml, DIR/f1tenth_gym_ros, DIR/maps, ...); data/ and runs_ext/ are made there.
"""
import argparse, fcntl, glob, hashlib, json, math, os, re, shutil, signal, subprocess, sys, time

FLOCK, NICE, IONICE, NVSMI = '/usr/bin/flock', '/usr/bin/nice', '/usr/bin/ionice', 'nvidia-smi'
BUSY = 99          # flock's exit code when the shard's lock is already held (it is running elsewhere)
LOST_WORKER = 75   # sim_rollout: a pool worker died (killed); finished episodes are saved
PRIO = {'train': 0, 'collect': 1, 'merge': 1, 'cache': 1, 'eval': 2, 'select': 3}

DEFAULTS = {
    'py': None,                     # python of the ML venv (torch, onnxruntime): required
    'home': None,                   # HOME for the shards (local disk); default: <root>/../home
    'maps': 'levine,Spielberg_map,comp_track,my_track',
    'eval_extra_maps': 'uploadtest',
    # demo sets: [dir, tasks per map, task seed] = the MORE=1 prep of ml/run_route.sh (2400 episodes)
    'sets': [['data/rdemos', 100, 0], ['data/rdemos_extra', 200, 5000], ['data/rdemos_more', 300, 9000]],
    'shard_episodes': 50,           # episodes per collect shard
    'collect_workers': 8,           # processes per collect shard
    'runs': 'runs_ext',
    'seeds': [0, 1, 2, 3, 4],
    'epochs': 50,
    'ckpt_mins': 5,
    'train_flags': ['--cam-aug', '0', '--cam-drop', '0', '--beam-drop', '0', '--state-mask', '0,0,1,1,0'],
    'neval': 30,                    # eval tasks per map
    'evals': [['eval_sel', 3000, 'none'], ['eval_none', 1000, 'none'], ['eval_lidar', 1000, 'lidar'],
              ['eval_camera', 1000, 'camera'], ['eval_nocam', 1000, 'nocam']],
    'eval_workers': 16,
    'cache_workers': 16,
    'cpu_slots': 64,                # our CPU workers at once (unicron: 96 threads, shared)
    'train_cpu': 4,                 # CPU slots counted per training shard
    'gpus': 'auto',                 # 'auto' or a list of GPU indices we may use
    'max_gpus': 5,                  # training shards at once (one GPU each)
    'gpu_free_mib': 2000,           # a GPU counts as free below this much memory in use
    'nice': 10,
    'max_crashes': 4,               # a shard that fails by itself (not killed) this often is given up
    # ---- optional, added 10/5 for the car-conditions runs (defaults = the recipe above, unchanged)
    'train_data': None,             # data dirs to train on (default: the sets' dirs); absolute or root-relative
    'variants': None,               # [[name, train_flags, seeds], ...]; default [['route', train_flags, seeds]]
    'external': [],                 # [[label, onnx path], ...]: models trained elsewhere, evaluated (not trained) here
    'select': ['eval_sel', 'route'],  # [eval that ranks the models, <runs>/<dir> for the winner]
    'select_external': False,       # may an external model win the selection?
    'select_variants': None,        # only these variants may win (default: any)
    'dagger': None,                 # second stage, see plan_dagger()
}


def now(): return time.strftime('%Y-%m-%d %H:%M:%S')


def log(msg): print(f'[{now()}] {msg}', flush=True)


def fmt(secs):
    secs = int(secs); h, m = secs // 3600, secs % 3600 // 60
    return f'{h}h{m:02d}m' if h else f'{m}m{secs % 60:02d}s'


def atomic_text(path, text):
    tmp = f'{path}.tmp{os.getpid()}'
    with open(tmp, 'w') as f:
        f.write(text); f.flush(); os.fsync(f.fileno())
    os.replace(tmp, path)


def load_json(path, default):
    try:
        with open(path) as f: return json.load(f)
    except (FileNotFoundError, ValueError):
        return default


def paths(root):
    st = os.path.join(root, 'shard_state')
    return {'st': st, 'locks': os.path.join(st, 'locks'), 'logs': os.path.join(root, 'logs', 'shard'),
            'cfg': os.path.join(st, 'config.json'), 'state': os.path.join(st, 'state.json'),
            'status': os.path.join(st, 'status.txt'), 'pid': os.path.join(st, 'orchestrator.pid'),
            'lock': os.path.join(st, 'orchestrator.lock')}


def fname(task): return task.replace(':', '_')


def lock_held(path):
    """True if some process holds the flock on path (the shard, or the orchestrator, is running)."""
    try: fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o600)
    except OSError: return False
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB); fcntl.flock(fd, fcntl.LOCK_UN); return False
    except BlockingIOError:
        return True
    finally:
        os.close(fd)


def count_lines(path):
    try:
        with open(path) as f: return max(0, sum(1 for _ in f) - 1)      # minus the journal header
    except FileNotFoundError:
        return 0


class Task:
    def __init__(self, name, kind, cmd=None, out=None, deps=(), slots=1, gpu=False, inline=None, progress=None, rank=0):
        self.name, self.kind, self.cmd, self.out, self.deps = name, kind, cmd, out, list(deps)
        self.slots, self.gpu, self.inline, self.progress, self.rank = slots, gpu, inline, progress, rank

    def done(self): return os.path.isfile(self.out)


def _entry(e):
    """A set or eval entry: its fixed fields and the optional list of extra sim_rollout args."""
    return list(e[:3]) + [list(e[3]) if len(e) > 3 else []]


def _collect_tasks(root, C, T, sets, deps=(), base_rank=0):
    """collect + merge shards for demo (or DAgger) sets; returns the merge task names. Extra
    sim_rollout args of a set come after the defaults, so e.g. '--policy x --beta 0.3' wins."""
    py, ml = C['py'], os.path.join(root, 'ml'); maps = C['maps']; nmaps = len(maps.split(','))
    merges = []
    for si, e in enumerate(sets):
        d, n, seed, extra = _entry(e)
        out = os.path.join(root, d); tag = os.path.basename(d.rstrip('/'))
        neps = n * nmaps; N = max(1, math.ceil(neps / C['shard_episodes'])); parts = []
        for k in range(N):
            name = f'collect:{tag}:{k}'; parts.append(name)
            # rank interleaves the sets (shard 0 of every set first), so the route planning of
            # every set's task list (get_tasks, minutes for a big set) starts right away, in parallel
            T[name] = Task(name, 'collect', [py, f'{ml}/sim_rollout.py', 'collect', '--policy', 'expert', '--beta', '1',
                                             '--maps', maps, '--n', str(n), '--seed', str(seed), '--workers',
                                             str(C['collect_workers']), '--out', out, '--shard', f'{k}/{N}'] + extra,
                           out=f'{out}/parts/part-{k}-of-{N}.jsonl', deps=deps, slots=C['collect_workers'],
                           rank=base_rank + k * 1000 + si,
                           progress=(lambda p=f'{out}/parts/part-{k}-of-{N}.partial.jsonl', tot=len(range(k, neps, N)):
                                     f'{count_lines(p)}/{tot} episodes'))
        T[f'merge:{tag}'] = Task(f'merge:{tag}', 'merge', [py, f'{ml}/sim_rollout.py', 'merge', '--out', out],
                                 out=f'{out}/summary.json', deps=parts)
        merges.append(f'merge:{tag}')
    return merges


def _eval_tasks(root, C, T, label, run, onnx, deps):
    """The closed-loop evals (config `evals`) of one model; outputs in <run>/<eval name>/."""
    py, ml = C['py'], os.path.join(root, 'ml'); maps = C['maps']; nmaps = len(maps.split(','))
    names = []
    for e in C['evals']:
        name, tseed, pert, extra = _entry(e)
        en = f'eval:{label}:{name}'; names.append(en); out = f'{run}/{name}'
        cmd = [py, f'{ml}/sim_rollout.py', 'eval', '--policy', onnx, '--maps',
               f"{maps},{C['eval_extra_maps']}", '--n', str(C['neval']), '--seed', str(tseed),
               '--workers', str(C['eval_workers']), '--out', out]
        if pert != 'none': cmd += ['--perturb', pert]
        cmd += extra
        tot = C['neval'] * (nmaps + len(C['eval_extra_maps'].split(',')))
        T[en] = Task(en, 'eval', cmd, out=f'{out}/summary.json', deps=deps, slots=C['eval_workers'],
                     progress=lambda p=f'{out}/episodes.partial.jsonl', tot=tot: f'{count_lines(p)}/{tot} episodes')
    return names


def _model_tasks(root, C, T, runs, data, variants, cache, epochs):
    """One training shard per variant and seed (one GPU each), then its evals.
    Returns ([(run dir, onnx, variant)], eval task names). Variant 'route' keeps the original names
    (train:s<seed>, runs_ext/route_s<seed>)."""
    py, ml = C['py'], os.path.join(root, 'ml')
    models, evals = [], []
    for vname, flags, seeds in variants:
        for s in seeds:
            label = f's{s}' if vname == 'route' else f'{vname}_s{s}'
            run = f'{runs}/{vname}_s{s}'; tn = f'train:{label}'
            if tn in T: sys.exit(f'two models would both be called {label}')
            T[tn] = Task(tn, 'train', [py, f'{ml}/train_policy.py', '--data', *data, '--out', run, '--epochs', str(epochs),
                                       '--seed', str(s), '--ckpt-mins', str(C['ckpt_mins'])] + list(flags),
                         out=f'{run}/student.onnx', deps=[cache], slots=C['train_cpu'], gpu=True,
                         progress=lambda run=run, ep=epochs: f"epoch {count_lines(f'{run}/log.jsonl') + 1}/{ep}")
            models.append((run, f'{run}/student.onnx', vname))
            evals += _eval_tasks(root, C, T, label, run, f'{run}/student.onnx', [tn])
    return models, evals


def plan(root, C):
    """Every shard of the run, dependencies before dependents."""
    py, ml = C['py'], os.path.join(root, 'ml')
    T = {}
    merges = _collect_tasks(root, C, T, C['sets'])
    runs = os.path.join(root, C['runs'])
    data = [os.path.join(root, d) for d in (C['train_data'] or [e[0] for e in C['sets']])]
    T['cache'] = Task('cache', 'cache', [py, f'{ml}/train_policy.py', '--data', *data, '--out', runs, '--build-cache-only',
                                         '--cache-workers', str(C['cache_workers'])],
                      out=f'{runs}/cache.json', deps=merges, slots=C['cache_workers'])
    variants = C['variants'] or [['route', C['train_flags'], C['seeds']]]
    models, evals = _model_tasks(root, C, T, runs, data, variants, 'cache', C['epochs'])
    external = []
    for label, onnx in C['external']:
        run = f'{runs}/{label}'
        if any(run == m[0] for m in models): sys.exit(f'external model {label} clashes with a trained one')
        external.append((run, os.path.join(root, onnx), None))
        evals += _eval_tasks(root, C, T, label, run, os.path.join(root, onnx), [])
    sel = list(C['select']) + ['eval_none'][len(C['select']) - 2:]
    cand = candidates(C, models, external)
    T['select'] = Task('select', 'select', out=f'{runs}/{sel[1]}/choice.json', deps=evals,
                       inline=lambda: select(root, runs, cand, sel[0], sel[1], sel[2]))
    if C['dagger']: plan_dagger(root, C, T, runs, data, models, external, sel)
    return T


def plan_dagger(root, C, T, runs, data, models, external, sel):
    """Second stage (config `dagger`, a dict): DAgger sets collected with the first stage's chosen
    model driving (--policy <runs>/<select dir>/student.onnx --beta <beta>, labels from the expert),
    a data cache of the first stage's data plus these sets, the `variants` trained on it, the same
    evals, and a final selection over BOTH stages' models, so DAgger has to beat the first stage.
      {'sets': [[dir, tasks per map, task seed(, extra args)]], 'beta': 0.3, 'variants': [[name, flags, seeds]],
       'epochs': 40, 'train_data': [dirs] (default: first-stage data + the sets), 'select': [eval, dir(, std eval)]}"""
    py, ml = C['py'], os.path.join(root, 'ml'); D = C['dagger']
    pol = f'{runs}/{sel[1]}/student.onnx'
    sets = [[e[0], e[1], e[2], ['--policy', pol, '--beta', str(D.get('beta', 0.3))] + _entry(e)[3]] for e in D['sets']]
    merges = _collect_tasks(root, C, T, sets, deps=['select'], base_rank=100)
    data2 = [os.path.join(root, d) for d in D['train_data']] if D.get('train_data') else data + [os.path.join(root, e[0]) for e in D['sets']]
    cache_out = f'{runs}/dagger_cache'
    T['cache:dagger'] = Task('cache:dagger', 'cache', [py, f'{ml}/train_policy.py', '--data', *data2, '--out', cache_out,
                                                       '--build-cache-only', '--cache-workers', str(C['cache_workers'])],
                             out=f'{cache_out}/cache.json', deps=merges, slots=C['cache_workers'])
    models2, evals2 = _model_tasks(root, C, T, runs, data2, D['variants'], 'cache:dagger', D.get('epochs', C['epochs']))
    sel2 = list(D.get('select', sel[:2])) + [sel[2]][len(D.get('select', sel[:2])) - 2:]
    cand = candidates(C, models + models2, external)
    T['select:dagger'] = Task('select:dagger', 'select', out=f'{runs}/{sel2[1]}/choice.json', deps=evals2 + ['select'],
                              inline=lambda: select(root, runs, cand, sel2[0], sel2[1], sel2[2]))


def candidates(C, models, external):
    """The models a selection may pick: trained ones of the allowed variants, external ones if allowed."""
    ok = [m for m in models if C['select_variants'] is None or m[2] in C['select_variants']]
    return [(r, o) for r, o, _ in ok + (external if C['select_external'] else [])]


def select(root, runs, models, sel_eval='eval_sel', out_dir='route', std_eval='eval_none'):
    """ml/run_route.sh select, with atomic outputs: rank the models on the selection eval (by
    default the task-seed-3000 set) by success, then fewest collisions, then progress, and copy the
    winner to <runs>/<out_dir>/student.onnx; choice.json is written last. models: [(run dir, onnx)]."""
    rows = []
    for d, onnx in models:
        f = os.path.join(d, sel_eval, 'summary.json')
        if os.path.isfile(f) and os.path.isfile(onnx):
            s = load_json(f, None); rel = os.path.relpath(d, root)
            rows.append((s['success_rate'], -s['collision_rate'], s['mean_progress'], rel, onnx))
    if not rows: raise RuntimeError(f'no model has a {sel_eval} result')
    rows.sort(key=lambda r: r[:4], reverse=True)
    for r in rows:
        log(f"  {r[3]}: {sel_eval} success {100 * r[0]:.1f}%  collisions {-100 * r[1]:.1f}%  progress {100 * r[2]:.1f}%")
    best, best_onnx = rows[0][3], rows[0][4]; os.makedirs(f'{runs}/{out_dir}', exist_ok=True)
    tmp = f'{runs}/{out_dir}/student.onnx.tmp{os.getpid()}'
    shutil.copyfile(best_onnx, tmp)
    fd = os.open(tmp, os.O_RDONLY); os.fsync(fd); os.close(fd)
    os.replace(tmp, f'{runs}/{out_dir}/student.onnx')
    std = load_json(os.path.join(root, best, std_eval, 'summary.json'), None)
    atomic_text(f'{runs}/{out_dir}/choice.json', json.dumps(
        {'chosen': best, 'selection_eval': sel_eval, 'selection_ranking': [[r[3], r[0], -r[1], r[2]] for r in rows],
         'standard_eval': std}, indent=1))
    if std:
        log(f"CHOSEN {best}: {std_eval} success {100 * std['success_rate']:.1f}%  collisions {100 * std['collision_rate']:.1f}%")
    else:
        log(f'CHOSEN {best} (no {std_eval} result)')


def child_env(C, t, gpu):
    """A clean environment: HOME on local disk (the NFS home needs a Kerberos ticket that services
    do not have), one BLAS/OpenMP thread per sim worker, no GPU for CPU shards."""
    home = C['home']
    env = {'PATH': '/usr/local/bin:/usr/bin:/bin', 'HOME': home, 'LANG': 'C.UTF-8',
           'USER': os.environ.get('USER') or os.environ.get('LOGNAME', ''), 'LOGNAME': os.environ.get('LOGNAME', ''),
           'PYTHONUNBUFFERED': '1', 'PYTHONNOUSERSITE': '1', 'TMPDIR': f'{home}/tmp',
           'XDG_CACHE_HOME': f'{home}/.cache', 'MPLCONFIGDIR': f'{home}/.config/matplotlib',
           'OMP_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1', 'NUMEXPR_NUM_THREADS': '1',
           'CUDA_VISIBLE_DEVICES': ''}
    if t.gpu:
        env['CUDA_VISIBLE_DEVICES'] = str(gpu); env['OMP_NUM_THREADS'] = str(C['train_cpu'])
    return env


def free_gpus(C):
    """GPU indices we may use that have (almost) no memory in use right now."""
    try:
        out = subprocess.run([NVSMI, '--query-gpu=index,memory.used', '--format=csv,noheader,nounits'],
                             capture_output=True, text=True, timeout=60).stdout
    except Exception as e:
        log(f'nvidia-smi failed: {e!r}'); return []
    allowed = None if C['gpus'] == 'auto' else {int(g) for g in C['gpus']}
    free = []
    for ln in out.strip().splitlines():
        try: i, used = (int(x) for x in ln.split(','))
        except ValueError: continue
        if (allowed is None or i in allowed) and used < C['gpu_free_mib']: free.append(i)
    return free


def group_alive(pgid):
    try: os.killpg(pgid, 0); return True
    except (ProcessLookupError, PermissionError): return False


def orchestrate(root, C, P):
    T = plan(root, C)
    state = load_json(P['state'], {})
    for n in T: state.setdefault(n, {'attempts': 0, 'crashes': 0})
    procs = {}                                    # shards this orchestrator started: name -> (Popen, log file)
    stop = []
    signal.signal(signal.SIGTERM, lambda s, f: stop.append(s)); signal.signal(signal.SIGINT, lambda s, f: stop.append(s))
    signal.signal(signal.SIGHUP, signal.SIG_IGN)
    lockpath = lambda name: os.path.join(P['locks'], fname(name) + '.lock')
    save = lambda: atomic_text(P['state'], json.dumps(state, indent=1))
    log(f'orchestrator {os.getpid()} up: {len(T)} shards, {sum(t.done() for t in T.values())} already done')

    def launch(t, gpu):
        s = state[t.name]; s['attempts'] += 1; s['t_start'] = time.time(); s['started'] = now(); s['gpu'] = gpu
        lf = open(os.path.join(P['logs'], fname(t.name) + '.log'), 'a')
        lf.write(f"\n===== {now()} attempt {s['attempts']}" + (f' on GPU {gpu}' if gpu is not None else '')
                 + f": {' '.join(t.cmd)}\n"); lf.flush()
        cmd = [FLOCK, '-n', '-E', str(BUSY), lockpath(t.name), NICE, '-n', str(C['nice']), IONICE, '-c2', '-n7'] + t.cmd
        p = subprocess.Popen(cmd, cwd=root, env=child_env(C, t, gpu), stdin=subprocess.DEVNULL, stdout=lf,
                             stderr=subprocess.STDOUT, start_new_session=True)
        s['pgid'] = p.pid; procs[t.name] = (p, lf); save()
        log(f"{t.name}: started (attempt {s['attempts']}" + (f', GPU {gpu}' if gpu is not None else '') + ')')

    def shutdown(grace=120):
        """Polite stop: SIGTERM to every shard we started (training writes a checkpoint first)."""
        for name, (p, lf) in procs.items():
            try: os.killpg(p.pid, signal.SIGTERM)
            except ProcessLookupError: pass
        t0 = time.time()
        while time.time() - t0 < grace and any(group_alive(p.pid) for p, _ in procs.values()): time.sleep(1)
        for name, (p, lf) in procs.items():
            if group_alive(p.pid):
                try: os.killpg(p.pid, signal.SIGKILL)
                except ProcessLookupError: pass
            p.poll(); lf.close()

    while True:
        if stop or os.path.exists(os.path.join(P['st'], 'STOP')):
            log(f'stopping ({len(procs)} shards of ours running)'); shutdown(); save(); log('stopped'); return
        # ---- shards that ended
        for name in list(procs):
            p, lf = procs[name]; rc = p.poll()
            if rc is None: continue
            lf.close(); del procs[name]; s = state[name]; s['last_rc'] = rc; s['ended'] = now()
            dur = fmt(time.time() - s.get('t_start', time.time()))
            if T[name].done():
                log(f'{name}: done after {dur}' + (f" (attempt {s['attempts']})" if s['attempts'] > 1 else ''))
            elif rc == BUSY:
                log(f'{name}: already running (started by an earlier orchestrator); leaving it be')
            else:
                crash = rc == 0 or (0 < rc < 128 and rc != LOST_WORKER)   # failed by itself, not killed
                if crash:
                    s['crashes'] += 1; delay = 60
                else:      # killed: restart soon, but back off if it keeps being killed (10 s doubling to 10 min)
                    s['kills'] = [k for k in s.get('kills', []) if time.time() - k < 1800] + [time.time()]
                    delay = min(600, 10 * 2 ** (len(s['kills']) - 1))
                s['retry_at'] = time.time() + delay
                why = 'failed' if crash else 'was killed' if (rc < 0 or rc >= 128) else 'lost a worker process'
                log(f'{name}: {why} (exit {rc}) after {dur}; ' + (
                    f"giving up after {s['crashes']} failures, see logs/shard/{fname(name)}.log"
                    if s['crashes'] >= C['max_crashes'] else f'restarting it in {delay} s'))
            save()
        # ---- where every shard stands
        st = {}
        for name, t in T.items():
            if t.done(): st[name] = 'done'
            elif name in procs: st[name] = 'running'
            elif lock_held(lockpath(name)): st[name] = 'orphan'        # left running by a killed orchestrator
            elif state[name]['crashes'] >= C['max_crashes']: st[name] = 'failed'
            elif any(st[d] in ('failed', 'blocked') for d in t.deps): st[name] = 'blocked'
            elif all(st[d] == 'done' for d in t.deps): st[name] = 'ready'
            else: st[name] = 'waiting'
        write_status(root, C, P, T, st, state)
        if all(v == 'done' for v in st.values()):
            atomic_text(os.path.join(P['st'], 'DONE'), now() + '\n'); timer_off(root)
            final = (C['dagger'] or {}).get('select', C['select'])[1] if C['dagger'] else C['select'][1]
            log(f"ALL DONE: {os.path.join(C['runs'], final, 'choice.json')}"); return
        active = [n for n, v in st.items() if v in ('running', 'orphan')]
        ready = [n for n, v in st.items() if v == 'ready']
        if not active and not ready:
            atomic_text(os.path.join(P['st'], 'FAILED'), now() + '\n'); timer_off(root)
            log('FAILED: ' + ', '.join(n for n, v in st.items() if v == 'failed') + ' gave up; nothing else can run'); return
        # ---- start what can start, within the CPU and GPU budget
        cpu = sum(T[n].slots for n in active)
        gpus_ours = {state[n].get('gpu') for n in active if T[n].gpu}
        ntrain = sum(1 for n in active if T[n].gpu); free = None
        order = {n: i for i, n in enumerate(T)}
        for name in sorted(ready, key=lambda n: (PRIO[T[n].kind], T[n].rank, order[n])):
            t = T[name]; s = state[name]
            if time.time() < s.get('retry_at', 0): continue
            if t.inline:
                try:
                    t.inline(); log(f'{name}: done')
                except Exception as e:
                    s['crashes'] += 1; s['retry_at'] = time.time() + 60; log(f'{name}: failed: {e!r}')
                save(); continue
            if cpu > 0 and cpu + t.slots > C['cpu_slots']: continue
            gpu = None
            if t.gpu:
                if ntrain >= C['max_gpus']: continue
                if free is None: free = [g for g in free_gpus(C) if g not in gpus_ours]
                if not free: continue
                gpu = free.pop(0); gpus_ours.add(gpu); ntrain += 1
            launch(t, gpu); cpu += t.slots
        for _ in range(10):
            if stop: break
            time.sleep(1)


def write_status(root, C, P, T, st, state):
    kinds = ['collect', 'merge', 'cache', 'train', 'eval', 'select']
    lines = [f'shard_run {root}', f'updated {now()}   orchestrator pid {os.getpid()}', '',
             f"{'stage':8s} {'shards':>6s} {'done':>5s} {'running':>8s} {'to do':>6s} {'failed':>7s}"]
    for k in kinds:
        ns = [n for n in T if T[n].kind == k]
        c = lambda *v: sum(1 for n in ns if st[n] in v)
        lines.append(f"{k:8s} {len(ns):6d} {c('done'):5d} {c('running', 'orphan'):8d} "
                     f"{c('ready', 'waiting'):6d} {c('failed', 'blocked'):7d}")
    run = [n for n in T if st[n] in ('running', 'orphan')]
    if run:
        lines += ['', 'running:']
        for n in run:
            s = state[n]; extra = T[n].progress() if T[n].progress else ''
            gpu = f"GPU {s.get('gpu')}" if T[n].gpu else ''
            since = fmt(time.time() - s['t_start']) if s.get('t_start') else '?'
            lines.append(f"  {n:26s} attempt {s.get('attempts', 0):<2d} {gpu:6s} {extra:22s} {since}"
                         + ('  (left from an earlier orchestrator)' if st[n] == 'orphan' else ''))
    bad = [n for n in T if st[n] == 'failed']
    if bad: lines += ['', 'failed (see logs/shard/<name>.log): ' + ', '.join(bad)]
    restarts = {n: state[n]['attempts'] - 1 for n in T if state[n].get('attempts', 0) > 1}
    if restarts: lines += ['', 'restarted: ' + ', '.join(f'{n} x{k}' for n, k in restarts.items())]
    atomic_text(P['status'], '\n'.join(lines) + '\n')


# ---------------------------------------------------------------- supervisor (systemd user units), commands

def unit(root):
    return 'atlas-shard-' + re.sub(r'[^A-Za-z0-9_-]', '-', os.path.basename(root))[:30] + '-' \
        + hashlib.sha1(root.encode()).hexdigest()[:6]


def unit_dir():
    return os.path.join(os.environ.get('XDG_RUNTIME_DIR') or f'/run/user/{os.getuid()}', 'systemd', 'user')


def systemctl(*args):
    env = dict(os.environ); env.setdefault('XDG_RUNTIME_DIR', f'/run/user/{os.getuid()}')
    return subprocess.run(['systemctl', '--user', *args], capture_output=True, text=True, env=env)


def supervisor_install(root, C):
    """<unit>.service runs `tick` (the orchestrator) and is restarted if it is killed by a signal;
    <unit>.timer starts it every 2 minutes if it is not running. Runtime units (gone at reboot)."""
    name, d, logf = unit(root), unit_dir(), f'{root}/logs/shard/orchestrator.log'
    os.makedirs(d, exist_ok=True)
    atomic_text(f'{d}/{name}.service', f"""[Unit]
Description=shard_run orchestrator for {root}
StartLimitIntervalSec=900
StartLimitBurst=8

[Service]
Type=simple
ExecStart=/usr/bin/python3 {os.path.abspath(__file__)} tick --root {root}
WorkingDirectory={C['home']}
Environment=HOME={C['home']} PYTHONNOUSERSITE=1 LANG=C.UTF-8 PATH=/usr/local/bin:/usr/bin:/bin
# killed by a signal (SIGKILL, OOM, ...) -> started again; a clean exit (finished, stopped) is final
Restart=on-failure
RestartSec=15
# only the orchestrator itself: the shards it started must outlive it, and the next orchestrator
# adopts them (each holds its own lock file) instead of starting them twice
KillMode=process
StandardOutput=append:{logf}
StandardError=append:{logf}
""")
    atomic_text(f'{d}/{name}.timer', f"""[Unit]
Description=keep the shard_run orchestrator for {root} running

[Timer]
OnActiveSec=3
OnCalendar=*:0/2
AccuracySec=5s

[Install]
WantedBy=timers.target
""")
    for args in (('daemon-reload',), ('enable', '--runtime', '--now', f'{name}.timer')):
        r = systemctl(*args)
        if r.returncode: sys.exit(f'systemctl --user {" ".join(args)} failed: {r.stderr.strip()}')


def timer_off(root):
    """Stop starting the orchestrator (the run is over, or stopped). The service unit stays until reboot."""
    name = unit(root)
    systemctl('disable', '--runtime', '--now', f'{name}.timer')
    try: os.remove(f'{unit_dir()}/{name}.timer')
    except FileNotFoundError: pass
    systemctl('daemon-reload')


def tick(root):
    """Become the orchestrator unless one is running or the run has ended (the systemd service)."""
    P = paths(root)
    if any(os.path.exists(os.path.join(P['st'], m)) for m in ('DONE', 'FAILED', 'STOP')): return
    fd = os.open(P['lock'], os.O_RDWR | os.O_CREAT, 0o600)        # held for this process's lifetime
    try: fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError: return
    atomic_text(P['pid'], f'{os.getpid()}\n')
    C = dict(DEFAULTS); C.update(load_json(P['cfg'], {}))
    C['home'] = C['home'] or os.path.join(os.path.dirname(root), 'home')
    for d in (P['locks'], P['logs'], C['home'], f"{C['home']}/tmp"): os.makedirs(d, exist_ok=True)
    orchestrate(root, C, P)


def install(root, sets, restart):
    P = paths(root)
    for d in (P['st'], P['locks'], P['logs']): os.makedirs(d, exist_ok=True)
    C = dict(DEFAULTS); C.update(load_json(P['cfg'], {}))
    for kv in sets:
        k, v = kv.split('=', 1)
        if k not in DEFAULTS: sys.exit(f'unknown setting {k}; settings: {", ".join(DEFAULTS)}')
        try: C[k] = json.loads(v)
        except ValueError: C[k] = v
    if not C['py'] or not os.path.isfile(C['py']): sys.exit('set py=<python of the ML venv>')
    C['home'] = C['home'] or os.path.join(os.path.dirname(root), 'home')
    for d in (C['home'], f"{C['home']}/tmp"): os.makedirs(d, exist_ok=True)
    atomic_text(P['cfg'], json.dumps(C, indent=1))
    for m in ('STOP', 'FAILED') + (('DONE',) if restart else ()):
        if os.path.exists(os.path.join(P['st'], m)): os.remove(os.path.join(P['st'], m))
    linger = subprocess.run(['loginctl', 'show-user', str(os.getuid()), '-p', 'Linger'], capture_output=True, text=True)
    if 'Linger=yes' not in linger.stdout:
        print('note: this account does not linger, so the run stops when you log out; `loginctl enable-linger` fixes that')
    if lock_held(P['lock']):
        print('note: an orchestrator is already running; config changes apply when it next restarts')
    supervisor_install(root, C)
    print(f'installed: systemd user units {unit(root)}.service/.timer (orchestrator starts in a few seconds)')
    print(f'config:    {P["cfg"]}')
    print(f'status:    python3 {os.path.abspath(__file__)} status --root {root}')


def ours(root):
    """Our processes that belong to this run (command line mentions a script in root/ml), except
    this one and its parents."""
    skip = set(); p = os.getpid()
    while p > 1:
        skip.add(p)
        try: p = int(open(f'/proc/{p}/stat').read().rsplit(')', 1)[1].split()[1])
        except (OSError, ValueError, IndexError): break
    pids = []
    for d in os.listdir('/proc'):
        if not d.isdigit() or int(d) in skip: continue
        try:
            if os.stat(f'/proc/{d}').st_uid != os.getuid(): continue
            cmd = open(f'/proc/{d}/cmdline', 'rb').read().decode(errors='replace').replace('\0', ' ')
            if f'{root}/ml/' in cmd: pids.append(int(d))
        except OSError:
            continue
    return pids


def stop(root):
    P = paths(root)
    atomic_text(os.path.join(P['st'], 'STOP'), now() + '\n'); timer_off(root)
    print('timer off; asking the orchestrator to stop (training shards write a checkpoint first) ...')
    systemctl('stop', '--no-block', f'{unit(root)}.service')      # SIGTERM to the orchestrator only
    try:
        pid = int(open(P['pid']).read())
        if lock_held(P['lock']): os.kill(pid, signal.SIGTERM)       # in case it was started by hand
    except (OSError, ValueError):
        pass
    t0 = time.time()
    while lock_held(P['lock']) and time.time() - t0 < 180: time.sleep(2)
    left = ours(root)                     # shards left running by an earlier, killed orchestrator
    for sig, wait in ((signal.SIGTERM, 120), (signal.SIGKILL, 10)):
        for pid in left:
            try: os.kill(pid, sig)
            except ProcessLookupError: pass
        t0 = time.time()
        while time.time() - t0 < wait and any(os.path.exists(f'/proc/{p}') for p in left): time.sleep(1)
        left = [p for p in left if os.path.exists(f'/proc/{p}')]
        if not left: break
    print('stopped. Everything finished so far is kept; `install` again to continue where it left off.')


def status(root):
    P = paths(root)
    try: print(open(P['status']).read(), end='')
    except FileNotFoundError: print('no status yet')
    marks = [m for m in ('DONE', 'FAILED', 'STOP') if os.path.exists(os.path.join(P['st'], m))]
    print(f"\norchestrator: {'running' if lock_held(P['lock']) else 'not running'}"
          + (f"   markers: {' '.join(marks)}" if marks else '')
          + f"   timer: {systemctl('is-active', unit(root) + '.timer').stdout.strip() or '?'}")
    try:
        with open(os.path.join(P['logs'], 'orchestrator.log')) as f: tail = f.readlines()[-12:]
        print('\nlast orchestrator log lines:\n' + ''.join('  ' + ln for ln in tail), end='')
    except FileNotFoundError:
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('cmd', choices=['install', 'tick', 'status', 'stop'])
    ap.add_argument('--root', required=True, help='experiment directory (holds ml/, data/, runs_ext/)')
    ap.add_argument('--set', action='append', default=[], metavar='KEY=VALUE', help='install: config setting (JSON value)')
    ap.add_argument('--restart', action='store_true', help='install: also clear a DONE marker')
    a = ap.parse_args(); root = os.path.abspath(a.root)
    if a.cmd == 'install': install(root, a.set, a.restart)
    elif a.cmd == 'tick': tick(root)
    elif a.cmd == 'status': status(root)
    else: stop(root)


if __name__ == '__main__':
    main()
