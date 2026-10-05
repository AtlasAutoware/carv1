#!/usr/bin/env python3
"""SIGKILL drill for ml/shard_run.py, on a tiny configuration (a few minutes on unicron).

    python3 ml/unicron/kill_test.py ROOT PY      # ROOT: a scratch copy of the repo; PY: the ML venv's python

Installs the run (systemd user service + timer) and then kills, in order:
  1. the orchestrator while collect shards run: systemd must start a new one, and the shards it
     left running must not be started a second time;
  2. one collect shard's whole process group after it has recorded an episode: it must restart and
     keep the episodes it had;
  3. a training process (python only, mid-epoch): it must resume from its last checkpoint;
  4. every process of the run at once -- orchestrator, shards and pool workers -- while evals run:
     the run must come back and finish.
Meanwhile it watches that no shard ever runs twice at once. Then ml/unicron/verify_run.py checks
every output. Exit status 0 = all passed.
"""
import glob, json, os, signal, subprocess, sys, time

ROOT, PY = os.path.abspath(sys.argv[1]), sys.argv[2]
SR = f'{ROOT}/ml/shard_run.py'


def log(m): print(f'[{time.strftime("%H:%M:%S")}] {m}', flush=True)


def snapshot():
    """our processes: pid -> (ppid, process group, command line)"""
    P = {}
    for d in os.listdir('/proc'):
        if not d.isdigit(): continue
        try:
            if os.stat('/proc/' + d).st_uid != os.getuid(): continue
            cmd = open(f'/proc/{d}/cmdline', 'rb').read().replace(b'\0', b' ').decode(errors='replace').strip()
            st = open(f'/proc/{d}/stat').read().rsplit(')', 1)[1].split()
            P[int(d)] = (int(st[1]), int(st[2]), cmd)
        except (OSError, IndexError, ValueError):
            pass
    return P


def mains(P):
    """shard main processes (python started by a flock wrapper), grouped by command line"""
    out = {}
    for pid, (ppid, _, cmd) in P.items():
        if (cmd and f'{ROOT}/ml/' in cmd and 'python' in cmd.split()[0] and ppid in P
                and P[ppid][2].startswith('/usr/bin/flock')):
            out.setdefault(cmd, []).append(pid)
    return out


def orchestrators(P):
    return [p for p, (_, _, c) in P.items() if c and 'python' in c.split()[0] and f'{SR} tick' in c]


def arg(cmd, name):
    t = cmd.split(); return t[t.index(name) + 1]


def nlines(path):
    try: return sum(1 for _ in open(path))
    except FileNotFoundError: return 0


def main():
    cfg = ['py=' + PY, 'sets=[["data/rdemos",2,0],["data/rdemos_extra",2,5000],["data/rdemos_more",2,9000]]',
           'shard_episodes=3', 'collect_workers=3', 'seeds=[0,1]', 'epochs=6', 'ckpt_mins=0.02', 'neval=2',
           'eval_workers=4', 'cache_workers=3', 'cpu_slots=24', 'max_gpus=2',
           'train_flags=["--cam-aug","0","--cam-drop","0","--beam-drop","0","--state-mask","0,0,1,1,0","--bs","32"]']
    subprocess.run(['/usr/bin/python3', SR, 'install', '--root', ROOT] + [x for kv in cfg for x in ('--set', kv)],
                   check=True)
    t0 = time.time(); step = 0; dups = 0; victim = None; tk = 0; old = set()
    while time.time() - t0 < 50 * 60:
        P = snapshot(); M = mains(P)
        for c, pids in M.items():
            if len(pids) > 1:
                dups += 1; log(f'DUPLICATE: {len(pids)} copies of {c[-120:]}')
        if os.path.exists(f'{ROOT}/shard_state/DONE'): log('run finished (DONE)'); break
        if os.path.exists(f'{ROOT}/shard_state/FAILED'): log('run FAILED'); break
        parts = [p for p in glob.glob(f'{ROOT}/data/*/parts/part-*.jsonl') if '.partial' not in p]
        if step == 0:
            coll = [c for c in M if 'sim_rollout.py collect' in c]
            if len(parts) >= 2 and coll:
                old = set(orchestrators(P))
                for p in old: os.kill(p, signal.SIGKILL)
                log(f'KILL 1: orchestrator {sorted(old)} (SIGKILL) while {len(coll)} collect shards run')
                tk = time.time(); step = 1
        elif step == 1:
            new = [p for p in orchestrators(P) if p not in old]
            if new: log(f'  systemd started a new orchestrator {new} {time.time() - tk:.0f} s later'); step = 2
        elif step == 2:
            for c, pids in M.items():
                if 'sim_rollout.py collect' not in c: continue
                out, (k, N) = arg(c, '--out'), arg(c, '--shard').split('/')
                n = nlines(f'{out}/parts/part-{k}-of-{N}.partial.jsonl') - 1
                if n >= 1:
                    os.killpg(P[pids[0]][1], signal.SIGKILL)
                    log(f'KILL 2: process group of collect {os.path.basename(out)} shard {k}/{N} (SIGKILL), '
                        f'{n} episode(s) already recorded'); victim = f'{out}/parts/part-{k}-of-{N}.jsonl'; step = 3
                    break
        elif step == 3:
            if os.path.exists(victim): log('  that shard restarted and finished'); step = 4
        elif step == 4:
            for c, pids in M.items():
                if 'train_policy.py' not in c or '--build-cache-only' in c: continue
                run = arg(c, '--out'); n = nlines(f'{run}/log.jsonl')
                if n >= 2 and os.path.exists(f'{run}/ckpt.pt'):
                    time.sleep(0.7)                      # land between checkpoints, mid-epoch
                    os.kill(pids[0], signal.SIGKILL); victim = run
                    log(f'KILL 3: training {os.path.basename(run)} (SIGKILL, python only) during epoch {n}')
                    step = 5; break
        elif step == 5:
            if any('sim_rollout.py eval' in c for c in M):
                pids = [p for p, (_, _, c) in P.items() if f'{ROOT}/ml/' in c and p != os.getpid()]
                for p in pids:
                    try: os.kill(p, signal.SIGKILL)
                    except ProcessLookupError: pass
                log(f'KILL 4: everything at once, {len(pids)} processes (orchestrator, shards, pool workers)')
                tk = time.time(); step = 6
        elif step == 6:
            if orchestrators(P): log(f'  systemd brought the orchestrator back {time.time() - tk:.0f} s later'); step = 7
        time.sleep(0.5)
    log(f'kill stages completed: {min(step, 7)} of 7; duplicate shard processes seen: {dups}')
    os.system(f"grep -h 'resumed from' {ROOT}/logs/shard/train_*.log | sed 's/^/  /'")
    os.system(f'/usr/bin/python3 {SR} status --root {ROOT}')
    r = subprocess.run([PY, f'{ROOT}/ml/unicron/verify_run.py', ROOT])
    sys.exit(0 if (r.returncode == 0 and step >= 7 and dups == 0) else 1)


if __name__ == '__main__':
    main()
