#!/usr/bin/env python3
"""train_policy: train the goal-conditioned student from sim_rollout shards (BC or DAgger).

Differences from ml/train_student.py (which trains from the LeRobot video dataset):
  * inputs come from ml/sim_rollout.py shards, built by f1tenth_gym_ros/policy_io.py -- the
    exact preprocessing the car runs. No lossy mp4 round trip for the lidar raster.
  * the lidar raster is rebuilt on the GPU from the raw scan every batch, so beam dropout
    and range noise (the RPLidar C1 returns ~19% empty bins) can be randomised;
  * camera augmentation (gain, bias, colour, noise) and modality dropout (blank frame) so
    the policy cannot lean on the synthetic camera render, which will not look like the
    OAK-D Pro;
  * the ONNX file carries action_order metadata, which policy_bridge reads.
Same network (Student from train_student.py), same ONNX inputs, so it is a drop-in.

Restartable, for long runs on shared machines where processes get killed: a checkpoint with the
model, optimiser, LR schedule, RNG state and position inside the epoch is written every
--ckpt-mins minutes and after every epoch, and a run started again with the same arguments
resumes from it (SIGTERM writes one, then exits). Every output -- checkpoint, best.pt, last.pt,
student.onnx, log.jsonl and the data-cache parts -- is written to a temp file and renamed into
place, so a SIGKILL at any moment leaves the previous version or the complete new one.

    python3 ml/train_policy.py --data data/demos data/dagger1 --out runs/pol --epochs 25 --seed 0
"""
import argparse, collections, fcntl, glob, hashlib, json, math, os, signal, sys, threading, time, zlib
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
import numpy as np
import torch, torch.nn as nn, torch.nn.functional as F

HERE = os.path.dirname(os.path.abspath(__file__)); REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(REPO, 'f1tenth_gym_ros'))
from train_student import Student                    # noqa: E402
import policy_io as PIO                               # noqa: E402

BEAMS = 540
CACHE_PART_FILES = 100      # episode shards per data-cache part


def atomic_write(path, write):
    """write(f) into a temp file, fsync it, rename it over path: readers -- and a run restarted
    after a SIGKILL -- see the old file or the complete new one, never a torn one."""
    tmp = f'{path}.tmp{os.getpid()}'
    with open(tmp, 'wb') as f:
        write(f); f.flush(); os.fsync(f.fileno())
    os.replace(tmp, path)


def _exit_with_parent():
    """Pool workers: exit as soon as the parent process is gone (SIGKILLed, say) instead of lingering."""
    ppid = os.getppid()

    def watch():
        while os.getppid() == ppid: time.sleep(2)
        os._exit(1)
    threading.Thread(target=watch, daemon=True).start()


def load_shards(dirs, max_files=0, max_per_file=0, seed=0):
    files = sorted(f for d in dirs for f in glob.glob(os.path.join(d, '*.npz')))
    if max_files: files = files[:max_files]
    keys = ('front', 'scan', 'state', 'act', 'ids'); out = {k: [] for k in keys}; grp = []
    for gi, f in enumerate(files):
        z = np.load(f)
        if len(z['act']) == 0 or z['scan'].shape[1] != BEAMS: continue
        n = len(z['act']); sel = slice(None)
        # cap only on-policy (DAgger) shards: long student rollouts (timeouts, up to ~700 frames)
        # would otherwise dominate the data; expert demonstrations are kept whole
        if max_per_file and n > max_per_file and 'dagger' in os.path.basename(os.path.dirname(f)):
            sel = np.sort(np.random.default_rng(seed + gi).choice(n, max_per_file, replace=False))
        for k in keys: out[k].append(z[k][sel])
        # group = task (file name without the seed) so train/val never share a route
        key = os.path.basename(f).rsplit('_s', 1)[0].encode()        # crc32: stable across runs
        grp.append(np.full(len(out['act'][-1]), zlib.crc32(key), np.int64))
    return {k: np.concatenate(v) for k, v in out.items()}, np.concatenate(grp), len(files)


def _part_ok(cdir, name):
    """A cache part is complete when its .npz exists and its frame file has the expected size."""
    try:
        z = np.load(os.path.join(cdir, name + '.npz')); n = int(z['n'])
        return os.path.getsize(os.path.join(cdir, name + '.front.u8')) == n * int(np.prod(z['fshape']))
    except Exception:
        return False


def _build_part(job):
    """Episode shards files[...] (global index lo, lo+1, ...) -> <name>.front.u8 (camera frames,
    raw uint8 rows) + <name>.npz (scan, state, act, ids, grp, row count), each written atomically."""
    files, lo, cdir, name, max_per_file, seed, keep_stops = job
    small = {k: [] for k in ('scan', 'state', 'act', 'ids', 'grp')}; n = dropped = 0; fshape = None; geoms = set()
    ffn = os.path.join(cdir, name + '.front.u8'); tmp = f'{ffn}.tmp{os.getpid()}'
    with open(tmp, 'wb') as ff:
        for gi, f in enumerate(files, lo):
            z = np.load(f); act = z['act']
            if len(act) == 0 or z['scan'].shape[1] != BEAMS: continue
            # where the lidar and camera were (m ahead of the rear axle); the sim had both at 0 until 10/5
            geoms.add(tuple(round(float(x), 4) for x in z['geom']) if 'geom' in z.files else (0.0, 0.0))
            m = len(act); sel = np.arange(m)
            # cap only on-policy (DAgger) shards, as in load_shards()
            if max_per_file and m > max_per_file and 'dagger' in os.path.basename(os.path.dirname(f)):
                sel = np.sort(np.random.default_rng(seed + gi).choice(m, max_per_file, replace=False))
            if not keep_stops:                     # same rule as main(): speed exactly 0 = at goal
                k = act[sel, 0] > 0.0; dropped += int((~k).sum()); sel = sel[k]
            if len(sel) == 0: continue
            fr = np.ascontiguousarray(z['front'][sel], dtype=np.uint8)
            if fshape is None: fshape = fr.shape[1:]
            assert fr.shape[1:] == fshape, (f, fr.shape)
            ff.write(fr.tobytes())
            for k in ('scan', 'state', 'act', 'ids'): small[k].append(z[k][sel])
            key = os.path.basename(f).rsplit('_s', 1)[0].encode()        # crc32: stable across runs
            small['grp'].append(np.full(len(sel), zlib.crc32(key), np.int64)); n += len(sel)
        ff.flush(); os.fsync(ff.fileno())
    os.replace(tmp, ffn)
    arrs = {k: np.concatenate(v) for k, v in small.items() if v}
    arrs.update(n=np.int64(n), dropped=np.int64(dropped), fshape=np.array(fshape or (0,), np.int64),
                geoms=np.array(sorted(geoms), np.float64).reshape(-1, 2))
    atomic_write(os.path.join(cdir, name + '.npz'), lambda f: np.savez(f, **arrs))
    return name, n


def _build_cache_parts(files, cdir, max_per_file, seed, keep_stops, workers):
    t0 = time.time(); nparts = (len(files) + CACHE_PART_FILES - 1) // CACHE_PART_FILES
    names = [f'part{i:04d}' for i in range(nparts)]
    for f in glob.glob(os.path.join(cdir, '*.tmp*')): os.remove(f)          # left by a killed build
    jobs = [(files[i * CACHE_PART_FILES:(i + 1) * CACHE_PART_FILES], i * CACHE_PART_FILES, cdir, nm,
             max_per_file, seed, keep_stops) for i, nm in enumerate(names) if not _part_ok(cdir, nm)]
    print(f'building the data cache {cdir}: {len(files)} shards in {nparts} parts '
          f'({nparts - len(jobs)} already built), {workers} workers', flush=True)
    report = lambda nm, n: print(f'  {nm}: {n} steps, {time.time() - t0:.0f} s', flush=True)
    if workers > 1 and len(jobs) > 1:
        with ProcessPoolExecutor(min(workers, len(jobs)), initializer=_exit_with_parent) as ex:
            for nm, n in ex.map(_build_part, jobs): report(nm, n)
    else:
        for j in jobs: report(*_build_part(j))
    parts, total, dropped, fshape, geoms = [], 0, 0, None, set()
    for nm in names:
        z = np.load(os.path.join(cdir, nm + '.npz')); n = int(z['n'])
        parts.append({'name': nm, 'n': n}); total += n; dropped += int(z['dropped'])
        if n and fshape is None: fshape = [int(x) for x in z['fshape']]
        geoms |= {tuple(float(v) for v in g) for g in (z['geoms'] if 'geoms' in z.files else [(0.0, 0.0)])}
    if len(geoms) > 1:
        # one network, one sensor layout: the car-side bridge can only match one of them
        raise SystemExit(f'the data mixes sensor positions (lidar_x, cam_x) {sorted(geoms)}; train on one')
    meta = {'n': total, 'front_shape': fshape, 'files': len(files), 'dropped_stops': dropped,
            'keep_stops': keep_stops, 'parts': parts, 'geom': list(geoms.pop() if geoms else (0.0, 0.0))}
    atomic_write(os.path.join(cdir, 'meta.json'), lambda f: f.write(json.dumps(meta).encode()))
    print(f'cache built: {total} steps in {time.time() - t0:.0f} s', flush=True)


class FrontRows:
    """Camera-frame rows of the data cache, read with pread. The rows go through the page cache
    but are never mapped into this process, so its resident size stays small: on a shared machine
    a process holding a big memory map has a big RSS, which makes it the first pick of earlyoom
    or the kernel OOM killer."""
    def __init__(self, paths, counts, shape):
        self.shape = tuple(int(x) for x in shape); self.row = int(np.prod(self.shape))
        self.starts = np.concatenate([[0], np.cumsum(counts)]).astype(np.int64); self.n = int(self.starts[-1])
        self.fds = []
        for p in paths:
            fd = os.open(p, os.O_RDONLY)
            try:
                os.posix_fadvise(fd, 0, 0, os.POSIX_FADV_WILLNEED)   # start pulling it into the page cache
                os.posix_fadvise(fd, 0, 0, os.POSIX_FADV_RANDOM)     # batches read random rows: no readahead
            except (AttributeError, OSError):
                pass
            self.fds.append(fd)

    def __getitem__(self, ix):
        ix = np.asarray(ix, np.int64); out = np.empty((len(ix), self.row), np.uint8)
        part = np.searchsorted(self.starts, ix, side='right') - 1
        offs = (ix - self.starts[part]) * self.row
        for j in range(len(ix)):
            if os.preadv(self.fds[part[j]], [out[j]], int(offs[j])) != self.row:
                raise IOError(f'short read from the data cache (row {int(ix[j])})')
        return out.reshape((len(ix),) + self.shape)


def build_cache(dirs, max_files=0, max_per_file=0, seed=0, keep_stops=False, workers=1):
    """Load the shards in pieces instead of all at once.

    load_shards() holds every shard in RAM and then concatenates (about twice the data at the
    peak); on the 2400-episode set that ran the 62 GB workstation out of memory. Here the shards
    are streamed into an on-disk cache next to the data (data/_cache/<signature>/), in parts of
    CACHE_PART_FILES shards: per part, the camera frames (~95% of the bytes) go into a raw uint8
    file that training reads row by row (FrontRows), the small arrays (scan, state, act, ids,
    group) into an .npz. Rows, their order and the stop-frame filter are exactly what
    load_shards() + the keep_stops filter in main() produced, so the train/val split and the
    per-epoch batches are unchanged. Parts are built in parallel (workers) and written
    atomically; a build that was killed keeps its finished parts and the next one builds only the
    rest. One process builds at a time (file lock), others wait for it. Shared by every seed.
    """
    files = sorted(f for d in dirs for f in glob.glob(os.path.join(d, '*.npz')))
    if max_files: files = files[:max_files]
    sig = hashlib.sha1(json.dumps([[os.path.abspath(f), os.path.getsize(f), int(os.path.getmtime(f))]
                                   for f in files] + [max_per_file, seed, keep_stops, BEAMS, CACHE_PART_FILES,
                                                      'parts']).encode()).hexdigest()[:16]
    cdir = os.path.join(os.path.dirname(os.path.abspath(dirs[0])), '_cache', sig)
    mfile = os.path.join(cdir, 'meta.json')
    if not os.path.isfile(mfile):
        os.makedirs(cdir, exist_ok=True)
        with open(cdir + '.lock', 'w') as lk:
            fcntl.flock(lk, fcntl.LOCK_EX)
            if not os.path.isfile(mfile):
                _build_cache_parts(files, cdir, max_per_file, seed, keep_stops, workers)
    meta = json.load(open(mfile)); meta.update(dir=cdir, sig=sig)
    parts = [p for p in meta['parts'] if p['n'] > 0]
    small = [np.load(os.path.join(cdir, p['name'] + '.npz')) for p in parts]
    D = {k: np.concatenate([z[k] for z in small]) for k in ('scan', 'state', 'act', 'ids')}
    grp = np.concatenate([z['grp'] for z in small])
    front = FrontRows([os.path.join(cdir, p['name'] + '.front.u8') for p in parts], [p['n'] for p in parts],
                      meta['front_shape'])
    assert len(grp) == meta['n'] == front.n, (len(grp), meta['n'], front.n)
    return front, D, grp, meta


class BEVRaster(nn.Module):
    """Batched GPU version of policy_io.bev_image (identical geometry)."""
    def __init__(self, beams=BEAMS, size=PIO.BEV_HW[0], extent=PIO.BEV_EXTENT):
        super().__init__()
        ang = -math.pi + 2 * math.pi * torch.arange(beams) / beams
        self.register_buffer('c', torch.cos(ang)); self.register_buffer('s', torch.sin(ang))
        self.size, self.extent = size, extent

    def forward(self, r):                              # r [B, beams] metres, <=0.05 invalid
        B, S, E = r.shape[0], self.size, self.extent
        ok = (r > 0.05) & (r < E)
        x, y = r * self.c, r * self.s
        # .long() truncates toward zero, exactly like numpy .astype(int) in policy_io
        px = (S / 2 - x / E * S / 2).long(); py = (S / 2 - y / E * S / 2).long()
        ok &= (px >= 0) & (px < S) & (py >= 0) & (py < S)
        img = torch.zeros(B, S * S, device=r.device)
        idx = torch.where(ok, px * S + py, torch.zeros_like(px))
        img.scatter_reduce_(1, idx, ok.float(), reduce='amax')   # any hit -> 1
        img = img.view(B, 1, S, S)
        img[:, :, S // 2 - 1:S // 2 + 2, S // 2 - 1:S // 2 + 2] = 128 / 255
        return img.clamp(max=1.0)


def augment(front, scan, g, p):
    """front float [B,3,H,W] in 0..1, scan float [B,beams]."""
    B = front.shape[0]; dev = front.device
    if p['cam_aug'] > 0:
        gain = torch.empty(B, 1, 1, 1, device=dev).uniform_(1 - p['cam_aug'], 1 + p['cam_aug'])
        bias = torch.empty(B, 3, 1, 1, device=dev).uniform_(-0.15, 0.15) * p['cam_aug'] / 0.4
        front = (front * gain + bias + torch.randn_like(front) * 0.04 * p['cam_aug'] / 0.4).clamp(0, 1)
    if p['cam_drop'] > 0:
        keep = (torch.rand(B, 1, 1, 1, device=dev) >= p['cam_drop']).float()
        front = front * keep
    if p['beam_drop'] > 0:
        rate = torch.rand(B, 1, device=dev) * p['beam_drop']          # 0 .. beam_drop per sample
        scan = torch.where(torch.rand_like(scan) < rate, torch.zeros_like(scan), scan)
        scan = scan + torch.randn_like(scan) * 0.02
    return front, scan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', nargs='+', required=True); ap.add_argument('--out', required=True)
    ap.add_argument('--epochs', type=int, default=25); ap.add_argument('--bs', type=int, default=256)
    ap.add_argument('--lr', type=float, default=1e-3); ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--cam-aug', type=float, default=0.4); ap.add_argument('--cam-drop', type=float, default=0.3)
    ap.add_argument('--beam-drop', type=float, default=0.3); ap.add_argument('--val-frac', type=float, default=0.1)
    ap.add_argument('--init', default=None, help='warm start from a best.pt')
    ap.add_argument('--max-files', type=int, default=0)
    ap.add_argument('--max-per-file', type=int, default=150, help='frames kept per episode shard')
    ap.add_argument('--state-mask', default='0,0,0,0,0',
                    help='per-dim multiplier on (vx, wz, gx, gy, gz), baked into the exported model. '
                         'Default zeros: with its own speed as an input the clone copies it '
                         '(v=0 -> speed 0) and never leaves the start line (the "inertia" problem)')
    ap.add_argument('--inputs', default='camera,lidar', help='ablation: which image inputs the '
                    'network may use; a removed one is zeroed inside the exported model too')
    ap.add_argument('--keep-stops', action='store_true',
                    help='keep the expert\'s at-goal stop frames. Off by default: where the goal is '
                         'is not in the observation, so these frames teach "stop" at arbitrary places')
    ap.add_argument('--ckpt-mins', type=float, default=10.0,
                    help='minutes between mid-epoch checkpoints (one is also written after every epoch; '
                         '0 = only then). A run started again with the same arguments resumes from it')
    ap.add_argument('--cache-workers', type=int, default=1, help='processes that build the data cache')
    ap.add_argument('--build-cache-only', action='store_true',
                    help='build the data cache, write <out>/cache.json and exit (no GPU used)')
    a = ap.parse_args()
    torch.manual_seed(a.seed); np.random.seed(a.seed); os.makedirs(a.out, exist_ok=True)
    # frames are read row by row from an on-disk cache (see build_cache); the stop-frame filter
    # (expert speed is exactly 0 only once done) is applied while the cache is built
    FRONT, D, grp, meta = build_cache(a.data, a.max_files, a.max_per_file, keep_stops=a.keep_stops,
                                      workers=a.cache_workers)
    if a.build_cache_only:
        atomic_write(os.path.join(a.out, 'cache.json'), lambda f: f.write(json.dumps(
            {'dir': meta['dir'], 'steps': meta['n'], 'files': meta['files']}).encode()))
        print(f"data cache ready: {meta['n']} steps from {meta['files']} shards in {meta['dir']}", flush=True)
        return
    nf = meta['files']
    if not a.keep_stops:
        print(f"dropping {meta['dropped_stops']} at-goal stop frames", flush=True)
    ug = np.unique(grp); rng = np.random.default_rng(12345)       # split fixed across seeds
    val_g = set(rng.choice(ug, max(1, int(len(ug) * a.val_frac)), replace=False).tolist())
    vm = np.array([g in val_g for g in grp]); tr_idx, va_idx = np.where(~vm)[0], np.where(vm)[0]
    A = D['act'][tr_idx]; mu, sd = A.mean(0), A.std(0) + 1e-6
    atomic_write(os.path.join(a.out, 'action_norm.npy'), lambda f: np.save(f, np.stack([mu, sd])))
    dev = 'cuda' if torch.cuda.is_available() else 'cpu'      # cpu: smoke tests only
    print(f'{nf} shards, {len(tr_idx)} train / {len(va_idx)} val steps, action mean {mu.round(3)} sd {sd.round(3)}', flush=True)
    # camera frames (~95% of the bytes) stay in the on-disk cache and are read a batch at a
    # time by a background thread; everything else lives on the GPU
    T = {'scan': torch.from_numpy(D['scan']).to(dev),
         'state': torch.from_numpy(D['state']).to(dev), 'act': torch.from_numpy(D['act']).to(dev),
         'ids': torch.from_numpy(D['ids']).to(dev)}
    del D
    mu_t, sd_t = torch.tensor(mu, device=dev), torch.tensor(sd, device=dev)
    raster = BEVRaster().to(dev); model = Student(0).to(dev)
    mask = torch.tensor([float(x) for x in a.state_mask.split(',')], device=dev)
    model.register_buffer('state_mask', mask)
    use = set(a.inputs.split(','))
    model.register_buffer('cam_on', torch.tensor(1.0 if 'camera' in use else 0.0, device=dev))
    model.register_buffer('lidar_on', torch.tensor(1.0 if 'lidar' in use else 0.0, device=dev))
    # Masks act on the encoder OUTPUTS, not the images: a zeroed image through a BatchNorm
    # encoder trains on zero variance and then divides by ~sqrt(eps) at eval time (the first
    # lidar-only run's validation error jumped between 0.4 and 33 m/s because of this).
    def _fwd(front, bev, state, ids):
        f = model.front(front) * model.cam_on
        b = model.bev(bev) * model.lidar_on
        t = model.txt(ids); st = model.state(state * model.state_mask)
        return model.head(torch.cat([f, b, t, st], 1)), f
    model.forward = _fwd
    if a.init:   # masks are set by this run's flags; older checkpoints may not carry them
        ckpt = {k: v for k, v in torch.load(a.init, map_location=dev).items()
                if k not in ('state_mask', 'cam_on', 'lidar_on')}
        missing, unexpected = model.load_state_dict(ckpt, strict=False)
        assert set(missing) <= {'state_mask', 'cam_on', 'lidar_on'} and not unexpected, (missing, unexpected)
    opt = torch.optim.AdamW(model.parameters(), lr=a.lr, weight_decay=1e-4)
    steps = a.epochs * (len(tr_idx) // a.bs)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, a.lr, total_steps=max(steps, 20) + 1, pct_start=0.1)
    aug = {'cam_aug': a.cam_aug, 'cam_drop': a.cam_drop, 'beam_drop': a.beam_drop}
    tr_t = torch.from_numpy(tr_idx); va_t = torch.from_numpy(va_idx)

    io = ThreadPoolExecutor(2)

    def gather(ix):
        # sorted rows = mostly forward reads in the cache files; a batch's order does not matter
        # (mean loss, BatchNorm statistics), only which rows are in it
        ix = torch.sort(ix).values
        return ix, torch.from_numpy(FRONT[ix.numpy()])

    def batches(order, size, drop_last, start=0):
        """Same chunks as range(0, len(order)[-size+1], size), read two batches ahead; `start`
        skips the batches a resumed epoch already trained on."""
        stop = len(order) - size + 1 if drop_last else len(order)
        pending = collections.deque()
        for i in range(start, max(stop, 0), size):
            pending.append(io.submit(gather, order[i:i + size]))
            if len(pending) > 2: yield pending.popleft().result()
        while pending: yield pending.popleft().result()

    def batch(ix, fr, train):
        f = fr.to(dev).permute(0, 3, 1, 2).float() / 255.0
        g = ix.to(dev); s = T['scan'][g].float()
        if train: f, s = augment(f, s, None, aug)
        return f, raster(s), T['state'][g], T['ids'][g], T['act'][g]

    # ---- checkpoint / resume. The settings that define the run must match to resume.
    cfg = {k: v for k, v in vars(a).items() if k not in ('out', 'ckpt_mins', 'cache_workers', 'build_cache_only')}
    cfg.update(data=[os.path.abspath(d) for d in a.data], cache=meta['sig'])
    ck_path = os.path.join(a.out, 'ckpt.pt')
    best, records, start_ep, start_b, perm0, tl0, nb0 = 1e9, [], 0, 0, None, 0.0, 0
    if os.path.isfile(ck_path):
        ck = torch.load(ck_path, map_location='cpu', weights_only=False)
        if ck['cfg'] != cfg:
            sys.exit(f'{ck_path} is from a run with other settings; move it away to start this run fresh.\n'
                     f'  checkpoint: {ck["cfg"]}\n  this run:   {cfg}')
        model.load_state_dict(ck['model']); opt.load_state_dict(ck['opt']); sched.load_state_dict(ck['sched'])
        best, records, start_ep, start_b = ck['best'], ck['log'], ck['epoch'], ck['batch']
        perm0, tl0, nb0 = ck['perm'], ck['tl'], ck['nb']
        torch.set_rng_state(ck['rng']['torch']); np.random.set_state(ck['rng']['numpy'])
        try: torch.cuda.set_rng_state_all(ck['rng']['cuda'])
        except Exception as e: print(f'(CUDA RNG state not restored: {e})', flush=True)
        print(f'resumed from {ck_path}: epoch {start_ep}, batch {start_b}, best val score {best:.4f}', flush=True)

    def save_ckpt(ep, bi, perm, tl, nb):
        ck = {'cfg': cfg, 'epoch': ep, 'batch': bi, 'perm': perm, 'tl': tl, 'nb': nb, 'best': best,
              'log': list(records), 'model': model.state_dict(), 'opt': opt.state_dict(),
              'sched': sched.state_dict(), 'time': time.time(),
              'rng': {'torch': torch.get_rng_state(), 'numpy': np.random.get_state(),
                      'cuda': torch.cuda.get_rng_state_all()}}
        atomic_write(ck_path, lambda f: torch.save(ck, f))

    term = []      # SIGTERM (a polite stop): write a checkpoint at the next batch, then exit
    signal.signal(signal.SIGTERM, lambda signum, frame: term.append(signum))

    atomic_write(os.path.join(a.out, 'log.jsonl'),
                 lambda f: f.write(''.join(json.dumps(r) + '\n' for r in records).encode()))
    log = open(os.path.join(a.out, 'log.jsonl'), 'a'); last_ck = time.time()
    for ep in range(start_ep, a.epochs):
        model.train(); t0 = time.time()
        if ep == start_ep and perm0 is not None:
            perm, b0, tl, nb = perm0, start_b, tl0, nb0
        else:
            perm, b0, tl, nb = tr_t[torch.randperm(len(tr_t))], 0, 0.0, 0
        bi = b0
        for ix, fr in batches(perm, a.bs, True, start=b0 * a.bs):
            f, b, st, ids, act = batch(ix, fr, True)
            pred, _ = model(f, b, st, ids)
            loss = F.smooth_l1_loss(pred, (act - mu_t) / sd_t)
            opt.zero_grad(set_to_none=True); loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), 1.0); opt.step(); sched.step()
            tl += loss.item(); nb += 1; bi += 1
            if term or (a.ckpt_mins > 0 and time.time() - last_ck > a.ckpt_mins * 60):
                save_ckpt(ep, bi, perm, tl, nb); last_ck = time.time()
                if term:
                    print(f'signal {term[0]}: checkpoint written at epoch {ep} batch {bi}, exiting', flush=True)
                    sys.exit(128 + term[0])
        model.eval(); err = torch.zeros(2, device=dev)
        with torch.no_grad():
            for ix, fr in batches(va_t, 1024, False):
                f, b, st, ids, act = batch(ix, fr, False)
                err += ((model(f, b, st, ids)[0] * sd_t + mu_t) - act).abs().sum(0)
        mae = (err / max(len(va_t), 1)).cpu().numpy()
        # dataset order is (speed, steer): index 0 is speed, index 1 is steer
        rec = {'epoch': ep, 'train_loss': round(tl / max(nb, 1), 5), 'val_mae_speed_mps': round(float(mae[0]), 4),
               'val_mae_steer_rad': round(float(mae[1]), 4), 'secs': round(time.time() - t0, 1)}
        if b0: rec['resumed_at_batch'] = b0
        records.append(rec); print(json.dumps(rec), flush=True); log.write(json.dumps(rec) + '\n'); log.flush()
        score = mae[1] / 0.4 + mae[0] / 1.5
        if score < best:
            best = score; atomic_write(os.path.join(a.out, 'best.pt'), lambda f: torch.save(model.state_dict(), f))
        save_ckpt(ep + 1, 0, None, 0.0, 0); last_ck = time.time()
        if term:
            print(f'signal {term[0]}: checkpoint written after epoch {ep}, exiting', flush=True)
            sys.exit(128 + term[0])
    atomic_write(os.path.join(a.out, 'last.pt'), lambda f: torch.save(model.state_dict(), f))
    g = meta.get('geom', [0.0, 0.0])          # caches built before 10/5 carry none: the sim's sensors at the rear axle
    export(model, os.path.join(a.out, 'best.pt'), mu, sd, os.path.join(a.out, 'student.onnx'),
           dict(vars(a), sensor_geom={'lidar_x': float(g[0]), 'cam_x': float(g[1])}))
    print(f'done: best val score {best:.4f} -> {a.out}/student.onnx', flush=True)


def export(model, weights, mu, sd, path, cfg):
    import onnx
    model = model.cpu(); model.load_state_dict(torch.load(weights, map_location='cpu')); model.eval()
    mu_t, sd_t = torch.tensor(mu), torch.tensor(sd)

    class MeanBag(nn.Module):
        """EmbeddingBag(mode='mean', padding_idx=0) without the op the legacy ONNX exporter
        rejects: gather, mask the padding, average (all-padding -> zeros, as EmbeddingBag)."""
        def __init__(s, bag): super().__init__(); s.w = bag.weight
        def forward(s, ids):
            m = (ids != 0).float().unsqueeze(-1)
            return (s.w[ids] * m).sum(1) / m.sum(1).clamp(min=1.0)
    ids_test = torch.tensor([PIO.text_ids('turn left, then go straight to the end and stop')])
    ref = model.txt(ids_test)
    model.txt = MeanBag(model.txt.emb)
    assert torch.allclose(ref, model.txt(ids_test), atol=1e-6), 'MeanBag != EmbeddingBag'

    class Wrap(nn.Module):
        def __init__(s, m): super().__init__(); s.m = m
        def forward(s, front, bev, state, ids): return s.m(front, bev, state, ids)[0] * sd_t + mu_t   # s.m.forward applies state_mask
    dummy = (torch.zeros(1, 3, *PIO.FRONT_HW), torch.zeros(1, 1, *PIO.BEV_HW), torch.zeros(1, 5),
             torch.zeros(1, PIO.MAX_TOK, dtype=torch.long))
    # written to a temp file and renamed: student.onnx existing means the run finished
    tmp = f'{path}.tmp{os.getpid()}'
    torch.onnx.export(Wrap(model), dummy, tmp, input_names=['front', 'bev', 'state', 'ids'],
                      output_names=['action'], opset_version=17, dynamo=False,
                      dynamic_axes={k: {0: 'batch'} for k in ('front', 'bev', 'state', 'ids', 'action')})
    m = onnx.load(tmp)
    for k, v in (('action_order', ','.join(PIO.ACTION_ORDER)), ('trainer', 'ml/train_policy.py'),
                 ('config', json.dumps({k: v for k, v in cfg.items() if k != 'data'}))):
        e = m.metadata_props.add(); e.key, e.value = k, v
    onnx.save(m, tmp)
    fd = os.open(tmp, os.O_RDONLY); os.fsync(fd); os.close(fd)
    os.replace(tmp, path)


if __name__ == '__main__':
    main()
