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

    python3 ml/train_policy.py --data data/demos data/dagger1 --out runs/pol --epochs 25 --seed 0
"""
import argparse, collections, glob, hashlib, json, math, mmap, os, shutil, sys, time, zlib
from concurrent.futures import ThreadPoolExecutor
import numpy as np
import torch, torch.nn as nn, torch.nn.functional as F

HERE = os.path.dirname(os.path.abspath(__file__)); REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(REPO, 'f1tenth_gym_ros'))
from train_student import Student                    # noqa: E402
import policy_io as PIO                               # noqa: E402

BEAMS = 540


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


def build_cache(dirs, max_files=0, max_per_file=0, seed=0, keep_stops=False):
    """Load the shards in pieces instead of all at once.

    load_shards() holds every shard in RAM and then concatenates (about twice the data at the
    peak). The 2400-episode extended set is ~55 GB once loaded, so on the 62 GB workstation it
    ran out of memory. Here the shards are streamed once, one file at a time, into an on-disk
    cache next to the data (data/_cache/<signature>/): the camera frames (~95% of the bytes)
    go into one raw uint8 file that training memory-maps, and the small arrays (scan, state,
    act, ids, group) are saved as .npy. Rows, their order and the stop-frame filter are exactly
    what load_shards() + the keep_stops filter in main() produced, so the train/val split and
    the per-epoch batches are unchanged. The cache is shared by every seed on the same data;
    a half-written one (build interrupted) is ignored and rebuilt.
    """
    files = sorted(f for d in dirs for f in glob.glob(os.path.join(d, '*.npz')))
    if max_files: files = files[:max_files]
    sig = hashlib.sha1(json.dumps([[os.path.abspath(f), os.path.getsize(f), int(os.path.getmtime(f))]
                                   for f in files] + [max_per_file, seed, keep_stops, BEAMS]).encode()).hexdigest()[:16]
    cdir = os.path.join(os.path.dirname(os.path.abspath(dirs[0])), '_cache', sig)
    if not os.path.isfile(os.path.join(cdir, 'meta.json')):
        tmp = cdir + '.tmp'; shutil.rmtree(tmp, ignore_errors=True); os.makedirs(tmp)
        small = {k: [] for k in ('scan', 'state', 'act', 'ids')}; grp = []
        n = dropped = 0; fshape = None; t0 = time.time()
        print(f'building the data cache {cdir} from {len(files)} shards (once per data set)', flush=True)
        with open(os.path.join(tmp, 'front.u8'), 'wb') as ff:
            for gi, f in enumerate(files):
                z = np.load(f); act = z['act']
                if len(act) == 0 or z['scan'].shape[1] != BEAMS: continue
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
                for k in small: small[k].append(z[k][sel])
                key = os.path.basename(f).rsplit('_s', 1)[0].encode()        # crc32: stable across runs
                grp.append(np.full(len(sel), zlib.crc32(key), np.int64)); n += len(sel)
                if gi % 200 == 0: print(f'  {gi}/{len(files)} shards, {n} steps, {time.time() - t0:.0f} s', flush=True)
        for k, v in small.items(): np.save(os.path.join(tmp, k + '.npy'), np.concatenate(v))
        np.save(os.path.join(tmp, 'grp.npy'), np.concatenate(grp))
        json.dump({'n': n, 'front_shape': list(fshape), 'files': len(files), 'dropped_stops': dropped,
                   'keep_stops': keep_stops}, open(os.path.join(tmp, 'meta.json'), 'w'))
        shutil.rmtree(cdir, ignore_errors=True); os.rename(tmp, cdir)
        print(f'cache built: {n} steps in {time.time() - t0:.0f} s', flush=True)
    meta = json.load(open(os.path.join(cdir, 'meta.json')))
    front = np.memmap(os.path.join(cdir, 'front.u8'), np.uint8, 'r', shape=(meta['n'], *meta['front_shape']))
    try: front._mmap.madvise(mmap.MADV_RANDOM)       # batches read random rows: no readahead
    except Exception: pass
    D = {k: np.load(os.path.join(cdir, k + '.npy')) for k in ('scan', 'state', 'act', 'ids')}
    return front, D, np.load(os.path.join(cdir, 'grp.npy')), meta


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
    a = ap.parse_args()
    torch.manual_seed(a.seed); np.random.seed(a.seed); os.makedirs(a.out, exist_ok=True)
    # frames stream from a memory-mapped on-disk cache (see build_cache); the stop-frame filter
    # (expert speed is exactly 0 only once done) is applied while the cache is built
    FRONT, D, grp, meta = build_cache(a.data, a.max_files, a.max_per_file, keep_stops=a.keep_stops)
    nf = meta['files']
    if not a.keep_stops:
        print(f"dropping {meta['dropped_stops']} at-goal stop frames", flush=True)
    ug = np.unique(grp); rng = np.random.default_rng(12345)       # split fixed across seeds
    val_g = set(rng.choice(ug, max(1, int(len(ug) * a.val_frac)), replace=False).tolist())
    vm = np.array([g in val_g for g in grp]); tr_idx, va_idx = np.where(~vm)[0], np.where(vm)[0]
    A = D['act'][tr_idx]; mu, sd = A.mean(0), A.std(0) + 1e-6
    np.save(os.path.join(a.out, 'action_norm.npy'), np.stack([mu, sd]))
    dev = 'cuda'
    print(f'{nf} shards, {len(tr_idx)} train / {len(va_idx)} val steps, action mean {mu.round(3)} sd {sd.round(3)}', flush=True)
    # camera frames (~95% of the bytes) stay in the memory-mapped cache and are read a batch at a
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
        # sorted rows = mostly forward reads in the cache file; a batch's order does not matter
        # (mean loss, BatchNorm statistics), only which rows are in it
        ix = torch.sort(ix).values
        return ix, torch.from_numpy(FRONT[ix.numpy()])

    def batches(order, size, drop_last):
        """Same chunks as range(0, len(order)[-size+1], size), read two batches ahead."""
        stop = len(order) - size + 1 if drop_last else len(order)
        pending = collections.deque()
        for i in range(0, max(stop, 0), size):
            pending.append(io.submit(gather, order[i:i + size]))
            if len(pending) > 2: yield pending.popleft().result()
        while pending: yield pending.popleft().result()

    def batch(ix, fr, train):
        f = fr.to(dev).permute(0, 3, 1, 2).float() / 255.0
        g = ix.to(dev); s = T['scan'][g].float()
        if train: f, s = augment(f, s, None, aug)
        return f, raster(s), T['state'][g], T['ids'][g], T['act'][g]

    best = 1e9; log = open(os.path.join(a.out, 'log.jsonl'), 'w')
    for ep in range(a.epochs):
        model.train(); t0 = time.time(); tl = 0.0; nb = 0
        perm = tr_t[torch.randperm(len(tr_t))]
        for ix, fr in batches(perm, a.bs, True):
            f, b, st, ids, act = batch(ix, fr, True)
            pred, _ = model(f, b, st, ids)
            loss = F.smooth_l1_loss(pred, (act - mu_t) / sd_t)
            opt.zero_grad(set_to_none=True); loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), 1.0); opt.step(); sched.step()
            tl += loss.item(); nb += 1
        model.eval(); err = torch.zeros(2, device=dev)
        with torch.no_grad():
            for ix, fr in batches(va_t, 1024, False):
                f, b, st, ids, act = batch(ix, fr, False)
                err += ((model(f, b, st, ids)[0] * sd_t + mu_t) - act).abs().sum(0)
        mae = (err / max(len(va_t), 1)).cpu().numpy()
        # dataset order is (speed, steer): index 0 is speed, index 1 is steer
        rec = {'epoch': ep, 'train_loss': round(tl / max(nb, 1), 5), 'val_mae_speed_mps': round(float(mae[0]), 4),
               'val_mae_steer_rad': round(float(mae[1]), 4), 'secs': round(time.time() - t0, 1)}
        print(json.dumps(rec), flush=True); log.write(json.dumps(rec) + '\n'); log.flush()
        score = mae[1] / 0.4 + mae[0] / 1.5
        if score < best:
            best = score; torch.save(model.state_dict(), os.path.join(a.out, 'best.pt'))
    torch.save(model.state_dict(), os.path.join(a.out, 'last.pt'))
    export(model, os.path.join(a.out, 'best.pt'), mu, sd, os.path.join(a.out, 'student.onnx'), vars(a))
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
    torch.onnx.export(Wrap(model), dummy, path, input_names=['front', 'bev', 'state', 'ids'],
                      output_names=['action'], opset_version=17, dynamo=False,
                      dynamic_axes={k: {0: 'batch'} for k in ('front', 'bev', 'state', 'ids', 'action')})
    m = onnx.load(path)
    for k, v in (('action_order', ','.join(PIO.ACTION_ORDER)), ('trainer', 'ml/train_policy.py'),
                 ('config', json.dumps({k: v for k, v in cfg.items() if k != 'data'}))):
        e = m.metadata_props.add(); e.key, e.value = k, v
    onnx.save(m, path)


if __name__ == '__main__':
    main()
