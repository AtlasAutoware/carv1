#!/usr/bin/env python3
"""return_start: back the car up to the calibration start spot, steering by the lidar.

After an auto_calibrate run the car stands a few metres down the hallway, roughly along it.
This reverses it at the slowest erpm-mode speed (about 0.8 m/s), steering to hold the
hallway axis and the start's lateral position (from the side walls), and stops when the wall
behind is as far away as it was at the start. The start is recorded once, at the start spot,
with --save (~/calib/start_ref.json).

Everything is measured in the car's own frame (the frame the lidar, the steering command and
auto_calibrate share), so it does not depend on which way round the hardware is mounted.

Safety: stops on anything within 0.6 m behind the rear bumper in the car's path, a wall within
0.27 m of either side, a stale lidar or VESC, a heading more than 10 deg off the hallway, a
lateral error over 0.6 m, losing the wall behind, a distance budget (start distance + 0.8 m)
and a time budget. Before moving it waits (up to --wait-clear s) for the strip behind the car
to be clear. Teleop (pilot page, gamepad) outranks /drive in the mux.

    python3 return_start.py --save        # at the start spot: record the reference
    python3 return_start.py --dry         # the car vs the reference; nothing moves
    python3 return_start.py --go          # back up to the start
"""
import argparse, json, math, os, sys, time
from types import SimpleNamespace
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import auto_calibrate as AC

REF = os.path.expanduser('~/calib/start_ref.json')
REAR, FRONT, HALF_W = -0.12, AC.FRONT_OVERHANG, 0.15    # body outline from base_link (rear axle), m
ROLL = 0.15                    # roll-out after the stop command from ~0.8 m/s (10/9 logs: ~0.1 m + dead time)
K_Y, K_PSI = 0.5, 1.5          # lateral and heading gains, 1/s
PSI_REF_MAX = math.radians(6)  # largest heading used to correct the lateral position
TAPER = 1.5                    # m before the stop over which the lateral correction fades out
# The car travels ~2.3 deg toward -y of the lidar's zero (10/9: both returns, 2.3-2.5 deg, and
# the forward runs' arc chords, 1.8-2.5 deg), so the wall heading reads 2.3 deg more than the
# direction the car really moves in. Steering uses the corrected heading.
LIDAR_YAW = math.radians(2.3)
DELTA_MAX = 0.20               # steering limit, rad (right lock is ~0.23)


def pts(scan, mask=None, rmax=14.0):
    """Scan as base_link x, y points (like auto_calibrate.points, but out to rmax: the wall
    behind can be 10 m away at the end of a fast run)."""
    a, r = AC.scan_arrays(scan)
    ok = np.isfinite(r) & (r > 0.12) & (r < rmax)
    if mask is not None and len(mask) == len(r): ok &= ~mask
    return np.c_[r[ok] * np.cos(a[ok]) + AC.LASER_X, r[ok] * np.sin(a[ok])]


def densest(v, w=0.05, band=0.08, nmin=6):
    """Median of the densest cluster of values: a wall is many returns at one distance, a
    door recess or a bin is fewer."""
    v = np.asarray(v, float)
    if len(v) < nmin: return float('nan')
    lo, hi = float(v.min()), float(v.max())
    if hi - lo < w: return float(np.median(v))
    n, e = np.histogram(v, np.arange(lo, hi + w, w))
    c = e[n.argmax()] + w / 2
    sel = np.abs(v - c) < band
    return float(np.median(v[sel])) if sel.sum() >= nmin else float('nan')


def estimate(scan, mask=None):
    """Car pose in the hallway from one scan: heading psi (rad, + = turned toward +y of the
    car frame), distances from base_link to the +y wall (dL), the -y wall (dR) and the wall
    behind (dB), measured across and along the hallway axis; y = lateral position
    (+ = toward the dL wall) from the middle."""
    P = pts(scan, mask)
    if len(P) < 40: return None
    psi, nw = AC.wall_heading(P[np.hypot(*P.T) < 5.0])
    c, s = math.cos(psi), math.sin(psi)
    X = c * P[:, 0] - s * P[:, 1]; Y = s * P[:, 0] + c * P[:, 1]       # hallway-aligned frame
    side = (X > -1.5) & (X < 2.5)
    dL = densest(Y[side & (Y > 0.35) & (Y < 3.0)])
    dR = densest(-Y[side & (Y < -0.35) & (Y > -3.0)])
    dB = densest(-X[(X < -0.3) & (np.abs(Y) < 1.0)])
    y = (dR - dL) / 2 if np.isfinite(dL) and np.isfinite(dR) else float('nan')
    return dict(psi=float(psi), nwall=int(nw), dL=dL, dR=dR, dB=dB, y=y)


def median_scan(scans):
    """One scan-like object holding the per-bin median of several scans."""
    R = np.array([np.asarray(s.ranges, np.float64) for s in scans])
    R = np.where(np.isfinite(R) & (R > 0), R, np.nan)
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        med = np.nanmedian(R, 0)
    s = scans[-1]
    return SimpleNamespace(ranges=np.where(np.isfinite(med), med, np.inf).tolist(),
                           angle_min=s.angle_min, angle_increment=s.angle_increment)


def strip_blocked(scan, mask, length, half=HALF_W + 0.15):
    """Nearest return in the strip the car sweeps when it backs up `length` metres
    (car frame, straight back from the rear bumper). Returns its distance or inf."""
    P = pts(scan, mask)
    sel = (P[:, 0] < REAR) & (P[:, 0] > REAR - length) & (np.abs(P[:, 1]) < half)
    return float(REAR - P[sel, 0].max()) if sel.any() else float('inf')


def side_min(scan, mask):
    P = pts(scan, mask)
    sel = (P[:, 0] > REAR - 0.05) & (P[:, 0] < FRONT + 0.05)
    return float(np.abs(P[sel, 1]).min()) if sel.any() else float('inf')


def rest_scans(car, secs, drive=False):
    """Scans collected while the car stands still (zero speed published when drive is set)."""
    t0 = car.now(); scans = []
    while car.now() - t0 < secs:
        if drive: car.hold(0.05, 0.0, 0.0)
        else: car.spin_for(0.05)
        if car.scan is not None and (not scans or car.scan is not scans[-1]): scans.append(car.scan)
    return scans


def self_mask(scans):
    R = np.array([np.asarray(s.ranges, np.float64) for s in scans])
    return np.where(np.isfinite(R) & (R > 0), R, np.inf).min(0) < 0.30


def fmt(e):
    return ('heading %+.1f deg, wall behind %.2f m, side walls %.2f / %.2f m, lateral %+.2f m'
            % (math.degrees(e['psi']), e['dB'], e['dL'], e['dR'], e['y']))


def realign(car, A, mask, psi0, v_cmd, v, cfg, log):
    """Back up on full lock until the car points along the hallway again (a fast run can end
    20-30 deg off it). Backing up, +delta turns the heading down; at 0.8 m/s and 0.20 rad the
    heading changes ~0.5 rad/s on a 1.6 m radius, and a 30 deg correction moves the rear axle
    ~0.7 m back and ~0.2 m sideways. Returns True when the heading is within ~5 deg."""
    delta = math.copysign(DELTA_MAX, psi0)
    rate = -v * math.tan(DELTA_MAX) / AC.L                  # |heading rate|, rad/s
    log('pointing %+.0f deg off the hallway: backing up on full lock to straighten out' % math.degrees(psi0))
    t0 = car.now(); since = None; said = False              # 1.2 m behind the car, clear for 1 s
    while True:
        car.hold(0.1, 0.0, delta)
        b = strip_blocked(car.scan, mask, 1.2)
        if not np.isfinite(b):
            since = since or car.now()
            if car.now() - since >= 1.0: break
        else:
            since = None
            if not said: log('waiting: something %.2f m behind the car' % b); said = True
        if car.now() - t0 > A.wait_clear: log('the way back never cleared'); return False
    t_go = car.now(); last = None; psi, t_psi = psi0, t_go; reason = None
    while True:
        now = car.now()
        if now - t_go > 4.0: reason = 'time budget used up'; break
        if car.scan is None or now - car.scan_t > 0.4: reason = 'lidar stale'; break
        if not car.core or now - car.core[-1][0] > 0.3: reason = 'VESC telemetry stale'; break
        core = car.arr('core', t_go)
        if len(core) > 1 and AC.trapz(np.abs(core[:, 1]), core[:, 0]) / cfg['erpm_gain'] > 1.5:
            reason = 'distance budget used up'; break
        if car.scan is not last:
            last = car.scan
            e = estimate(last, mask)
            if e is not None and e['nwall'] >= 40: psi, t_psi = e['psi'] - LIDAR_YAW, now
            b = strip_blocked(last, mask, 0.6)
            if b < 0.6: reason = 'something %.2f m behind the rear bumper' % b; break
            sm = side_min(last, mask)
            if sm < HALF_W + 0.12: reason = 'side clearance %.2f m' % sm; break
        if now - t_psi > 0.6: reason = 'lost the walls'; break
        # heading predicted from the last scan (+0.08 s scan age) and the roll-out still to come
        moving = now - t_go > 0.15
        pred = abs(psi) - (rate * (now - t_psi + 0.08) if moving else 0.0) - ROLL / (AC.L / math.tan(DELTA_MAX))
        if pred <= math.radians(1.5): break
        car.hold(0.02, v_cmd, delta)
    car.hold(0.02, 0.0, delta)
    car.rest(0.0)
    if reason: log('straightening stopped: ' + reason); return False
    return True


def back_up(car, A, ref, cfg, log):
    """The return itself. Returns (exit code, summary dict)."""
    v_cmd = -abs(A.speed)
    v = -AC.wheel_speed(abs(A.speed), cfg)              # wheel speed, m/s, negative = backward
    # the walls, at rest; someone standing behind the car can hide the wall behind, so wait for it
    # (10/9: the first try after a run saw no wall behind at 10.6 m; a minute later every scan did)
    t0w = car.now(); said = False
    while True:
        scans = rest_scans(car, 1.2, drive=True)
        if len(scans) < 5: return 3, dict(reason='lidar too slow')
        mask = self_mask(scans)
        e0 = estimate(median_scan(scans), mask)
        if e0 is not None and np.isfinite(e0['dB']) and np.isfinite(e0['y']): break
        if not said:
            log('waiting: cannot see the side walls and the wall behind (%s)' % (fmt(e0) if e0 else 'no scan points'))
            said = True
        if car.now() - t0w > A.wait_clear:
            log('the walls never showed up'); return 3, dict(reason='walls not found')
    log('now:   ' + fmt(e0)); log('start: ' + fmt(ref))
    psi_c0 = e0['psi'] - LIDAR_YAW
    if math.radians(10) < abs(psi_c0) <= math.radians(40) and e0['dB'] - ref['dB'] > 1.0:
        if side_min(median_scan(scans), mask) < HALF_W + 0.20:
            log('a wall is within 0.2 m of the car\'s side: move it by hand'); return 3, dict(reason='side')
        if not realign(car, A, mask, psi_c0, -abs(A.speed), -AC.wheel_speed(abs(A.speed), cfg), cfg, log):
            return 2, dict(reason='straightening failed')
        scans = rest_scans(car, 1.2, drive=True)
        e1 = estimate(median_scan(scans), mask)
        if e1 is None or not np.isfinite(e1['dB']) or not np.isfinite(e1['y']):
            log('walls lost after straightening out'); return 2, dict(reason='walls not found')
        e0 = e1
        log('now:   ' + fmt(e0))
    travel = e0['dB'] - ref['dB']
    if travel < 0.35:
        log('already at the start (%.2f m from it)' % travel); return 0, dict(travel=0.0, start=e0, end=e0)
    if travel > A.max_travel: log('%.1f m from the start: more than --max-travel' % travel); return 3, dict(reason='too far')
    if abs(e0['psi'] - LIDAR_YAW) > math.radians(12):
        log('pointing %+.0f deg off the hallway: line it up by hand' % math.degrees(e0['psi'] - LIDAR_YAW))
        return 3, dict(reason='heading')
    # lateral target: the middle of the hallway (default), or the start's own lateral position.
    # Far down the hallway the walls are not exactly in line with the start's (10/9: ~0.2 m), so
    # the start's offset only means something near the start; the middle is safe everywhere.
    yt = ref['y'] if A.lateral == 'start' else 0.0
    if side_min(median_scan(scans), mask) < HALF_W + 0.20:
        log('a wall is within 0.2 m of the car\'s side: move it by hand'); return 3, dict(reason='side')
    y_lim = max(abs(e0['y'] - yt) + 0.25, 0.6)          # divergence guard while backing up
    # the strip behind must stay clear for 1 s (stop point plus 0.5 m)
    t0 = car.now(); since = None; said = False
    while True:
        car.hold(0.1, 0.0, 0.0)
        b = strip_blocked(car.scan, mask, travel + 0.5)
        if not np.isfinite(b):
            since = since or car.now()
            if car.now() - since >= 1.0: break
        else:
            since = None
            if not said: log('waiting: something %.2f m behind the car' % b); said = True
        if car.now() - t0 > A.wait_clear:
            log('the way back never cleared'); return 3, dict(reason='blocked')
    log('backing up %.2f m at %.2f m/s' % (travel, -v))
    t_go = car.now(); last = None; delta = 0.0
    psi, y, dB, t_dB = e0['psi'], e0['y'], e0['dB'], t_go
    reason = None; tr = []
    while True:
        now = car.now()
        if now - t_go > A.max_time: reason = 'time budget used up'; break
        if car.scan is None or now - car.scan_t > 0.4: reason = 'lidar stale'; break
        if not car.core or now - car.core[-1][0] > 0.3: reason = 'VESC telemetry stale'; break
        core = car.arr('core', t_go)
        dist = float(AC.trapz(np.abs(core[:, 1]), core[:, 0]) / cfg['erpm_gain']) if len(core) > 1 else 0.0
        if dist > travel + 0.8: reason = 'distance budget used up (%.2f m)' % dist; break
        if car.scan is not last:
            last = car.scan
            e = estimate(last, mask)
            if e is not None:
                if np.isfinite(e['dB']) and abs(e['dB'] - (dB + v * (now - t_dB))) < 0.5: dB, t_dB = e['dB'], now
                if e['nwall'] >= 40 and abs(e['psi'] - psi) < 0.15: psi = e['psi']
                if np.isfinite(e['y']) and abs(e['y'] - y) < 0.3: y = e['y']
            if now - t_dB > 0.6: reason = 'lost the wall behind'; break
            psi_c = psi - LIDAR_YAW               # the direction the car actually moves in
            if abs(psi_c) > math.radians(10): reason = 'heading %+.1f deg off the hallway' % math.degrees(psi_c); break
            if abs(y - yt) > y_lim: reason = 'lateral error %+.2f m and growing' % (y - yt); break
            b = strip_blocked(last, mask, 0.6)
            if b < 0.6: reason = 'something %.2f m behind the rear bumper' % b; break
            sm = side_min(last, mask)
            if sm < HALF_W + 0.12: reason = 'side clearance %.2f m' % sm; break
            # lateral -> heading reference -> steering, all for backing up (v < 0). The lateral
            # correction fades out over the last TAPER m so the car arrives pointing along the
            # hallway (10/9: far down the hallway the walls sit ~0.2 m off the start's walls, and
            # chasing that held the car at +5 deg all the way back)
            psi_ref = math.asin(float(np.clip(K_Y * (yt - y) / v, -math.sin(PSI_REF_MAX), math.sin(PSI_REF_MAX))))
            psi_ref *= float(np.clip((dB - ref['dB'] - 0.3) / TAPER, 0.0, 1.0))
            delta = float(np.clip(math.atan(-K_PSI * (psi_c - psi_ref) * AC.L / v), -DELTA_MAX, DELTA_MAX))
            tr.append((now - t_go, dB, psi, y, delta, dist))
        # the wall-behind distance, predicted forward from the last scan (+0.08 s scan age)
        if dB + v * (now - t_dB + 0.08) - ref['dB'] <= ROLL: break
        car.hold(0.02, v_cmd, delta)
    car.hold(0.02, 0.0, 0.0)
    car.rest(0.0)
    if reason: log('stopped: ' + reason)
    e1 = estimate(median_scan(rest_scans(car, 1.0, drive=True)), mask)
    core = car.arr('core', t_go)
    dist = float(AC.trapz(np.abs(core[:, 1]), core[:, 0]) / cfg['erpm_gain']) if len(core) > 1 else 0.0
    if e1: log('end:   ' + fmt(e1))
    log('backed up %.2f m (eRPM) in %.1f s; %s' % (dist, car.now() - t_go,
        'off the start by %+.2f m along, %+.2f m across from the lateral target, travel direction %+.1f deg'
        % (e1['dB'] - ref['dB'], e1['y'] - yt, math.degrees(e1['psi'] - LIDAR_YAW)) if e1 else 'no final fix'))
    ok = (reason is None and e1 is not None and abs(e1['dB'] - ref['dB']) < 0.5
          and abs(e1['psi'] - LIDAR_YAW) < math.radians(8))
    return (0 if ok else 2), dict(reason=reason, travel=travel, dist=dist, start=e0, end=e1, trace=tr)


def ref_from_npz(path):
    z = np.load(path)
    R = z['ranges'].astype(np.float64); a = z['angles']
    scans = [SimpleNamespace(ranges=r.tolist(), angle_min=float(a[0]), angle_increment=float(a[1] - a[0])) for r in R]
    return estimate(median_scan(scans), self_mask(scans))


def main():
    ap = argparse.ArgumentParser(description='Back the car up to the calibration start spot (see the docstring).')
    ap.add_argument('--go', action='store_true', help='drive back to the start')
    ap.add_argument('--dry', action='store_true', help='show where the car is against the start; nothing moves')
    ap.add_argument('--save', action='store_true', help='record the car\'s current spot as the start')
    ap.add_argument('--ref-from-npz', help='record the start from a saved scan file (ranges, angles) instead')
    ap.add_argument('--speed', type=float, default=0.3, help='/drive speed magnitude (erpm mode: 0.3 -> ~0.8 m/s)')
    ap.add_argument('--wait-clear', type=float, default=30.0, help='how long to wait for a blocked way back, s')
    ap.add_argument('--max-travel', type=float, default=9.5)
    ap.add_argument('--max-time', type=float, default=25.0)
    ap.add_argument('--delay', type=float, default=2.0, help='countdown before moving, s')
    ap.add_argument('--lateral', choices=('center', 'start'), default='center',
                    help='steer for the middle of the hallway (default) or the start\'s lateral position')
    A = ap.parse_args()
    if A.ref_from_npz:
        e = ref_from_npz(A.ref_from_npz)
        if e is None or not all(np.isfinite([e['dB'], e['dL'], e['dR']])): raise SystemExit('walls not found: %s' % e)
        json.dump(dict(e, source=A.ref_from_npz, saved=time.strftime('%Y%m%d_%H%M%S')), open(REF, 'w'), indent=1)
        print('start recorded: ' + fmt(e)); return 0
    if A.go + A.dry + A.save != 1: ap.error('pass one of --go, --dry, --save')
    import signal
    signal.signal(signal.SIGTERM, AC._term)
    cfg = AC.read_vesc_yaml(AC.VESC_YAML)
    AC.rclpy.init()
    car = AC.Car()
    lines = []
    def log(s):
        line = '[%s] %s' % (time.strftime('%H:%M:%S'), s); print(line, flush=True); lines.append(line)
    code = 1
    try:
        t_end = car.now() + 8.0
        while car.now() < t_end and not (car.core and car.scan is not None): car.spin_for(0.1)
        if car.scan is None: raise SystemExit('no data on /scan')
        if A.save:
            scans = rest_scans(car, 1.5)
            e = estimate(median_scan(scans), self_mask(scans))
            if e is None or not all(np.isfinite([e['dB'], e['dL'], e['dR']])): raise SystemExit('walls not found: %s' % e)
            json.dump(dict(e, source='live', saved=time.strftime('%Y%m%d_%H%M%S')), open(REF, 'w'), indent=1)
            log('start recorded: ' + fmt(e)); return 0
        if not os.path.exists(REF): raise SystemExit('no start recorded (%s): run --save at the start spot' % REF)
        ref = json.load(open(REF))
        if A.dry:
            scans = rest_scans(car, 1.2); mask = self_mask(scans)
            e = estimate(median_scan(scans), mask)
            log('now:   ' + (fmt(e) if e else 'walls not found')); log('start: ' + fmt(ref))
            if e and np.isfinite(e['dB']):
                tr = e['dB'] - ref['dB']
                log('to go: %.2f m back, %+.2f m across, heading %+.1f deg; strip behind: nearest %.2f m'
                    % (tr, ref['y'] - e['y'], math.degrees(e['psi']), strip_blocked(median_scan(scans), mask, max(tr, 0) + 0.5)))
            return 0
        if not car.core: raise SystemExit('no data on /sensors/core (VESC)')
        log('backing up to the start in %.0f s - stand clear' % A.delay)
        car.spin_for(A.delay)
        car.moving_ok = True
        try:
            code, out = back_up(car, A, ref, cfg, log)
        except KeyboardInterrupt:
            log('interrupted'); code, out = 2, dict(reason='interrupted')
    finally:
        if A.go:
            car.moving_ok = True
            for _ in range(25):
                car.send(0.0, 0.0); car.spin_for(0.02)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    os.makedirs(AC.OUT_DIR, exist_ok=True)
    with open(os.path.join(AC.OUT_DIR, 'return_%s.txt' % stamp), 'w') as f: f.write('\n'.join(lines) + '\n')
    if out.get('trace'):
        np.savez_compressed(os.path.join(AC.OUT_DIR, 'return_%s.npz' % stamp), trace=np.array(out['trace']),
                            core=car.arr('core'), odom=car.arr('odom'), servo=car.arr('servo'))
    car.destroy_node(); AC.rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
