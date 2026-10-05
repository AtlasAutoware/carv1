#!/usr/bin/env python3
"""auto_calibrate: steering centre, steering gain and the odometry sign in one unattended run.

Put the car in a clear hallway about 1 m from the end wall, pointing down the hallway, start
this, and walk away. After the countdown it drives a short scripted sequence at the slowest
erpm-mode speed (about 1 m/s): three pairs of left/right arcs and two straight runs, stopping
fully between segments. Nobody pushes or steers anything. About 9 m of travel in total.

Measurements:
  steering    Yaw rate from the OAK-D gyro, projected on the gravity axis measured at rest.
              The camera's IMU is mounted on its side, so its own z axis is not vertical.
              Each constant-steering segment gives the real steering angle from the bicycle
              model, delta = atan(L * yaw / v). A line fitted to (servo command, delta) gives
              the servo value for straight ahead and the servo units per radian, i.e. the
              steering_angle_to_servo_offset and steering_angle_to_servo_gain for vesc.yaml.
  odom sign   /odom twist.linear.x while the car rolls forward (milestone T7). The direction
              of travel is checked independently with the lidar: the scans taken at rest
              before and after each segment are aligned (ICP), which gives the real motion.
  odom scale  That lidar motion against the distance implied by the VESC eRPM, as a check
              of speed_to_erpm_gain (3575).
  gyro check  The lidar's rotation per segment against the integrated gyro.

Safety: emergency stop on the front lidar sector and on both sides, a watchdog on every
sensor, fixed limits on time and distance, and zero speed on every exit. A segment does not
start until the path is clear, so a person walking past only pauses the run. Teleop (pilot
page keys, gamepad) outranks /drive in the mux, so anyone at the pilot page can take over.

    python3 auto_calibrate.py --dry            # sensors and preflight only; nothing moves
    python3 auto_calibrate.py --go --delay 10

Results go to ~/calib/calib_<time>.txt, .json and .npz. It never edits vesc.yaml.
"""
import argparse, json, math, os, re, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped
from vesc_msgs.msg import VescStateStamped

L = 0.324               # wheelbase, m (Traxxas Slash 4x4: 12.75 in)
LASER_X = 0.27          # base_link -> laser, from the static TF in bringup
WEB_SCALE = 0.34        # pilot page / gamepad steering scale, rad per unit (joy_teleop_f310.yaml)
VESC_YAML = os.path.expanduser('~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/vesc.yaml')
OUT_DIR = os.path.expanduser('~/calib')
trapz = getattr(np, 'trapezoid', None) or np.trapz


def read_vesc_yaml(path):
    t = open(path).read()
    num = lambda k, s=t: float(re.search(r'^\s*%s:\s*(-?[\d.]+)' % k, s, re.M).group(1))
    odom = t[t.index('vesc_to_odom_node:'):]
    mode = re.search(r'^\s*control_mode:\s*"?(\w+)"?', t, re.M)
    cfg = dict(gain=num('steering_angle_to_servo_gain'), offset=num('steering_angle_to_servo_offset'),
               smin=num('servo_min'), smax=num('servo_max'), erpm_gain=num('speed_to_erpm_gain'),
               odom_erpm_gain=num('speed_to_erpm_gain', odom), mode=mode.group(1) if mode else 'speed')
    if cfg['mode'] == 'erpm':
        cfg.update(max_speed=num('max_speed'), min_erpm=num('min_erpm'), max_erpm=num('max_erpm'),
                   erpm_deadband=num('erpm_deadband'))
    return cfg


# ---------------------------------------------------------------- speed (fast runs, --target)
# The motor, from the 10/5 runs at 0.8 m/s: ~60 ms from command to the first wheel movement, then a
# ramp of ~2 m/s^2; braking ~1.6 m/s^2 after a quick initial drop. DEC_SAFE is the braking assumed
# for safety margins, a little under what was logged.
LAT, ACC, DEC_SAFE = 0.06, 2.0, 1.5


def wheel_speed(cmd, cfg):
    """Speed at the wheels (m/s) that ackermann_to_vesc makes of a /drive speed."""
    if cfg.get('mode') != 'erpm': return abs(cmd)
    thr = min(abs(cmd) / cfg['max_speed'], 1.0)
    if thr < cfg['erpm_deadband']: return 0.0
    return (cfg['min_erpm'] + thr * (cfg['max_erpm'] - cfg['min_erpm'])) / cfg['erpm_gain']


def drive_cmd(v, cfg):
    """The /drive speed that gives v m/s at the wheels."""
    if cfg.get('mode') != 'erpm': return v
    thr = (v * cfg['erpm_gain'] - cfg['min_erpm']) / (cfg['max_erpm'] - cfg['min_erpm'])
    if not cfg['erpm_deadband'] <= thr <= 1.0:
        lo = wheel_speed(cfg['erpm_deadband'] * cfg['max_speed'], cfg); hi = wheel_speed(cfg['max_speed'], cfg)
        raise SystemExit('--target %.2f m/s: erpm mode can only hold %.2f..%.2f m/s' % (v, lo, hi))
    return thr * cfg['max_speed']


def power_dist(T, v):
    """Distance covered while the throttle is held for T s from rest (dead time, ramp, cruise)."""
    t = max(T - LAT, 0.0); ta = v / ACC
    return 0.5 * ACC * t * t if t <= ta else 0.5 * v * ta + v * (t - ta)


def power_time(s, v):
    """Inverse of power_dist."""
    ta = v / ACC; sa = 0.5 * v * ta
    return LAT + (math.sqrt(2 * s / ACC) if s <= sa else ta + (s - sa) / v)


def stop_dist(v):
    """Roll-out after a stop command at v, with margin."""
    return v * v / (2 * DEC_SAFE) + LAT * v + 0.15


# ---------------------------------------------------------------- lidar helpers
LIDAR_FLIP = False      # True undoes a front-to-back mirrored /scan. The car's was mirrored until
                        # 2026-10-05 (fixed in the driver: inverted + flip_x_axis in bringup_launch.py;
                        # checked while driving: lidar rotation and travel agree with gyro and odom)


def scan_arrays(m):
    r = np.asarray(m.ranges, np.float64)
    a = m.angle_min + np.arange(r.size) * m.angle_increment
    if LIDAR_FLIP: a = math.pi - a
    return (a + math.pi) % (2 * math.pi) - math.pi, r


def clear_dist(m, mask, lo_deg, hi_deg):
    """Nearest return in a sector (degrees, 0 = straight ahead, left positive)."""
    a, r = scan_arrays(m)
    d = np.degrees(a)
    sel = (d >= lo_deg) & (d <= hi_deg) & np.isfinite(r) & (r > 0.05)
    if mask is not None: sel &= ~mask
    return float(r[sel].min()) if sel.any() else float('inf')


def points(m, mask, floor=None):
    """Scan as x, y points in the base_link frame, in scan order. Bins whose range matches the
    floor model (see floor_line) are dropped: they are the floor, not an obstacle."""
    a, r = scan_arrays(m)
    ok = np.isfinite(r) & (r > 0.12) & (r < 10.0)
    if mask is not None: ok &= ~mask
    if floor is not None:
        with np.errstate(invalid='ignore'):
            ok &= ~(np.abs(r - floor) < 0.08)
    return np.c_[r[ok] * np.cos(a[ok]) + LASER_X, r[ok] * np.sin(a[ok])]


def floor_line(med, ang):
    """If the scan plane is tilted down it meets the floor along a straight line ahead, which
    then looks like a wall that moves with the car. Find it in the at-rest scan: the longest
    straight line ahead, facing the car, 1-5 m out. Returns per-bin floor ranges (NaN elsewhere)
    and (distance, normal angle deg, bins) or None."""
    deg = np.degrees(ang)
    sel = np.where((deg >= -60) & (deg <= 85) & np.isfinite(med) & (med > 1.0) & (med < 8.0))[0]
    floor = np.full(med.shape, np.nan)
    if len(sel) < 60: return floor, None
    P = np.c_[med[sel] * np.cos(ang[sel]), med[sel] * np.sin(ang[sel])]
    rng = np.random.default_rng(0); best, best_n = None, None
    for _ in range(400):
        i, j = rng.choice(len(P), 2, replace=False)
        d = P[j] - P[i]; nd = np.linalg.norm(d)
        if nd < 0.3: continue
        n = np.array([-d[1], d[0]]) / nd
        if P[i] @ n < 0: n = -n
        if not (1.0 < P[i] @ n < 5.0 and abs(math.atan2(n[1], n[0])) < math.radians(30)): continue
        inl = np.abs((P - P[i]) @ n) < 0.04
        if best is None or inl.sum() > best.sum(): best, best_n = inl, n
    if best is None or best.sum() < 60: return floor, None
    floor[sel[best]] = med[sel[best]]
    p = float(np.median(P[best] @ best_n))
    return floor, (p, math.degrees(math.atan2(best_n[1], best_n[0])), int(best.sum()))


def wall_heading(P, iters=400, tol=0.03):
    """Car heading relative to the hallway, from the longest straight wall within reach.
    Returns (psi, inlier count); psi = 0 when no wall within 50 deg of the car's axis is found."""
    if len(P) < 40: return 0.0, 0
    rng = np.random.default_rng(1); best_n, best = 0, None
    for _ in range(iters):
        i, j = rng.choice(len(P), 2, replace=False)
        d = P[j] - P[i]; nd = np.linalg.norm(d)
        if nd < 0.5: continue
        n = np.array([-d[1], d[0]]) / nd
        inl = np.abs((P - P[i]) @ n) < tol
        if inl.sum() > best_n: best_n, best = int(inl.sum()), inl
    if best is None or best_n < 40: return 0.0, best_n
    Q = P[best]
    w, v = np.linalg.eigh(np.cov((Q - Q.mean(0)).T))
    u = v[:, 1]                                       # direction of the wall
    # side walls run along the hallway and end walls across it, so either one gives the
    # hallway axis modulo 90 deg
    a = (math.atan2(u[1], u[0]) + math.pi / 4) % (math.pi / 2) - math.pi / 4
    return -a, best_n


FRONT_OVERHANG = 0.45   # rear axle (base_link) to front bumper, m, roughly


def path_clearance(m, mask, delta, dist, tail=0.35, floor=None):
    """Smallest distance from any lidar return to the rear-axle path: a constant-steering arc
    of length dist, then straight on for the roll-out (tail) and the front overhang, since the
    wheels are straightened when the car stops. Car half-width is about 0.15 m."""
    s = np.arange(0.0, dist + 0.025, 0.05)
    if abs(delta) < 1e-4:
        arc = np.c_[s, np.zeros_like(s)]; th_end = 0.0
    else:
        R = L / math.tan(delta); th = s / R          # signed radius, left positive
        arc = np.c_[R * np.sin(th), R * (1 - np.cos(th))]; th_end = dist / R
    hd = (s / R) if abs(delta) >= 1e-4 else np.zeros_like(s)
    u = np.arange(0.05, tail + 0.025, 0.05)
    rear = np.r_[arc, arc[-1] + np.c_[u * math.cos(th_end), u * math.sin(th_end)]]
    hd = np.r_[hd, np.full(len(u), th_end)]
    # sweep of the front bumper, which is what can hit something; points on the car's own
    # body behind the bumper are never near it
    path = rear + FRONT_OVERHANG * np.c_[np.cos(hd), np.sin(hd)]
    p = points(m, mask, floor)
    if not len(p): return float('inf')
    d = np.sqrt(((p[:, None, :] - path[None, :, :]) ** 2).sum(-1))
    path_clearance.nearest = tuple(p[d.min(1).argmin()])
    return float(d.min())


path_clearance.nearest = (float('nan'), float('nan'))


def normals(p):
    n = np.zeros_like(p)
    for i in range(len(p)):
        q = p[max(0, i - 3):i + 4]
        q = q[np.hypot(*(q - p[i]).T) < 0.25]
        if len(q) < 3: continue
        w, v = np.linalg.eigh(np.cov((q - q.mean(0)).T))
        n[i] = v[:, 0]
    return n


def icp(src, dst, init, iters=45):
    """Rigid 2D fit R(th) @ src + (x, y) ~= dst, starting from init = (x, y, th).
    With src = scan after a segment and dst = scan before it, (x, y, th) is the motion."""
    x, y, th = init
    if len(src) < 40 or len(dst) < 40: return None
    for it in range(iters):
        c, s = math.cos(th), math.sin(th)
        p = src @ np.array([[c, s], [-s, c]]) + (x, y)
        d2 = ((p[:, None, :] - dst[None, :, :]) ** 2).sum(-1)
        j = d2.argmin(1)
        dmin = np.sqrt(d2[np.arange(len(p)), j])
        ok = dmin < (0.6 if it < 12 else 0.3 if it < 25 else 0.12)
        if ok.sum() < 40: return None
        A, B = p[ok], dst[j[ok]]
        ma, mb = A.mean(0), B.mean(0)
        U, _, Vt = np.linalg.svd((A - ma).T @ (B - mb))
        D = np.diag([1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
        R = Vt.T @ D @ U.T
        dth = math.atan2(R[1, 0], R[0, 0]); dt = mb - R @ ma
        c2, s2 = math.cos(dth), math.sin(dth)
        x, y = c2 * x - s2 * y + dt[0], s2 * x + c2 * y + dt[1]
        th += dth
        if it > 25 and abs(dth) < 1e-6 and np.abs(dt).max() < 1e-6: break
    return dict(x=x, y=y, th=th, rms=float(np.sqrt((dmin[ok] ** 2).mean())), inl=float(ok.mean()),
                j=j, ok=ok)


def observability(dst, nrm, res):
    """How well the matched surfaces pin down translation. Returns (weak/strong eigenvalue
    ratio, share of the weak direction along x). A bare corridor gives (~0, ~1): no fix along it."""
    n = nrm[res['j'][res['ok']]]
    n = n[np.hypot(*n.T) > 0.5]
    if len(n) < 10: return 0.0, 1.0
    w, v = np.linalg.eigh(n.T @ n / len(n))
    return float(w[0] / max(w[1], 1e-9)), float(abs(v[0, 0]))


# ---------------------------------------------------------------- ROS side
class Car(Node):
    def __init__(self):
        super().__init__('auto_calibrate')
        q = qos_profile_sensor_data            # best effort: connects to any publisher
        self.imu, self.core, self.odom, self.servo = [], [], [], []
        self.scan, self.scan_t, self.mask, self.floor = None, 0.0, None, None
        self.create_subscription(Imu, '/oakd/imu', self._imu, q)
        self.create_subscription(VescStateStamped, '/sensors/core', self._core, q)
        self.create_subscription(Odometry, '/odom', self._odom, q)
        self.create_subscription(Float64, '/commands/servo/position', self._servo, q)
        self.create_subscription(LaserScan, '/scan', self._scan, q)
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.moving_ok = False                 # only --go sets this; send() is a no-op without it

    now = staticmethod(time.monotonic)

    def _imu(self, m):
        w, a = m.angular_velocity, m.linear_acceleration
        self.imu.append((self.now(), w.x, w.y, w.z, a.x, a.y, a.z))

    def _core(self, m): self.core.append((self.now(), float(m.state.speed)))
    def _odom(self, m): self.odom.append((self.now(), float(m.twist.twist.linear.x)))
    def _servo(self, m): self.servo.append((self.now(), float(m.data)))
    def _scan(self, m): self.scan, self.scan_t = m, self.now()

    def arr(self, name, t0=-1e18, t1=1e18):
        a = np.asarray(getattr(self, name), np.float64)
        if a.size == 0: return np.zeros((0, 7 if name == 'imu' else 2))
        return a[(a[:, 0] >= t0) & (a[:, 0] <= t1)]

    def spin_for(self, sec):
        t0 = self.now()
        while self.now() - t0 < sec: rclpy.spin_once(self, timeout_sec=0.01)

    def send(self, v, d):
        if not self.moving_ok: return
        m = AckermannDriveStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.drive.speed, m.drive.steering_angle = float(v), float(d)
        self.pub.publish(m)

    def hold(self, sec, v=0.0, d=0.0, check=None):
        """Publish (speed v, steering d) at 50 Hz for sec seconds. check() returning a string
        stops early with zero speed and returns that string."""
        t0 = self.now(); nxt = t0
        while self.now() - t0 < sec:
            if check is not None:
                r = check()
                if r:
                    self.send(0.0, d); return r
            if self.now() >= nxt:
                self.send(v, d); nxt += 0.02
            rclpy.spin_once(self, timeout_sec=0.005)
        return None

    def erpm(self):
        return abs(self.core[-1][1]) if self.core else 0.0

    def rest(self, d, timeout=4.0):
        """Zero speed (steering d) until the wheels have been still for 0.4 s."""
        t0 = self.now(); still = None
        while self.now() - t0 < timeout:
            self.hold(0.02, 0.0, d)
            if self.erpm() < 80:
                still = still or self.now()
                if self.now() - still > 0.4: break
            else:
                still = None
        return self.now()

    def fresh_scan(self, d, timeout=1.0):
        t0 = self.now()
        while self.scan_t <= t0 + 0.02 and self.now() - t0 < timeout:
            self.hold(0.02, 0.0, d)
        return self.scan


def steady(c, rec, up, bias, erpm_gain):
    """Medians over the part of a segment where the car rolled at its cruising speed."""
    w0, w1 = rec['t_go'] + 0.2, rec['t_stop']
    imu, core = c.arr('imu', w0, w1), c.arr('core', w0 - 0.1, w1 + 0.1)
    out = dict(n=0, yaw_raw=0.0, v=0.0, servo_cmd=float('nan'), odom_vx=float('nan'), erpm_sign=0.0)
    if len(imu) < 10 or len(core) < 3: return out
    v = np.interp(imu[:, 0], core[:, 0], np.abs(core[:, 1])) / erpm_gain
    sel = (v > 0.4) & (v > 0.85 * np.percentile(v, 90))
    if not sel.any(): return out
    yr = imu[:, 1:4] @ up - bias
    serv, od = c.arr('servo', w0, w1), c.arr('odom', w0, w1)
    cw = core[(core[:, 0] >= w0) & (core[:, 0] <= w1)]
    out.update(n=int(sel.sum()), yaw_raw=float(np.median(yr[sel])), v=float(np.median(v[sel])),
               servo_cmd=float(np.median(serv[:, 1])) if len(serv) else float('nan'),
               odom_vx=float(np.median(od[:, 1])) if len(od) else float('nan'),
               erpm_sign=float(np.sign(np.median(cw[:, 1]))) if len(cw) else 0.0)
    return out


def preflight(car, secs=2.5):
    """Car at rest: gravity axis, gyro bias, self-occluded lidar bins, clearances."""
    t_end = car.now() + 8.0
    while car.now() < t_end and not (car.imu and car.core and car.scan is not None):
        car.spin_for(0.1)
    missing = [n for n, ok in (('/oakd/imu', car.imu), ('/sensors/core', car.core),
                               ('/scan', car.scan is not None)) if not ok]
    if missing: raise SystemExit('no data on ' + ', '.join(missing))
    for attempt in range(12):                  # wait (up to ~30 s) for the car to be still
        t0 = car.now(); scans = []
        while car.now() - t0 < secs:
            car.spin_for(0.05)
            if car.scan is not None and (not scans or car.scan is not scans[-1]): scans.append(car.scan)
        t1 = car.now()
        imu, core = car.arr('imu', t0, t1), car.arr('core', t0, t1)
        if len(imu) < 50: raise SystemExit('IMU too slow: %d samples in %.1f s' % (len(imu), secs))
        acc, gyr = imu[:, 4:7], imu[:, 1:4]
        if acc.std(0).max() <= 0.5: break
        print('the car is moving or being handled (accel std %.2f); waiting' % acc.std(0).max(), flush=True)
    else:
        raise SystemExit('the car never stayed still')
    if len(core) and np.abs(core[:, 1]).max() > 100: raise SystemExit('wheels turning during preflight')
    g = acc.mean(0); up = g / np.linalg.norm(g)
    yaw = gyr @ up
    R = np.array([np.asarray(s.ranges, np.float64) for s in scans])
    Rm = np.where(np.isfinite(R) & (R > 0), R, np.inf).min(0)
    mask = Rm < 0.30                          # returns this close at rest are the car itself
    car.mask = mask
    s = scans[-1]
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        med = np.nanmedian(np.where(np.isfinite(R) & (R > 0), R, np.nan), 0)
    med[mask] = np.nan
    car.floor, fl = floor_line(med, scan_arrays(s)[0])
    wp = points(s, mask)
    psi0, nwall = wall_heading(wp[np.hypot(*wp.T) < 5.0])
    return dict(up=up, g=g, bias=float(yaw.mean()), noise=float(yaw.std()), imu_hz=len(imu) / (t1 - t0),
                core_hz=len(core) / (t1 - t0), scans=scans, n_masked=int(mask.sum()), floor=fl,
                psi0=float(psi0), nwall=nwall,
                front=clear_dist(s, mask, -30, 30), left=clear_dist(s, mask, 45, 135),
                right=clear_dist(s, mask, -135, -45), rear=clear_dist(s, mask, 150, 180))


class Run:
    """Drives the segments and keeps the car near the middle of the hallway, pointing along it."""

    def __init__(self, car, A, cfg, pre):
        self.car, self.A, self.cfg = car, A, cfg
        self.up, self.bias = pre['up'], pre['bias']
        self.sign = None                  # gyro sign along 'up', fixed by the first arc
        self.psi = pre.get('psi0', 0.0)   # heading relative to the hallway, rad (walls, then gyro)
        self.k, self.b = A.prior_k, A.prior_b   # actual = k * commanded + b, until measured
        self.npts = 0                     # segments behind the current k, b
        self.v = max(0.95, A.v_exp)       # planning speed until a segment measures it
        self.dist = 0.0
        self.recs = []
        self.t0 = car.now()
        self.lines = []

    def log(self, s):
        line = '[%5.1f s] %s' % (self.car.now() - self.t0, s)
        print(line, flush=True); self.lines.append(line)

    def clear(self):
        s, m = self.car.scan, self.car.mask
        return clear_dist(s, m, -30, 30), clear_dist(s, m, 45, 135), clear_dist(s, m, -135, -45)

    def hazard(self):
        c, t = self.car, self.car.now()
        if not c.imu or t - c.imu[-1][0] > 0.3: return 'IMU stale'
        if not c.core or t - c.core[-1][0] > 0.3: return 'VESC telemetry stale'
        if c.scan is None or t - c.scan_t > 0.4: return 'lidar stale'
        s, m = c.scan, c.mask
        # a fast run stops from further out, so it looks down a narrower cone: +-12 deg still covers
        # the car's width (+-0.15 m) beyond 0.7 m, and a 20 deg cone at 1.4 m would see the side wall
        # of a 2.4 m hallway whenever the car is turned toward it
        cone = 12 if self.A.fast else 20
        f = clear_dist(s, m, -cone, cone)
        if f < self.A.aeb_front: return 'obstacle ahead at %.2f m' % f
        side = min(clear_dist(s, m, 60, 120), clear_dist(s, m, -120, -60))
        if side < self.A.aeb_side: return 'side clearance %.2f m' % side
        return None

    def wait_clear(self, d, T):
        """Hold still until the predicted path of this segment has been clear for 1 s."""
        act = self.k * d + self.b
        # fast: the modelled distance under power (dead time, ramp, cruise) plus 20%, and the longer
        # roll-out of a faster car (A.tail). Planning the arc as if the car were at full speed at once
        # would over-predict how far it turns and block every arc in a 2.4 m hallway.
        dist = 1.2 * power_dist(T, max(self.v, self.A.v_exp)) if self.A.fast else self.v * T
        t0 = self.car.now(); since = None; said = False
        while self.car.now() - t0 < self.A.wait_clear:
            self.car.hold(0.1, 0.0, d)
            pc = path_clearance(self.car.scan, self.car.mask, act, dist, tail=self.A.tail, floor=self.car.floor)
            if pc >= self.A.path_margin:
                since = since or self.car.now()
                if self.car.now() - since >= 1.0: return True
            else:
                since = None
                if not said:
                    self.log('waiting: something %.2f m from the planned path' % pc); said = True
        return False

    def straight(self):
        return float(np.clip(-self.b / self.k, -0.1, 0.1)) if self.k > 0.2 else 0.0

    def segment(self, kind, delta, T):
        c = self.car
        if not self.wait_clear(delta, T):
            self.log('path never cleared; ending the run'); return None
        c.hold(0.5, 0.0, delta)           # servo in position before the wheels turn
        s0 = c.fresh_scan(delta)
        t_go = c.now()
        reason = c.hold(T, self.A.speed, delta, check=self.hazard)
        t_stop = c.now()
        t_rest = c.rest(self.straight())  # roll out with the wheels pointing straight
        s1 = c.fresh_scan(self.straight())
        rec = dict(kind=kind, delta=float(delta), T=float(T), t_go=t_go, t_stop=t_stop, t_rest=t_rest,
                   scan0=s0, scan1=s1, abort=reason)
        if reason: self.log('  stopped early: ' + reason)
        self.quick(rec)
        self.recs.append(rec)
        return rec

    def quick(self, rec):
        c, g = self.car, self.cfg['erpm_gain']
        core = c.arr('core', rec['t_go'], rec['t_rest'])
        rec['dist_erpm'] = float(trapz(np.abs(core[:, 1]), core[:, 0]) / g) if len(core) > 1 else 0.0
        self.dist += rec['dist_erpm']
        imu = c.arr('imu', rec['t_go'], rec['t_rest'])
        rec['dpsi_raw'] = float(trapz(imu[:, 1:4] @ self.up - self.bias, imu[:, 0])) if len(imu) > 1 else 0.0
        rec.update(steady(c, rec, self.up, self.bias, g))
        if self.sign is None and rec['kind'] == 'arc' and rec['n'] >= 15 and not rec['abort']:
            self.sign = 1.0 if rec['yaw_raw'] * rec['delta'] > 0 else -1.0
            self.log('gyro sign %+d (a %s arc read %+.3f rad/s about the gravity axis)'
                     % (self.sign, 'left' if rec['delta'] > 0 else 'right', rec['yaw_raw']))
        sg = self.sign or 1.0
        self.psi += sg * rec['dpsi_raw']
        if rec['n'] >= 15 and not rec['abort']: self.v = rec['v']
        pts = [(r['delta'], math.atan(L * sg * r['yaw_raw'] / r['v'])) for r in self.recs + [rec]
               if r['n'] >= 15 and not r['abort'] and r['v'] > 0.3]
        if len(pts) >= 2 and np.ptp([p[0] for p in pts]) > 0.05:
            self.k, self.b = (float(v) for v in np.polyfit(*np.array(pts).T, 1))
        elif len(pts) == 1:
            self.b = pts[0][1] - self.k * pts[0][0]
        self.npts = len(pts)
        act = math.atan(L * sg * rec['yaw_raw'] / rec['v']) if rec['v'] > 0.3 else float('nan')
        self.log('%-8s cmd %+.3f rad for %.2f s: %.2f m/s, actual %+.3f rad, %.2f m, heading now %+.1f deg'
                 '  (model k %.2f b %+.4f)' % (rec['kind'], rec['delta'], rec['T'], rec['v'], act,
                                               rec['dist_erpm'], math.degrees(self.psi), self.k, self.b))

    def lidar_agrees(self, rec):
        """After the first segment: the lidar must say the car went forward. If it says
        backward, the scan orientation assumed here is wrong and the emergency stop would be
        watching the wrong way, so stop."""
        p0, p1 = points(rec['scan0'], self.car.mask), points(rec['scan1'], self.car.mask)
        d, ps = rec['dist_erpm'], (self.sign or 1.0) * rec['dpsi_raw']
        fits = [r for r in (icp(p1, p0, (s * d * math.cos(ps / 2), s * d * math.sin(ps / 2), ps))
                            for s in (1, -1)) if r]
        if not fits:
            self.log('lidar check: no fit; continuing on the eRPM direction'); return True
        best = max(fits, key=lambda r: (round(r['inl'], 2), -r['rms']))
        self.log('lidar check: moved x %+.2f y %+.2f m, turned %+.1f deg (rms %.3f, inliers %.0f%%)'
                 % (best['x'], best['y'], math.degrees(best['th']), best['rms'], 100 * best['inl']))
        if best['x'] < -0.2:
            self.log('lidar says the car went BACKWARD: scan orientation is wrong; stopping'); return False
        return True

    def closing_time(self, d2, T1):
        act = self.k * d2 + self.b
        if act * self.psi >= 0: return None   # this arc would not bring the heading back
        if self.npts < 2: return T1           # no model yet: mirror the first arc
        if self.A.fast:                       # most of a fast arc is spent speeding up
            s = abs(self.psi) * L / max(math.tan(abs(act)), 1e-3)
            return float(np.clip(power_time(s, self.v) + 0.05, 0.4, 1.3 * T1))
        w = self.v * math.tan(abs(act)) / L
        return float(np.clip(abs(self.psi) / max(w, 0.05) + 0.12, 0.3, 1.2))

    def heading_fix(self, T):
        """Constant steering for a 'straight' segment that also brings the heading back to the
        hallway axis over its length. Still one constant command, so still a valid data point."""
        if self.npts < 2: return self.straight()
        act = math.atan(L * -self.psi / max(power_dist(T, self.v) if self.A.fast else self.v * T, 0.3))
        return float(np.clip((act - self.b) / self.k, -0.15, 0.15))

    def fast_plan(self):
        """Segments for a run above ~1 m/s. The car needs v / ACC seconds (and about a metre) to get
        up to speed, so segments are longer, and the steering angles are chosen so each arc still
        turns the car only ~9 and ~15 degrees: the same hallway has to hold the run."""
        v = self.A.v_exp; ta = LAT + v / ACC
        T_arc, T_str = ta + 0.25, ta + 0.45
        s = power_dist(T_arc, v)
        d1, d2 = (float(min(0.16, math.atan(dpsi * L / s))) for dpsi in (0.16, 0.26))
        return [('pair', d1, T_arc), ('pair', d2, T_arc), ('straight', T_str)]

    def execute(self):
        steps = [('pair', 0.12, 0.6), ('straight', 1.2), ('pair', 0.20, 0.55), ('straight', 1.2),
                 ('pair', 0.28, 0.5), ('straight', 1.2)]
        if self.A.fast:
            if abs(self.psi) > 0.2:           # no turning-back manoeuvre at speed
                self.log('pointing %+.0f deg off the hallway: line the car up with it and start again'
                         % math.degrees(self.psi))
                return self.car.rest(0.0)
            steps = self.fast_plan()
            self.log('fast plan at %.2f m/s: arcs %.3f / %.3f rad for %.2f s, straight %.2f s'
                     % (self.A.v_exp, steps[0][1], steps[1][1], steps[0][2], steps[2][1]))
        for _ in range(2):                    # line up with the hallway first if it is pointing off
            if abs(self.psi) <= 0.2: break
            cmd = -math.copysign(0.25, self.psi)
            act = self.k * cmd + self.b
            w = self.v * math.tan(abs(act)) / L
            T = float(np.clip(abs(self.psi) / max(w, 0.05) + 0.1, 0.3, 1.5))
            self.log('pointing %+.0f deg off the hallway; turning back' % math.degrees(self.psi))
            rec = self.segment('arc', cmd, T)
            if rec is None: return self.car.rest(0.0)
            if len(self.recs) == 1 and not self.lidar_agrees(rec): return self.car.rest(0.0)
        for st in steps:
            if self.dist > self.A.max_dist or self.car.now() - self.t0 > self.A.max_time:
                self.log('distance or time budget used up'); break
            if st[0] == 'straight':
                if self.segment('straight', self.heading_fix(st[1]), st[1]) is None: break
                continue
            _, d, T = st
            f, l, r = self.clear()
            if abs(self.psi) > 0.15:
                first = -math.copysign(d, self.psi)   # turn back toward the hallway axis first
            else:
                first = d if l >= r else -d       # otherwise toward the side with more room
            rec = self.segment('arc', first, T)
            if rec is None: break
            if len(self.recs) == 1 and not np.isfinite(rec['servo_cmd']):
                self.log('no servo commands came back: /drive is not reaching the VESC. Something on '
                         '/teleop (the pilot page) outranks it in the mux; ending'); break
            if len(self.recs) == 1 and not self.lidar_agrees(rec): break
            T2 = self.closing_time(-first, T)
            if T2 is None:
                self.log('heading already back within reach; skipping the closing arc'); continue
            if self.segment('arc', -first, T2) is None: break
        self.car.rest(0.0)
        self.log('done: %.1f m driven, heading %+.1f deg from the start' % (self.dist, math.degrees(self.psi)))


# ---------------------------------------------------------------- analysis
def analyse(run, pre, cfg, mask, floor=None):
    sg = run.sign or 1.0
    rows = []
    for i, r in enumerate(run.recs):
        w = dict(i=i, kind=r['kind'], cmd=r['delta'], T=r['T'], abort=r['abort'], v=r['v'], n=r['n'],
                 dist_erpm=r['dist_erpm'], odom_vx=r['odom_vx'], erpm_sign=r['erpm_sign'],
                 dpsi_gyro=sg * r['dpsi_raw'], yaw=sg * r['yaw_raw'])
        s = r['servo_cmd']
        w['servo'] = float(np.clip(s, cfg['smin'], cfg['smax'])) if np.isfinite(s) else float('nan')
        w['servo_clipped'] = bool(np.isfinite(s) and not cfg['smin'] <= s <= cfg['smax'])
        ok = r['v'] > 0.3 and r['n'] >= 15 and not r['abort']
        w['actual'] = math.atan(L * w['yaw'] / r['v']) if ok else float('nan')
        p0, p1 = points(r['scan0'], mask, floor), points(r['scan1'], mask, floor)
        best = None
        for d in (r['dist_erpm'], -r['dist_erpm']):      # try both directions of travel
            ps = w['dpsi_gyro']
            res = icp(p1, p0, (d * math.cos(ps / 2), d * math.sin(ps / 2), ps))
            if res and (best is None or res['inl'] > best['inl'] + 0.02
                        or (abs(res['inl'] - best['inl']) <= 0.02 and res['rms'] < best['rms'])):
                best = res                               # a wrong-way fit keeps far fewer inliers
        if best:
            ratio, weak_x = observability(p0, normals(p0), best)
            th, chord = best['th'], math.hypot(best['x'], best['y'])
            good = best['rms'] < 0.06 and best['inl'] > 0.65
            w.update(icp_x=best['x'], icp_y=best['y'], icp_th=th, icp_rms=best['rms'], icp_inl=best['inl'],
                     obs_ratio=ratio, obs_weak_x=weak_x, icp_good=good,
                     icp_arc=chord * ((th / 2) / math.sin(th / 2) if abs(th) > 1e-3 else 1.0),
                     icp_x_ok=good and not (ratio < 0.04 and weak_x > 0.8))
        rows.append(w)

    out = dict(rows=rows, sign=run.sign, config=cfg,
               imu=dict(up=pre['up'].tolist(), g=pre['g'].tolist(), bias=pre['bias'], noise=pre['noise'],
                        hz=pre['imu_hz']))
    pts = [(w['servo'], w['actual']) for w in rows if np.isfinite(w['servo']) and np.isfinite(w['actual'])]
    out['n_points'] = len(pts)
    if len(pts) >= 3 and np.ptp([p[0] for p in pts]) > 0.1:
        s, d = np.array(pts).T
        a, c0 = np.polyfit(s, d, 1)
        G, s0 = 1.0 / a, -c0 / a
        out.update(gain=float(G), offset=float(s0),
                   resid_deg=float(np.degrees(np.sqrt(np.mean((d - (a * s + c0)) ** 2)))),
                   pull_deg=float(np.degrees(a * cfg['offset'] + c0)),
                   throw_left=float((cfg['smin'] - s0) / G), throw_right=float((cfg['smax'] - s0) / G),
                   web_trim=float((s0 - cfg['offset']) / (cfg['gain'] * WEB_SCALE)))
    fw = [w['odom_vx'] for w in rows if not w['abort'] and w['n'] >= 15 and np.isfinite(w['odom_vx'])]
    out['odom_vx'] = float(np.median(fw)) if fw else float('nan')
    es = [w['erpm_sign'] for w in rows if w['n'] >= 15]
    out['erpm_sign'] = float(np.median(es)) if es else 0.0
    xs = [w['icp_x'] for w in rows if w.get('icp_x_ok')]
    out['lidar_forward_m'] = float(np.sum(xs)) if xs else float('nan')
    good = [w for w in rows if w.get('icp_x_ok') and w['dist_erpm'] > 0.3]
    if good:
        de, dl = sum(w['dist_erpm'] for w in good), sum(w['icp_arc'] for w in good)
        out.update(dist_erpm=de, dist_lidar=dl, erpm_gain_measured=cfg['erpm_gain'] * de / dl)
    gg = [w for w in rows if w.get('icp_good') and abs(w.get('icp_th', 0.0)) > 0.1]
    if gg:
        out['gyro_over_lidar'] = sum(abs(w['dpsi_gyro']) for w in gg) / sum(abs(w['icp_th']) for w in gg)
        out['gyro_sign_agrees'] = all(np.sign(w['dpsi_gyro']) == np.sign(w['icp_th']) for w in gg)
    return out


def report(out, pre, run_lines, stamp):
    cfg = out['config']; L_ = []
    p = L_.append
    p('auto_calibrate %s' % stamp)
    p('config: servo gain %.4f, offset %.4f, limits %.2f..%.2f; speed_to_erpm_gain %.0f (odom %.0f)'
      % (cfg['gain'], cfg['offset'], cfg['smin'], cfg['smax'], cfg['erpm_gain'], cfg['odom_erpm_gain']))
    u = pre['up']
    p('gravity axis in the IMU frame: (%+.3f, %+.3f, %+.3f); vertical IMU axis: %s'
      % (u[0], u[1], u[2], 'xyz'[int(np.argmax(np.abs(u)))]))
    p('gyro at rest: bias %+.4f rad/s, noise %.4f rad/s, %.0f Hz; sign from the first arc: %s'
      % (pre['bias'], pre['noise'], pre['imu_hz'], out['sign']))
    p('')
    p(' #  kind      cmd     servo   v     actual   gyro dpsi  lidar dpsi  lidar x,y         erpm m  odom vx  note')
    for w in out['rows']:
        lid = ('%+6.1f deg' % math.degrees(w['icp_th'])) if 'icp_th' in w else '   -     '
        xy = ('%+5.2f,%+5.2f%s' % (w['icp_x'], w['icp_y'], '' if w.get('icp_x_ok') else '?')) if 'icp_x' in w else '     -      '
        note = (w['abort'] or '') + (' servo clipped' if w['servo_clipped'] else '')
        p('%2d  %-8s %+.3f  %.4f  %.2f  %+.4f  %+6.1f deg  %s  %-16s %5.2f  %+.2f  %s'
          % (w['i'], w['kind'], w['cmd'], w['servo'], w['v'], w['actual'], math.degrees(w['dpsi_gyro']),
             lid, xy, w['dist_erpm'], w['odom_vx'], note))
    p('')
    if 'offset' in out:
        p('STEERING (%d segments, fit residual %.2f deg)' % (out['n_points'], out['resid_deg']))
        p('  steering_angle_to_servo_offset  %.4f -> %.4f' % (cfg['offset'], out['offset']))
        p('  steering_angle_to_servo_gain   %+.4f -> %+.4f' % (cfg['gain'], out['gain']))
        p('  told to go straight with the current config, the car steers %.2f deg to the %s'
          % (abs(out['pull_deg']), 'left' if out['pull_deg'] > 0 else 'right'))
        p('  same correction as a pilot-page trim of %+.2f' % out['web_trim'])
        p('  throw with the new centre: left %.3f rad, right %.3f rad (servo limits %.2f..%.2f)'
          % (out['throw_left'], -out['throw_right'], cfg['smin'], cfg['smax']))
    else:
        p('STEERING: not enough good segments for a fit (%d)' % out['n_points'])
    p('ODOMETRY (T7)')
    p('  lidar: the car moved %+.2f m along its own x axis over the segments it could measure'
      % out['lidar_forward_m'])
    vx = out['odom_vx']
    verdict = ('correct' if vx > 0 else 'INVERTED') if np.isfinite(vx) else 'unknown'
    p('  /odom twist.linear.x while rolling forward: %+.2f m/s -> odometry sign %s' % (vx, verdict))
    p('  VESC eRPM sign when rolling forward: %+d' % out['erpm_sign'])
    if 'erpm_gain_measured' in out:
        p('  distance: eRPM %.2f m vs lidar %.2f m -> speed_to_erpm_gain about %.0f (config %.0f)'
          % (out['dist_erpm'], out['dist_lidar'], out['erpm_gain_measured'], cfg['erpm_gain']))
    else:
        p('  distance: no segment where the lidar could fix the travel along the hallway')
    if 'gyro_over_lidar' in out:
        p('GYRO CHECK: gyro/lidar rotation %.3f, signs %s'
          % (out['gyro_over_lidar'], 'agree' if out['gyro_sign_agrees'] else 'DISAGREE'))
    p('')
    p('run log:')
    L_ += run_lines
    return '\n'.join(L_)


def jsonable(o):
    if isinstance(o, dict): return {k: jsonable(v) for k, v in o.items() if k not in ('j', 'ok')}
    if isinstance(o, (list, tuple)): return [jsonable(v) for v in o]
    if isinstance(o, np.ndarray): return jsonable(o.tolist())
    if isinstance(o, (np.floating, np.integer, np.bool_)): o = o.item()
    if isinstance(o, float) and not math.isfinite(o): return None
    return o


def _term(*_):
    raise KeyboardInterrupt


def speed_setup(A, cfg):
    """Settings that follow from the speed: --target -> --speed, the expected wheel speed, and for a
    fast run (above ~1 m/s) the longer front e-stop distance and roll-out allowance."""
    if A.target is not None: A.speed = drive_cmd(A.target, cfg)
    A.v_exp = wheel_speed(A.speed, cfg)
    if A.v_exp <= 0.0: raise SystemExit('/drive %.2f does not move the car (%s mode)' % (A.speed, cfg['mode']))
    if A.v_exp > 2.4: raise SystemExit('%.2f m/s is more than this routine is meant for' % A.v_exp)
    A.fast = A.v_exp > 1.05
    A.tail = stop_dist(A.v_exp) if A.fast else 0.35          # roll-out allowed for in the path check
    if A.fast:   # front lidar e-stop: scan age + dead time, braking, laser-to-bumper 0.18 m, margin
        A.aeb_front = max(A.aeb_front, 0.18 + 0.16 * A.v_exp + A.v_exp ** 2 / (2 * DEC_SAFE) + 0.2)
    print('/drive %.2f -> about %.2f m/s at the wheels (%s mode)%s' % (
        A.speed, A.v_exp, cfg['mode'], '; fast plan, emergency stop at %.2f m, roll-out allowance %.2f m'
        % (A.aeb_front, A.tail) if A.fast else ''), flush=True)


def main():
    ap = argparse.ArgumentParser(description='Unattended steering and odometry calibration (see the docstring).')
    ap.add_argument('--go', action='store_true', help='drive; without it nothing moves')
    ap.add_argument('--dry', action='store_true', help='preflight only')
    ap.add_argument('--delay', type=float, default=10.0, help='countdown before anything moves, s')
    ap.add_argument('--speed', type=float, default=0.3,
                    help='/drive speed; erpm mode turns 0.3 into about 0.96 m/s at the wheels')
    ap.add_argument('--aeb-front', type=float, default=0.6,
                    help='emergency stop if anything is this close within 20 deg of straight ahead, m')
    ap.add_argument('--aeb-side', type=float, default=0.22, help='emergency stop if a wall is this close to the side, m')
    ap.add_argument('--path-margin', type=float, default=0.30,
                    help='a segment starts only if its planned path keeps this far from every lidar return, m')
    ap.add_argument('--wait-clear', type=float, default=12.0, help='how long to wait for a blocked path, s')
    ap.add_argument('--prior-k', type=float, default=1.05,
                    help='planning guess for actual/commanded steering before the run measures it')
    ap.add_argument('--prior-b', type=float, default=0.0,
                    help='planning guess for the steering bias, rad (0.07 before the 10/5 calibration)')
    ap.add_argument('--max-dist', type=float, default=14.0)
    ap.add_argument('--max-time', type=float, default=180.0)
    ap.add_argument('--target', type=float, default=None,
                    help='speed at the wheels, m/s, instead of --speed (erpm mode: 0.79..2.33). Above ~1 m/s '
                         'the run uses the fast plan: longer segments, smaller steering angles, and an '
                         'emergency stop and path check sized for the longer stop')
    A = ap.parse_args()
    if A.go == A.dry: ap.error('pass exactly one of --dry or --go')
    import signal
    signal.signal(signal.SIGTERM, _term)
    cfg = read_vesc_yaml(VESC_YAML)
    speed_setup(A, cfg)
    os.makedirs(OUT_DIR, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    rclpy.init()
    car = Car()
    try:
        if A.go:
            print('moving in %.0f s - stand clear' % A.delay, flush=True)
            car.spin_for(A.delay)
        pre = preflight(car)
        print('preflight: IMU %.0f Hz, VESC %.0f Hz, gravity axis (%+.2f %+.2f %+.2f), gyro bias %+.4f rad/s, '
              'noise %.4f; %d lidar bins masked as the car; clear front %.2f, left %.2f, right %.2f, rear %.2f m'
              % (pre['imu_hz'], pre['core_hz'], *pre['up'], pre['bias'], pre['noise'], pre['n_masked'],
                 pre['front'], pre['left'], pre['right'], pre['rear']), flush=True)
        print('heading relative to the hallway walls: %+.1f deg (%d wall points)'
              % (math.degrees(pre['psi0']), pre['nwall']), flush=True)
        pc = path_clearance(pre['scans'][-1], car.mask, 0.0, 1.0)
        # every segment checks its own planned path before it moves; here only the sides
        room = min(pre['left'], pre['right']) >= A.aeb_side + 0.15
        if A.dry:
            s = pre['scans']
            res = icp(points(s[-1], car.mask), points(s[0], car.mask), (0.0, 0.0, 0.0)) if len(s) > 1 else None
            if res:
                print('lidar self-check, two scans at rest: dx %+.3f dy %+.3f m, dth %+.2f deg, rms %.3f m, '
                      'inliers %.0f%%' % (res['x'], res['y'], math.degrees(res['th']), res['rms'], 100 * res['inl']))
            print('room to run here: %s (a 3 m straight path keeps %.2f m from everything, nearest return at '
                  'x %.2f y %.2f; needs %.2f)' % ('yes' if room else 'NO', pc, *path_clearance.nearest,
                                                 A.path_margin + 0.1))
            return 0
        if not room:
            raise SystemExit('not enough room: a 3 m straight path comes within %.2f m of something; '
                             'left %.2f, right %.2f m' % (pc, pre['left'], pre['right']))
        car.moving_ok = True
        run = Run(car, A, cfg, pre)
        try:
            run.execute()
        except KeyboardInterrupt:
            run.log('interrupted')
    finally:
        if A.go:
            car.moving_ok = True
            for _ in range(25):
                car.send(0.0, 0.0); car.spin_for(0.02)
    out = analyse(run, pre, cfg, car.mask)
    txt = report(out, pre, run.lines, stamp)
    base = os.path.join(OUT_DIR, 'calib_' + stamp)
    with open(base + '.txt', 'w') as f: f.write(txt + '\n')
    with open(base + '.json', 'w') as f: json.dump(jsonable(out), f, indent=1)
    sc = [[np.asarray(r['scan0'].ranges, np.float32), np.asarray(r['scan1'].ranges, np.float32)]
          for r in run.recs]
    np.savez_compressed(base + '.npz', imu=car.arr('imu'), core=car.arr('core'), odom=car.arr('odom'),
                        servo=car.arr('servo'), scans=np.array(sc), mask=car.mask,
                        seg=np.array([[r['t_go'], r['t_stop'], r['t_rest'], r['delta']] for r in run.recs]))
    print(txt)
    print('\nsaved %s.{txt,json,npz}' % base)
    car.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
