#!/usr/bin/env python3
"""car_model: how car 1 differs from the plain sim, for evaluation and data collection under
car conditions (ml/sim_rollout.py --car). ROS-free, numpy.

Measured on the car or read from its configuration on Oct 5, 2026:
  geometry  base_link is the rear axle. The static TFs in f1tenth_stack bringup put the laser
            0.27 m and the camera 0.30 m ahead of it. The plain sim casts the lidar and renders
            the camera from the rear axle, so a policy trained there sees the car's walls 0.27 m
            closer than it learned. Footprint: Traxxas Slash 4x4, 568 x 296 mm, rear axle
            ~0.12 m from the rear bumper and ~0.45 m from the front one.
  speed     ackermann_to_vesc in control_mode 'erpm' (policy_io.erpm_wheel_speed): /drive 1.0
            gives 1.03 m/s, nothing between 0 and 0.78 m/s exists.
  motor     auto_calibrate logs (10/5 11:39, 0.8 m/s): ~60 ms from command to the first wheel
            movement, then a ramp of ~2 m/s^2 (0.32 m/s at 0.21 s, 0.69 at 0.41 s); after a stop
            command ~25 ms, a quick drop, then ~1.6 m/s^2 (0.8 m/s to rest in ~0.33 s, ~0.15 m).
  steering  calibrated gain/offset (fit residual 0.4 deg). servo_max 0.90 with the centre at
            0.6705 leaves ~0.25 rad of right lock against ~0.56 left until the servo horn is
            re-centred.
  bridge    policy_bridge: 10 Hz; NaN -> stop; steer clipped to +-max_steer (0.4) and speed to
            [0, max_speed] (1.0); AEB below 0.35 m within +-0.2 rad of the scan; no route, or
            inside goal_tol (0.4 m) of the goal, -> speed 0 and steering 0.
  sensing   RPLidar C1 on the bench: 581 of 720 bins valid (~19% empty); map-frame pose from
            SLAM / the particle filter, with noise and ~0.1 s of lag; the route replanned from that
            pose every second (route_source.RouteSource, the bridge's planner).
Guesses, not measured (flagged so nobody mistakes them for data): servo slew 3 rad/s at the
wheels, 20 ms servo dead time, lidar/camera ages of 80/100 ms, pose noise 5 cm / 0.03 rad.
Not modelled at all: what the OAK-D image looks like (the sim render is not a photo), lidar
motion distortion, tyre slip.
"""
import json, math, os, sys
from collections import deque
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'f1tenth_gym_ros'))
import policy_io as PIO                                                  # noqa: E402

WHEELBASE_CAR = 0.324           # Traxxas Slash 4x4: 12.75 in (vesc.yaml)
FRONT, REAR, HALF_W = 0.45, 0.12, 0.15

# 'car' = car 1 as it is on 10/5 with today's policy_bridge (no speed inversion, max_speed 1.0)
CAR = {
    'lidar_x': 0.27, 'cam_x': 0.30,          # sensors ahead of the rear axle, m
    'lidar_lat': 0.08, 'cam_lat': 0.10,      # age of the observation the policy acts on, s
    'lidar_drop': 0.19, 'lidar_noise': 0.03,  # share of empty bins, range noise sd (m)
    'cam': 'render',                         # 'render' (sim image), 'perturb' (perturb_obs camera proxy), 'blank'
    'pose_noise': 0.05, 'yaw_noise': 0.03, 'pose_lag': 0.10, 'goal_tol': 0.4,   # route hint, as route_source
    'replan': 1.0,                           # s between A* replans from the car's pose (policy_bridge: 1.0);
                                             # 0 = the task's path, planned once from the start (the plain sim)
    'max_speed': 1.0, 'max_steer': 0.4, 'aeb': 0.35,   # policy_bridge
    'speed_map': 'raw',                      # 'raw' (/drive into erpm mode), 'inverse' (bridge inverts it), 'linear'
    'bridge_shift': False,                   # bridge shifts the scan by lidar_x minus the model's training lidar_x
    'coast_below': 0.2,                      # 'inverse': asked speeds below this -> 0
    'dead': 0.06, 'acc': 2.0, 'dec': 2.2,    # motor: dead time s, ramp up / down m/s^2 (stops a bit longer than logged)
    'steer_dead': 0.02, 'steer_rate': 3.0,   # servo dead time s, slew rad/s at the wheels
    'steer_left': 0.56, 'steer_right': 0.25,  # physical lock, rad
    'steer_gain': 1.0, 'steer_bias': 0.0,    # actual angle = gain * commanded + bias
    'wheelbase': WHEELBASE_CAR,
    'footprint': True,                       # collision = any part of the car body, not the rear axle point
    'instruction': 'task',                   # 'task' (the task's route description) or a fixed string
    'expert_vmax': 1.5,                      # top speed of the pure-pursuit expert (sim default 1.5)
    'dart': 0.0, 'dart_tau': 0.5,            # collection: OU noise (sd rad, time constant s) on the expert's executed steering
}
PRESETS = {
    'car': {},                                               # today: policy_bridge as of 10/5 morning
    'car_bridge': {'speed_map': 'inverse', 'bridge_shift': True},   # with the 10/5 policy_bridge fixes
    'car_fixed': {'speed_map': 'inverse', 'bridge_shift': True, 'steer_right': 0.56},   # ... and the servo horn re-centred
    # data collection. Demos: the fixed bridge, the expert at the car's top speed (1.0), a little
    # steering noise for recovery examples, symmetric lock (the asymmetry is a fault to fix, not
    # something to learn). DAgger: exactly the car as it will be driven, the student at the wheel.
    'car_train': {'speed_map': 'inverse', 'bridge_shift': True, 'expert_vmax': 1.0, 'dart': 0.03, 'steer_right': 0.56},
    'car_dagger': {'speed_map': 'inverse', 'bridge_shift': True, 'expert_vmax': 1.0},
}


def resolve(preset, sets=()):
    """Preset name + 'key=value' overrides (JSON values) -> full config dict."""
    if preset not in PRESETS: raise SystemExit(f'unknown car preset {preset!r}; presets: {", ".join(PRESETS)}')
    c = dict(CAR); c.update(PRESETS[preset])
    for kv in sets:
        k, v = kv.split('=', 1)
        if k not in CAR: raise SystemExit(f'unknown car setting {k!r}; settings: {", ".join(CAR)}')
        try: c[k] = json.loads(v)
        except ValueError: c[k] = v
    if c['speed_map'] not in ('raw', 'inverse', 'linear'): raise SystemExit(f"bad speed_map {c['speed_map']!r}")
    if c['cam'] not in ('render', 'perturb', 'blank'): raise SystemExit(f"bad cam {c['cam']!r}")
    return c


def wheel_target(v, c):
    """policy_bridge speed (m/s, already clipped to [0, max_speed]) -> the speed the VESC holds."""
    if c['speed_map'] == 'linear': return v
    cmd = v if c['speed_map'] == 'raw' else PIO.erpm_command(v, c['coast_below'])
    return PIO.erpm_wheel_speed(cmd)


def bridge(v, s, scan, inc, c):
    """policy_bridge's checks on the network output: NaN -> stop, clips, forward-cone AEB."""
    if not (math.isfinite(v) and math.isfinite(s)): return 0.0, 0.0
    s = max(-c['max_steer'], min(c['max_steer'], s)); v = max(0.0, min(c['max_speed'], v))
    if PIO.front_clear(scan, -math.pi, inc) < c['aeb']: v = 0.0
    return v, s


class Delay:
    """A fixed transport delay of n steps (n = 0: none)."""
    def __init__(self, sec, dt):
        self.q = deque([0.0] * int(round(sec / dt)))

    def __call__(self, x):
        if not self.q: return x
        self.q.append(x); return self.q.popleft()


class Motor:
    """Wheel speed: dead time, then a ramp toward the target (no reverse)."""
    def __init__(self, c, dt):
        self.delay = Delay(c['dead'], dt); self.up, self.down = c['acc'] * dt, c['dec'] * dt

    def step(self, v, target):
        dv = self.delay(target) - v
        return max(0.0, v + (min(dv, self.up) if dv > 0 else max(dv, -self.down)))


class Servo:
    """Steering angle at the wheels: dead time, calibration error, physical lock, slew rate."""
    def __init__(self, c, dt):
        self.delay = Delay(c['steer_dead'], dt); self.rate = c['steer_rate'] * dt
        self.lo, self.hi = -c['steer_right'], c['steer_left']; self.g, self.b = c['steer_gain'], c['steer_bias']

    def step(self, cur, cmd):
        tgt = min(self.hi, max(self.lo, self.g * self.delay(cmd) + self.b))
        return cur + max(-self.rate, min(self.rate, tgt - cur))


def _footprint_pts(step=0.05):
    """Points on the outline of the car body in the base_link frame (x forward, y left)."""
    xs = np.arange(-REAR, FRONT + 1e-9, step); ys = np.arange(-HALF_W, HALF_W + 1e-9, step)
    P = [np.c_[xs, np.full_like(xs, HALF_W)], np.c_[xs, np.full_like(xs, -HALF_W)],
         np.c_[np.full_like(ys, FRONT), ys], np.c_[np.full_like(ys, -REAR), ys]]
    return np.unique(np.round(np.vstack(P), 4), axis=0)


FOOTPRINT = _footprint_pts()


def body_hits(sm, st):
    """True if any point of the car's outline is in an occupied (or off-map) cell."""
    x, y, th = st[0], st[1], st[2]; c, s = math.cos(th), math.sin(th)
    px = x + c * FOOTPRINT[:, 0] - s * FOOTPRINT[:, 1]; py = y + s * FOOTPRINT[:, 0] + c * FOOTPRINT[:, 1]
    col = np.floor((px - sm.ox) / sm.res).astype(np.int64); row = np.floor(sm.H - (py - sm.oy) / sm.res).astype(np.int64)
    oob = (col < 0) | (col >= sm.W) | (row < 0) | (row >= sm.H)
    if oob.any(): return True
    return bool(sm.occ[row, col].any())


class History:
    """Recent true states, to read the one `lag` seconds ago (sensor and pose latency)."""
    def __init__(self, keep):
        self.keep = keep; self.buf = deque()

    def add(self, t, st):
        self.buf.append((t, st.copy()))
        while len(self.buf) > 1 and self.buf[1][0] <= t - self.keep: self.buf.popleft()

    def at(self, t):
        """Latest state at or before time t (the oldest kept one if t is earlier)."""
        best = self.buf[0][1]
        for ti, s in self.buf:
            if ti <= t + 1e-9: best = s
            else: break
        return best


class Hint:
    """The car-side route hint on a noisy, lagged map-frame pose, with route_source's rules (no
    hint inside goal_tol). replan = 0: along the task's path, planned once from the start.
    replan > 0: what policy_bridge does, route_source.RouteSource replanning A* from the car's
    pose on the map every `replan` s; `path` is then the current route (None when it has none)."""
    def __init__(self, path, c, rng, map_name=None):
        self.path, self.c, self.rng = path, c, rng
        self.goal = path[-1]; self.rs = None; self.t_plan = -1e9
        if c['replan'] > 0:
            from route_source import RouteSource, occ_from_grid
            from sim_core import load_map
            occ, res, origin = load_map(os.path.join(REPO, 'maps', f'{map_name}.yaml'))
            msg = (np.flipud(occ).astype(np.int8) * 100).ravel()          # what /map carries
            self.rs = RouteSource(goal_tol=c['goal_tol'])
            self.rs.set_map(occ_from_grid(msg, occ.shape[1], occ.shape[0]), res, origin)
            self.rs.set_goal(tuple(self.goal)); self.path = None

    def __call__(self, st_lagged, t=0.0):
        c = self.c
        x = st_lagged[0] + self.rng.normal(0, c['pose_noise']); y = st_lagged[1] + self.rng.normal(0, c['pose_noise'])
        th = st_lagged[2] + self.rng.normal(0, c['yaw_noise'])
        if self.rs is None:
            if math.hypot(self.goal[0] - x, self.goal[1] - y) < c['goal_tol']: return None
            return PIO.route_hint((x, y, th), self.path)
        if t - self.t_plan >= c['replan']:
            self.t_plan = t; self.rs.replan((x, y, th), t)
        h, _ = self.rs.hint((x, y, th), t)
        self.path = self.rs.path
        return h


class OU:
    """Ornstein-Uhlenbeck noise sampled every dt (DART-style perturbation of the expert)."""
    def __init__(self, sd, tau, dt, rng):
        self.a = math.exp(-dt / tau) if tau > 0 else 0.0; self.sd, self.rng, self.x = sd, rng, 0.0

    def __call__(self):
        if self.sd <= 0: return 0.0
        self.x = self.a * self.x + self.sd * math.sqrt(1 - self.a * self.a) * self.rng.normal()
        return self.x
