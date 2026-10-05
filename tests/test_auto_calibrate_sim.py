"""
tools/auto_calibrate.py driven against a simulated car in a simulated hallway.
==============================================================================

No ROS and no car: the ROS modules are stubs, and the Car node is replaced by SimCar, which
runs a kinematic car with the motor, servo and VESC behaviour measured on car 1 (10/5) in a
2.4 m wide hallway shaped like the one used that day (left wall 1.5 m, right wall 0.9 m, end
wall 8.8 m ahead). The car's true steering map differs from the configured one, so the run has
something to find. Run.execute(), analyse() and report() are the script's own code.

Checks: the stock run (0.8 m/s) and the fast plan (--target 1.3 and 1.5 m/s) finish without
touching a wall, the fast run never comes closer to the end wall than its stop allowance, and
the fitted steering centre and gain and the eRPM gain match the simulated truth.

    python3 -m pytest tests/test_auto_calibrate_sim.py -q
"""
import argparse, math, os, sys
from collections import deque
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

VESC_YAML = """/**:
  ros__parameters:
    speed_to_erpm_gain: 4285.0
    speed_to_erpm_offset: 0.0
    steering_angle_to_servo_gain: -0.9175
    steering_angle_to_servo_offset: 0.6705
    control_mode: "erpm"
    max_speed: 5.0          # m/s at full stick
    min_erpm: 3000.0
    max_erpm: 10000.0
    erpm_deadband: 0.05
    servo_min: 0.15
    servo_max: 0.90
vesc_to_odom_node:
  ros__parameters:
    speed_to_erpm_gain: 4285.0
"""
TRUE_S0, TRUE_G = 0.6669, -0.9175      # the simulated car's real straight-ahead servo value and gain
L = 0.324
LEFT, RIGHT, END, BACK = 1.5, -0.9, 8.77, -1.7      # hallway walls (base_link start at the origin)


@pytest.fixture
def AC(monkeypatch, tmp_path):
    for n in ('rclpy', 'rclpy.node', 'rclpy.qos', 'sensor_msgs', 'sensor_msgs.msg', 'nav_msgs', 'nav_msgs.msg',
              'std_msgs', 'std_msgs.msg', 'ackermann_msgs', 'ackermann_msgs.msg', 'vesc_msgs', 'vesc_msgs.msg'):
        monkeypatch.setitem(sys.modules, n, ModuleType(n))
    r = sys.modules['rclpy']
    r.spin_once = lambda node, timeout_sec=None: node.advance(timeout_sec or 0.005)
    r.init = r.shutdown = lambda *a, **k: None
    sys.modules['rclpy.node'].Node = object
    sys.modules['rclpy.qos'].qos_profile_sensor_data = None
    for mod, names in (('sensor_msgs.msg', ('Imu', 'LaserScan')), ('nav_msgs.msg', ('Odometry',)),
                       ('std_msgs.msg', ('Float64',)), ('ackermann_msgs.msg', ('AckermannDriveStamped',)),
                       ('vesc_msgs.msg', ('VescStateStamped',))):
        for c in names: setattr(sys.modules[mod], c, object)
    sys.path.insert(0, os.path.join(REPO, 'tools'))
    monkeypatch.delitem(sys.modules, 'auto_calibrate', raising=False)
    import auto_calibrate
    y = tmp_path / 'vesc.yaml'; y.write_text(VESC_YAML)
    auto_calibrate.TEST_CFG = auto_calibrate.read_vesc_yaml(str(y))
    return auto_calibrate


def make_sim_car(AC, seed=0):
    class SimCar(AC.Car):
        """The interface Run, preflight() and analyse() use, on a simulated car."""
        def __init__(self):
            self.rng = np.random.default_rng(seed); self.t = 0.0; self.dt = 0.005
            self.imu, self.core, self.odom, self.servo = [], [], [], []
            self.scan, self.scan_t, self.mask, self.floor = None, 0.0, None, None
            self.moving_ok = False
            self.x = self.y = self.th = self.v = self.delta = 0.0
            self.cmd = (0.0, 0.0); self.cmd_t = -1.0
            self.q = deque([0.0] * int(round(0.06 / self.dt)))          # motor dead time
            self.nxt = {'imu': 0.0, 'core': 0.0, 'scan': 0.0}
            self.min_clear = 99.0; self.hit = False; self.max_x = 0.0
            self.ang = -math.pi + 2 * math.pi * np.arange(720) / 720

        def now(self): return self.t

        def send(self, v, d):
            if not self.moving_ok: return
            self.cmd = (float(v), float(d)); self.cmd_t = self.t
            self.servo.append((self.t, AC.TEST_CFG['gain'] * float(d) + AC.TEST_CFG['offset']))

        def advance(self, sec):
            for _ in range(max(1, int(round(sec / self.dt)))): self._step()

        def _step(self):
            cfg = AC.TEST_CFG; dt = self.dt
            v_cmd, d_cmd = self.cmd if self.t - self.cmd_t < 0.2 else (0.0, self.cmd[1])   # mux timeout
            servo = min(cfg['smax'], max(cfg['smin'], cfg['gain'] * d_cmd + cfg['offset']))
            tgt_d = (servo - TRUE_S0) / TRUE_G
            self.delta += max(-3.0 * dt, min(3.0 * dt, tgt_d - self.delta))
            self.q.append(AC.wheel_speed(v_cmd, cfg)); tgt_v = self.q.popleft()
            dv = tgt_v - self.v
            self.v = max(0.0, self.v + (min(dv, 2.0 * dt) if dv > 0 else max(dv, -1.6 * dt)))
            w = self.v * math.tan(self.delta) / L
            self.x += self.v * math.cos(self.th) * dt; self.y += self.v * math.sin(self.th) * dt; self.th += w * dt
            self.t += dt
            # body outline vs the walls
            c, s = math.cos(self.th), math.sin(self.th)
            pts = [(self.x + c * a - s * b, self.y + s * a + c * b) for a in (-0.12, 0.45) for b in (-0.15, 0.15)]
            for px, py in pts:
                clear = min(LEFT - py, py - RIGHT, END - px, px - BACK)
                self.min_clear = min(self.min_clear, clear); self.hit |= clear <= 0.0
            self.max_x = max(self.max_x, self.x + 0.45 * c)
            n = self.rng.normal
            if self.t >= self.nxt['imu']:
                self.nxt['imu'] += 1 / 200.0
                self.imu.append((self.t, -w + n(0, 0.0035), n(0, 0.0035), n(0, 0.0035), -9.81 + n(0, 0.02), n(0, 0.02), 0.8))
            if self.t >= self.nxt['core']:
                self.nxt['core'] += 1 / 50.0
                e = self.v * 4285.0; e = 0.0 if e < 900 else e                  # the VESC reads 0 below ~900 eRPM
                self.core.append((self.t, e)); self.odom.append((self.t, e / 4285.0))
            if self.t >= self.nxt['scan']:
                self.nxt['scan'] += 0.1
                lx, ly = self.x + 0.27 * c, self.y + 0.27 * s; a = self.th + self.ang
                ca, sa = np.cos(a), np.sin(a)
                with np.errstate(divide='ignore', invalid='ignore'):
                    d = np.stack([np.where(sa > 1e-9, (LEFT - ly) / sa, np.inf), np.where(sa < -1e-9, (RIGHT - ly) / sa, np.inf),
                                  np.where(ca > 1e-9, (END - lx) / ca, np.inf), np.where(ca < -1e-9, (BACK - lx) / ca, np.inf)])
                r = d.min(0) + n(0, 0.01, 720); r[r > 12.0] = np.inf
                self.scan = SimpleNamespace(ranges=r.astype(np.float32).tolist(), angle_min=-math.pi,
                                            angle_increment=2 * math.pi / 720)
                self.scan_t = self.t
    return SimCar()


def args(AC, **kw):
    A = argparse.Namespace(go=True, dry=False, delay=0.0, speed=0.3, aeb_front=0.6, aeb_side=0.22, path_margin=0.30,
                           wait_clear=3.0, prior_k=1.05, prior_b=0.0, max_dist=14.0, max_time=180.0, target=None)
    for k, v in kw.items(): setattr(A, k, v)
    AC.speed_setup(A, AC.TEST_CFG)
    return A


def run(AC, A, seed=0):
    car = make_sim_car(AC, seed)
    pre = AC.preflight(car)
    car.moving_ok = True
    r = AC.Run(car, A, AC.TEST_CFG, pre); r.execute()
    out = AC.analyse(r, pre, AC.TEST_CFG, car.mask, car.floor)
    print(AC.report(out, pre, r.lines, 'test'))
    return car, r, out


def test_speed_mapping(AC):
    cfg = AC.TEST_CFG
    assert cfg['mode'] == 'erpm' and cfg['max_speed'] == 5.0 and cfg['min_erpm'] == 3000.0
    assert AC.wheel_speed(0.3, cfg) == pytest.approx(0.798, abs=1e-3)
    assert AC.wheel_speed(AC.drive_cmd(1.5, cfg), cfg) == pytest.approx(1.5)
    with pytest.raises(SystemExit): AC.drive_cmd(0.5, cfg)                # below the erpm floor
    with pytest.raises(SystemExit): AC.drive_cmd(2.5, cfg)                # above full throttle
    assert AC.power_time(AC.power_dist(1.1, 1.5), 1.5) == pytest.approx(1.1)
    assert AC.power_time(AC.power_dist(0.4, 1.5), 1.5) == pytest.approx(0.4)
    A = args(AC); assert not A.fast and A.aeb_front == 0.6 and A.tail == 0.35          # stock run unchanged
    A = args(AC, target=1.5); assert A.fast and A.aeb_front > 1.2 and A.tail > 0.9


def test_stock_run_in_the_hallway(AC):
    car, r, out = run(AC, args(AC))
    assert not car.hit and len(r.recs) == 9 and not any(x['abort'] for x in r.recs)
    assert out['offset'] == pytest.approx(TRUE_S0, abs=0.004) and out['gain'] == pytest.approx(TRUE_G, rel=0.04)
    # reads ~3-4% low here: the VESC reports 0 eRPM below ~900 while the wheels still roll, so the eRPM
    # distance misses the last ~1.5 cm of every stop (the car's own log shows the same cut-off)
    assert 0.94 * 4285 < out['erpm_gain_measured'] < 1.0 * 4285


@pytest.mark.parametrize('target', [1.3, 1.5])
def test_fast_run_in_the_hallway(AC, target):
    A = args(AC, target=target)
    car, r, out = run(AC, A)
    v = [x['v'] for x in r.recs if x['n'] >= 15]
    assert not car.hit and car.min_clear > 0.15
    assert END - car.max_x > 0.3                                     # stopped well short of the end wall
    assert len(r.recs) >= 3 and min(v) > 0.85 * target               # it really ran at speed
    assert not any(x['abort'] for x in r.recs)                       # no emergency stops
    if out.get('n_points', 0) >= 3:
        assert out['offset'] == pytest.approx(TRUE_S0, abs=0.006) and out['gain'] == pytest.approx(TRUE_G, rel=0.06)
