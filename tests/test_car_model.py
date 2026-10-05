"""
ml/car_model.py and the car-side speed/scan corrections in policy_io.
=====================================================================

Pins the numbers the car-conditions sim rests on: the erpm-mode speed mapping against what
auto_calibrate logged on the car (10/5: /drive 0.3 -> 0.78-0.82 m/s), the motor and servo
models, the body outline, the scan shift for models trained with the lidar at the rear axle,
and that sim_rollout's plain mode is untouched by all of it.

    python3 -m pytest tests/test_car_model.py -q
"""
import math, os, sys

import numpy as np
import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'ml')); sys.path.insert(0, os.path.join(REPO, 'f1tenth_gym_ros'))
import car_model as CM            # noqa: E402
import policy_io as PIO           # noqa: E402


def test_erpm_mapping_matches_the_car():
    assert PIO.erpm_wheel_speed(0.3) == pytest.approx(0.798, abs=1e-3)      # logged 0.78-0.82 m/s at --speed 0.3
    assert PIO.erpm_wheel_speed(1.0) == pytest.approx(1.027, abs=1e-3)
    assert PIO.erpm_wheel_speed(0.24) == 0.0                                 # inside the 0.05 throttle deadband
    assert PIO.erpm_wheel_speed(5.0) == pytest.approx(10000 / 4285)          # full throttle
    assert PIO.erpm_floor() == pytest.approx(3350 / 4285)


@pytest.mark.parametrize('v', [0.79, 0.8, 0.9, 1.0, 1.5, 2.0, 2.3])
def test_erpm_command_inverts_the_mapping(v):
    assert PIO.erpm_wheel_speed(PIO.erpm_command(v)) == pytest.approx(v, abs=1e-6)


def test_erpm_command_floor_and_coast():
    assert PIO.erpm_wheel_speed(PIO.erpm_command(0.5)) == pytest.approx(PIO.erpm_floor(), abs=1e-6)
    assert PIO.erpm_command(0.19) == 0.0 and PIO.erpm_command(0.0) == 0.0 and PIO.erpm_command(float('nan')) == 0.0
    assert PIO.erpm_command(9.0) == 5.0                                      # saturates at full throttle


def test_wheel_target_per_speed_map():
    raw, inv, lin = (CM.resolve('car', [f'speed_map="{m}"']) for m in ('raw', 'inverse', 'linear'))
    assert CM.wheel_target(1.0, raw) == pytest.approx(1.027, abs=1e-3)
    assert CM.wheel_target(1.0, inv) == pytest.approx(1.0, abs=1e-6)
    assert CM.wheel_target(0.4, inv) == pytest.approx(PIO.erpm_floor(), abs=1e-6)
    assert CM.wheel_target(0.4, lin) == 0.4


def test_motor_matches_the_logged_ramp():
    """auto_calibrate 10/5 11:39, /drive 0.3: 0.32 m/s at 0.21 s, 0.52 at 0.31 s, 0.69 at 0.41 s."""
    c = CM.resolve('car'); m = CM.Motor(c, 0.02); v = 0.0; tgt = CM.wheel_target(0.3, c); out = []
    for _ in range(40): v = m.step(v, tgt); out.append(v)
    at = lambda t: out[int(round(t / 0.02)) - 1]
    assert at(0.06) == 0.0                                                   # dead time
    assert at(0.20) == pytest.approx(0.30, abs=0.06) and at(0.30) == pytest.approx(0.50, abs=0.06)
    assert at(0.40) == pytest.approx(0.69, abs=0.06) and at(0.80) == pytest.approx(tgt)
    for _ in range(60): v = m.step(v, 0.0)
    assert v == 0.0                                                          # stops, never reverses


def test_servo_lock_and_slew():
    c = CM.resolve('car'); s = CM.Servo(c, 0.02); a = 0.0; out = []
    for _ in range(10): a = s.step(a, -0.4); out.append(a)
    assert out[0] == 0.0 and out[1] == pytest.approx(-0.06)                  # 20 ms dead time, 3 rad/s
    assert min(out) == pytest.approx(-0.25)                                  # right lock until the horn is re-centred
    f = CM.resolve('car_fixed'); s = CM.Servo(f, 0.02); a = 0.0
    for _ in range(20): a = s.step(a, -0.4)
    assert a == pytest.approx(-0.4)


def test_body_outline_and_hits():
    assert CM.FOOTPRINT[:, 0].min() == pytest.approx(-CM.REAR) and CM.FOOTPRINT[:, 0].max() == pytest.approx(CM.FRONT)
    assert np.abs(CM.FOOTPRINT[:, 1]).max() == pytest.approx(CM.HALF_W)

    class M:                                       # 10 x 10 m, 5 cm cells, one wall at x = 5 m
        res, ox, oy, H, W = 0.05, 0.0, 0.0, 200, 200
        occ = np.zeros((200, 200), bool); occ[:, 100] = True
    st = lambda x, th: np.array([x, 5.0, th, 0.0])
    assert not CM.body_hits(M, st(4.50, 0.0))      # bumper 4.95: clear
    assert CM.body_hits(M, st(4.56, 0.0))          # bumper 5.01: in the wall
    assert not CM.body_hits(M, st(5.20, 0.0))      # past it, facing away: rear bumper at 5.08
    assert CM.body_hits(M, st(5.15, 0.0))          # rear bumper at 5.03, in the wall's cells (5.00-5.05)
    assert CM.body_hits(M, st(5.20, math.pi))      # turned around: the body spans 4.75-5.32 across the wall
    assert CM.body_hits(M, st(0.05, 0.0))          # rear sticks out of the map


def test_bev_dx_moves_returns_forward():
    r = np.full(4, 2.0, np.float32); ang0, inc = 0.0, math.pi / 2                # returns at +x, +y, -x, -y
    a, b = PIO.bev_image(r, ang0, inc), PIO.bev_image(r, ang0, inc, dx=0.5)
    S, E = PIO.BEV_HW[0], PIO.BEV_EXTENT
    row = lambda x: int(S / 2 - x / E * S / 2)
    assert a[row(2.0), S // 2] == 255 and b[row(2.5), S // 2] == 255 and b[row(2.0), S // 2] == 0
    assert np.array_equal(PIO.bev_image(r, ang0, inc, dx=0.0), a)


def test_history_and_delay():
    h = CM.History(0.2)
    for k in range(20): h.add(k * 0.02, np.array([k * 1.0, 0, 0, 0]))
    assert h.at(0.38)[0] == 19 and h.at(0.30)[0] == 15 and h.at(-1.0)[0] == h.buf[0][1][0]
    d = CM.Delay(0.06, 0.02); assert [d(x) for x in (1, 2, 3, 4)] == [0.0, 0.0, 0.0, 1]
    assert CM.Delay(0.0, 0.02)(7) == 7


def test_presets_and_bad_settings():
    for p in CM.PRESETS: CM.resolve(p)
    assert CM.resolve('car_bridge')['speed_map'] == 'inverse' and CM.resolve('car')['speed_map'] == 'raw'
    assert CM.resolve('car', ['cam=perturb'])['cam'] == 'perturb'             # bare strings are fine too
    with pytest.raises(SystemExit): CM.resolve('nope')
    with pytest.raises(SystemExit): CM.resolve('car', ['speed=1'])
    with pytest.raises(SystemExit): CM.resolve('car', ['speed_map="fast"'])


def test_car_rollout_runs_and_plain_rollout_is_unchanged():
    pytest.importorskip('cv2'); pytest.importorskip('scipy')
    import sim_rollout as SR
    tasks = SR.sample_tasks(['my_track'], 1, 4242)
    plain, d0 = SR.rollout(tasks[0], None, record=True, seed=5)
    again, d1 = SR.rollout(tasks[0], None, record=True, seed=5)
    assert plain == again and all(np.array_equal(d0[k], d1[k]) for k in d0)      # deterministic
    assert 'geom' not in d0                                                       # plain data: sensors at the axle
    res, data = SR.rollout_car(tasks[0], None, CM.resolve('car_train'), record=True, seed=5)
    assert res['reached'] and data['geom'].tolist() == pytest.approx([0.27, 0.30])
    assert res['max_speed'] <= 1.0 + 1e-9                                          # the bridge's max_speed
    assert set(data) == set(d0) | {'geom'} and len(data['act']) > 10


def test_hint_replans_like_the_bridge():
    pytest.importorskip('scipy')
    import sim_rollout as SR
    task = SR.sample_tasks(['levine'], 1, 4242)[0]; path = [tuple(p) for p in task['path']]
    c = CM.resolve('car', ['pose_noise=0', 'yaw_noise=0'])
    assert c['replan'] == 1.0                                         # the bridge's default
    h = CM.Hint(path, c, np.random.default_rng(0), 'levine')
    st = np.array([path[0][0], path[0][1], math.atan2(path[1][1] - path[0][1], path[1][0] - path[0][0]), 0.0])
    hint = h(st, 0.0)
    assert hint is not None and h.path is not None and hint[0] > 0.3          # points ahead along the route
    assert math.hypot(h.path[-1][0] - path[-1][0], h.path[-1][1] - path[-1][1]) < 1e-9   # ends at the goal
    t_plan = h.t_plan; h(st, 0.5); assert h.t_plan == t_plan                   # replans once a second
    h(st, 1.0); assert h.t_plan == 1.0
    g = np.array([path[-1][0] + 0.1, path[-1][1], 0.0, 0.0])
    assert h(g, 2.0) is None                                                  # inside goal_tol: hold
    c0 = CM.resolve('car', ['replan=0', 'pose_noise=0', 'yaw_noise=0'])
    h0 = CM.Hint(path, c0, np.random.default_rng(0))
    assert h0.path == path and h0(st, 0.0) == pytest.approx(PIO.route_hint(st[:3], path))
