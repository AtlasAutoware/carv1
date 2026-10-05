#!/usr/bin/env python3
"""policy_io: the ONE definition of the student policy's inputs and outputs (ROS-free).

Training (ml/train_student.py, ml/train_policy.py), closed-loop evaluation in the sim
(ml/sim_rollout.py) and the car (policy_bridge.py) all import from here, so the three can
no longer drift apart. They had: the dataset stores actions as (speed, steer) while
policy_bridge unpacked the network output as (steer, speed), which on the car turned the
predicted speed (~1.4 m/s) into a steering command (clamped to full left lock) and the
predicted steering angle (~0 rad) into the speed. See docs/ml_audit_2026-09-23.md.
"""
import hashlib, math
import numpy as np

FRONT_HW = (96, 128)          # front camera, resized (H, W), BGR uint8
BEV_HW = (96, 96)             # lidar bird's-eye raster
BEV_EXTENT = 6.0              # metres from the car to the raster edge
MAX_TOK = 24
TXT_VOCAB = 4096
# Order of the 2-vector the network predicts. Every dataset in this repo was written by
# EpisodeWriter.set_action(speed, steer) -> act = (speed, steer); the student learns that.
ACTION_ORDER = ('speed', 'steer')
IDX_SPEED, IDX_STEER = 0, 1


def text_ids(s, n=TXT_VOCAB, max_tok=MAX_TOK):
    """Hashed bag-of-words token ids (no downloads, deterministic across machines)."""
    toks = ''.join(c if c.isalnum() else ' ' for c in s.lower()).split()[:max_tok]
    ids = [1 + int(hashlib.md5(t.encode()).hexdigest(), 16) % (n - 1) for t in toks]
    return ids + [0] * (max_tok - len(ids))


def bev_image(ranges, angle_min, angle_inc, size=BEV_HW[0], extent=BEV_EXTENT, dx=0.0):
    """Rasterise a scan to a size x size image: car at the centre, forward = up, left = left.
    dx moves the returns forward by dx metres before rasterising, i.e. it centres the raster
    dx behind the scanner (see sensor_geom: a model trained with the lidar at the rear axle needs
    the car's scan shifted by the laser's 0.27 m offset)."""
    img = np.zeros((size, size), np.uint8)
    r = np.asarray(ranges, np.float32); n = len(r)
    ang = angle_min + angle_inc * np.arange(n)
    ok = np.isfinite(r) & (r > 0.05) & (r < extent)
    x, y = r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])
    if dx: x = x + np.float32(dx)
    px = (size / 2 - x / extent * size / 2).astype(int)
    py = (size / 2 - y / extent * size / 2).astype(int)
    m = (px >= 0) & (px < size) & (py >= 0) & (py < size)
    img[px[m], py[m]] = 255
    img[size // 2 - 1:size // 2 + 2, size // 2 - 1:size // 2 + 2] = 128
    return img


def front_image(bgr):
    """Full-size BGR frame -> the (96, 128, 3) uint8 the network sees."""
    import cv2
    return cv2.resize(bgr, (FRONT_HW[1], FRONT_HW[0]), interpolation=cv2.INTER_AREA)


def make_feed(front_bgr_small, bev, state, ids):
    """Numpy inputs for the ONNX session (batch of 1)."""
    return {'front': (front_bgr_small.transpose(2, 0, 1)[None] / 255.0).astype(np.float32),
            'bev': (bev[None, None] / 255.0).astype(np.float32),
            'state': np.asarray(state, np.float32).reshape(1, 5),
            'ids': np.asarray(ids, np.int64).reshape(1, MAX_TOK)}


def action_order_of(session):
    """Read the output order from ONNX metadata; models exported before 2026-09-23 carry
    none, and every one of them was trained on (speed, steer)."""
    try:
        meta = session.get_modelmeta().custom_metadata_map or {}
    except Exception:
        meta = {}
    order = tuple(s.strip() for s in meta.get('action_order', 'speed,steer').split(','))
    if sorted(order) != ['speed', 'steer']:
        raise ValueError(f'bad action_order metadata {order!r}')
    return order


def model_config(session):
    """The training settings ml/train_policy.py writes into the ONNX metadata ({} if none)."""
    import json
    try:
        meta = session.get_modelmeta().custom_metadata_map or {}
        return json.loads(meta.get('config', '{}')) or {}
    except Exception:
        return {}


# Where the sensors were when the model's training data was recorded, metres ahead of the rear
# axle (base_link). Models trained before 10/5 saw the sim's lidar and camera AT the rear axle;
# on the car the laser is 0.27 m and the camera 0.30 m ahead of it (bringup static TFs).
def sensor_geom(cfg):
    g = cfg.get('sensor_geom') or {}
    return float(g.get('lidar_x', 0.0)), float(g.get('cam_x', 0.0))


# Motor speed on the car. ackermann_to_vesc runs control_mode 'erpm' (f1tenth_stack vesc.yaml):
# throttle = speed / max_speed, |throttle| < deadband -> 0 eRPM, else eRPM = min_erpm +
# |throttle| * (max_erpm - min_erpm); the wheels then roll at eRPM / speed_to_erpm_gain. So a
# /drive speed is NOT the speed the car drives: 0.3 -> 0.80 m/s, 1.0 -> 1.03 m/s, and nothing
# between 0 and 0.78 m/s exists.
ERPM_MODE = {'max_speed': 5.0, 'min_erpm': 3000.0, 'max_erpm': 10000.0, 'deadband': 0.05, 'erpm_gain': 4285.0}


def erpm_wheel_speed(cmd, p=ERPM_MODE):
    """Wheel speed (m/s) erpm mode makes of a /drive speed."""
    thr = max(-1.0, min(1.0, cmd / p['max_speed']))
    if abs(thr) < p['deadband']: return 0.0
    mag = p['min_erpm'] + abs(thr) * (p['max_erpm'] - p['min_erpm'])
    return math.copysign(mag, thr) / p['erpm_gain']


def erpm_floor(p=ERPM_MODE):
    """Slowest wheel speed erpm mode can hold (throttle at the deadband edge), ~0.78 m/s."""
    return (p['min_erpm'] + p['deadband'] * (p['max_erpm'] - p['min_erpm'])) / p['erpm_gain']


def erpm_command(v, coast_below=0.2, p=ERPM_MODE):
    """/drive speed that makes erpm mode roll at v m/s (forward), the inverse of
    erpm_wheel_speed: asked speeds under the floor get the floor, under coast_below zero."""
    if not v >= coast_below: return 0.0                     # also NaN
    thr = (max(v, erpm_floor(p)) * p['erpm_gain'] - p['min_erpm']) / (p['max_erpm'] - p['min_erpm'])
    return min(max(thr, p['deadband'] + 1e-9), 1.0) * p['max_speed']


def split_action(vec, order=ACTION_ORDER):
    """Network output -> (speed, steer) regardless of the model's output order."""
    d = dict(zip(order, (float(vec[0]), float(vec[1]))))
    return d['speed'], d['steer']


def front_clear(ranges, angle_min, angle_inc, half_width_rad=0.2):
    """Nearest valid return inside +/- half_width of straight ahead (bearing 0), whatever the
    scan's angle convention (-pi..pi or 0..2pi)."""
    r = np.asarray(ranges, np.float32)
    ang = angle_min + angle_inc * np.arange(len(r))
    ang = (ang + np.pi) % (2 * np.pi) - np.pi
    m = (np.abs(ang) <= half_width_rad) & np.isfinite(r) & (r > 0.05)
    return float(r[m].min()) if m.any() else 99.0


# Route hint (added 9/25). The goal-conditioned task was not solvable from the original
# inputs: the policy sees the scene and a bag-of-words route description, never where the
# goal is, so it cannot know where to turn or stop (5-seed success plateaued near 8-11%).
# The fix is the standard one for learned local driving: give the network a point on the
# planned route, in the car frame. It rides in state[2:4], which on the car carried IMU
# roll/pitch rates (near zero on a flat floor and masked out of every earlier model), so
# the 5-wide ONNX interface does not change and old models are unaffected.
HINT_LOOKAHEAD = 2.0          # metres along the route ahead of the car
HINT_SCALE = 2.0              # hint = car-frame point / HINT_SCALE, clipped to +-1.5
IDX_HINT = (2, 3)


def route_hint(pose, path, lookahead=HINT_LOOKAHEAD, scale=HINT_SCALE):
    """pose (x, y, theta) in the map frame; path = list of (x, y) from start to goal.
    Returns the point `lookahead` metres along the path past the nearest path point,
    in the car frame (x forward, y left), divided by `scale`. Near the goal the point
    is the goal itself, so the hint shrinks toward (0, 0) as the car arrives."""
    import math
    P = np.asarray(path, np.float64)
    x, y, th = pose
    i = int(np.argmin(np.hypot(P[:, 0] - x, P[:, 1] - y)))
    seg = np.hypot(*np.diff(P[i:], axis=0).T) if len(P) - i > 1 else np.zeros(0)
    acc = 0.0
    tx, ty = P[-1]
    for k, L in enumerate(seg):
        if acc + L >= lookahead:
            f = (lookahead - acc) / max(L, 1e-9)
            tx, ty = P[i + k] + f * (P[i + k + 1] - P[i + k])
            break
        acc += L
    dx, dy = tx - x, ty - y
    lx = math.cos(-th) * dx - math.sin(-th) * dy
    ly = math.sin(-th) * dx + math.cos(-th) * dy
    return float(np.clip(lx / scale, -1.5, 1.5)), float(np.clip(ly / scale, -1.5, 1.5))
