"""tools/check_episode.py on episodes written by the real EpisodeWriter."""
import os, sys, time
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'tools'))
sys.path.insert(0, os.path.join(REPO, 'f1tenth_gym_ros'))
import check_episode as ce                       # noqa: E402
from episode_writer import EpisodeWriter         # noqa: E402

T0 = time.mktime((2026, 10, 14, 9, 15, 0, 0, 0, -1))


def record(root, n=30, odom=True, t0=T0, label='good'):
    w = EpisodeWriter(str(root), {'wheelbase': 0.324})
    ok, eid = w.start('drive to the end of the hallway and stop')
    assert ok
    for i in range(n):
        t = t0 + 0.1 * i
        w.update('scan', np.full(720, 2.0, np.float32), t - 0.03)
        if odom:
            w.update('pose', (0.1 * i, 0.0, 0.0))
            w.update('vel', (1.0, 0.0))
            w.stamp('odom', t - 0.01)
        w.update('imu', (0.0, 0.0, 9.8, 0.0, 0.0, 0.0), t - 0.004)
        w.update('volts', 15.7)
        w.set_action(1.0, 0.05, 0, t - 0.05)
        w.add_frame(np.zeros((36, 64, 3), np.uint8), t)
    w.stop(label)
    return os.path.join(str(root), eid)


def test_healthy_episode_passes(tmp_path):
    d = record(tmp_path)
    lines, problems, mins, label = ce.check(d)
    assert label == 'good'
    jpegs = len(os.listdir(os.path.join(d, 'frames')))      # 0 where OpenCV is missing
    assert problems == ([] if jpegs else ['30 frames in sync.npz but 0 JPEGs'])
    assert abs(mins - 2.9 / 60) < 1e-6
    text = '\n'.join(lines)
    assert 'odometry' in text and 'human 30, policy 0' in text and '2026-10-14' in text


def test_missing_odometry_is_a_problem(tmp_path, capsys):
    d = record(tmp_path, odom=False)
    _, problems, _, _ = ce.check(d)
    assert 'odometry missing' in problems
    assert ce.main([d]) == 1
    assert 'MISSING' in capsys.readouterr().out


def test_unset_clock_is_reported_but_does_not_fail(tmp_path, capsys):
    d = record(tmp_path, t0=time.mktime((2026, 10, 5, 12, 36, 0, 0, 0, -1)))
    _, problems, _, _ = ce.check(d)
    assert any(p.startswith('car clock not set') for p in problems)
    assert ce.main([str(tmp_path)]) == (0 if len(problems) == 1 else 1)
    assert 'Minutes recorded: good 0.0' in capsys.readouterr().out


def test_root_with_no_episodes(tmp_path, capsys):
    assert ce.main([str(tmp_path)]) == 1
    assert 'no episodes' in capsys.readouterr().out
