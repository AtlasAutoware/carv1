#!/usr/bin/env python3
"""check_episode: is a recorded car episode usable for training (milestone T13)?

    python3 tools/check_episode.py ~/episodes/ep_20261014_091502    # one episode
    python3 tools/check_episode.py ~/episodes                         # every episode under a root

For each episode: instruction, label, length, frame rate, battery, the date the car's clock
put on it, and for every stream the logger copies into each frame (lidar, odometry, IMU,
drive command) how many frames had it at all and how old it was at the frame. A stream that
never arrived is the usual failure: the logger listened on a topic nothing publishes, and the
episode quietly holds zeros instead (pose 0, 0, 0). Ends with the total recorded minutes per
label, since T13 asks for 30 minutes of driving.

Exit status 1 if any episode has a missing stream, no frames or missing JPEGs (an unset clock
is reported but does not fail it). Numpy only, so it runs on the Jetson as well as the laptop.
"""
import argparse, json, os, sys, time
import numpy as np

STREAMS = (('scan', 'lidar'), ('odom', 'odometry'), ('imu', 'IMU'), ('act', 'drive command'))
STALE = {'scan': 0.3, 'odom': 0.3, 'imu': 0.1, 'act': 0.5}   # s; older than this at a frame = stale
EARLIEST = time.mktime((2026, 10, 11, 0, 0, 0, 0, 0, -1))   # stamps before this mean the car's
                                                            # clock was not set (no RTC battery)


def check(ep_dir):
    """Return (lines, problems, minutes, label) for one episode directory."""
    meta = json.load(open(os.path.join(ep_dir, 'meta.json')))
    name = meta.get('episode_id', os.path.basename(ep_dir))
    label = meta.get('label', '?')
    lines = ['%s  "%s"  label=%s' % (name, meta.get('instruction', ''), label)]
    problems = []
    sp = os.path.join(ep_dir, 'sync.npz')
    if not os.path.isfile(sp):
        return lines + ['  no sync.npz'], ['no sync.npz'], 0.0, label
    s = np.load(sp)
    t = np.asarray(s['t'], np.float64)
    n = len(t)
    if n == 0:
        return lines + ['  no frames'], ['no frames'], 0.0, label
    dur = float(t[-1] - t[0]) if n > 1 else 0.0
    fdir = os.path.join(ep_dir, 'frames')
    on_disk = len([f for f in os.listdir(fdir) if f.endswith('.jpg')]) if os.path.isdir(fdir) else 0
    volts = np.asarray(s['volts'], np.float64) if 'volts' in s else np.zeros(n)
    v = volts[volts > 0]
    lines.append('  %d frames, %.1f s, %.2f fps, %d JPEGs on disk%s; car clock says %s' % (
        n, dur, (n - 1) / dur if dur > 0 else 0.0, on_disk,
        '; battery %.1f-%.1f V' % (v.min(), v.max()) if v.size else '; no battery reading',
        time.strftime('%Y-%m-%d %H:%M', time.localtime(t[0]))))
    if on_disk != n:
        problems.append('%d frames in sync.npz but %d JPEGs' % (n, on_disk))
    if t[0] < EARLIEST:
        problems.append("car clock not set (stamps say %s)" % time.strftime('%Y-%m-%d', time.localtime(t[0])))
    for key, nice in STREAMS:
        st = np.asarray(s[key + '_t'], np.float64) if key + '_t' in s.files else np.zeros(n)
        have = st > 0
        k = int(have.sum())
        if k == 0:
            lines.append('  %-14s    0/%d frames: MISSING (nothing arrived on its topic)' % (nice, n))
            problems.append(nice + ' missing')
            continue
        age = (t - st)[have]
        extra = ''
        if key == 'act':
            src = np.asarray(s['act_src'])[have]
            extra = '; human %d, policy %d' % (int((src == 0).sum()), int((src == 1).sum()))
        if key == 'scan':
            valid = (np.asarray(s['scan'])[have] > 0).mean(1)
            extra = '; median %.0f%% of beams valid' % (100 * np.median(valid))
        lines.append('  %-14s %4d/%d frames, age median %.2f s, 95%% %.2f s, %d older than %.1f s%s' % (
            nice, k, n, np.median(age), np.percentile(age, 95), int((age > STALE[key]).sum()),
            STALE[key], extra))
    return lines, problems, dur / 60.0, label


def episode_dirs(path):
    path = os.path.expanduser(path)
    if os.path.isfile(os.path.join(path, 'meta.json')):
        return [path]
    if not os.path.isdir(path):
        return []
    return [os.path.join(path, d) for d in sorted(os.listdir(path))
            if os.path.isfile(os.path.join(path, d, 'meta.json'))]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('path', nargs='?', default='~/episodes', help='an episode directory or a root of them')
    a = ap.parse_args(argv)
    dirs = episode_dirs(a.path)
    if not dirs:
        print('no episodes under %s' % a.path)
        return 1
    bad, minutes = 0, {}
    for d in dirs:
        try:
            lines, problems, mins, label = check(d)
        except (OSError, ValueError, KeyError) as e:
            lines, problems, mins, label = ['%s: unreadable (%s)' % (os.path.basename(d), e)], ['unreadable'], 0.0, '?'
        print('\n'.join(lines))
        print('  -> ' + ('PROBLEMS: ' + '; '.join(problems) if problems else 'ok'))
        bad += bool([p for p in problems if not p.startswith('car clock')])
        minutes[label] = minutes.get(label, 0.0) + mins
    print('\n%d episodes, %d with problems. Minutes recorded: %s (T13 asks for 30)' % (
        len(dirs), bad, ', '.join('%s %.1f' % kv for kv in sorted(minutes.items()))))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
