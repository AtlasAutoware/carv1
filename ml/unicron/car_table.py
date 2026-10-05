#!/usr/bin/env python3
"""Results of a car-conditions run (ml/unicron/car_run_1005.json) as a markdown table.

    python3 ml/unicron/car_table.py <root> [--runs runs_car]

One row per model (trained variants and the external old_s* models), success % on every eval
(collisions % in brackets where they are not simply 100 - success), the selection picks marked.
"""
import argparse, glob, json, os

EVALS = ['car_sel', 'car_std', 'car_today', 'car_fixed', 'car_cam', 'car_nocam', 'car_instr', 'car_stress', 'car_aeb60']


def load(p):
    try:
        with open(p) as f: return json.load(f)
    except (OSError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('root'); ap.add_argument('--runs', default='runs_car')
    a = ap.parse_args(); runs = os.path.join(a.root, a.runs)
    picks = {}
    for d in ('car', 'final'):
        c = load(os.path.join(runs, d, 'choice.json'))
        if c: picks.setdefault(os.path.basename(c['chosen']), []).append(d)
    dirs = sorted(d for d in glob.glob(os.path.join(runs, '*')) if any(os.path.isdir(os.path.join(d, e)) for e in EVALS))
    print('| model | ' + ' | '.join(EVALS) + ' |')
    print('|---' * (len(EVALS) + 1) + '|')
    for d in dirs:
        name = os.path.basename(d); cells = []
        for e in EVALS:
            s = load(os.path.join(d, e, 'summary.json'))
            if not s: cells.append('-'); continue
            succ, coll = 100 * s['success_rate'], 100 * s['collision_rate']
            cells.append(f'{succ:.1f}' + (f' ({coll:.1f})' if abs(succ + coll - 100) > 0.05 else ''))
        mark = f" **{'+'.join(picks[name])}**" if name in picks else ''
        print(f'| {name}{mark} | ' + ' | '.join(cells) + ' |')
    print('\nsuccess % (collision % in brackets when success + collisions != 100: the rest timed out or stopped short)')
    for d in ('car', 'final'):
        c = load(os.path.join(runs, d, 'choice.json'))
        if c: print(f"{d}: {c['chosen']} (picked on {c.get('selection_eval', 'eval_sel')})")


if __name__ == '__main__':
    main()
