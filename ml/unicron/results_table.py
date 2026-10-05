#!/usr/bin/env python3
"""Results table of a finished shard_run route-hint run:  python ml/unicron/results_table.py ROOT"""
import json, os, sys
R = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else '.')
C = json.load(open(f'{R}/shard_state/config.json'))
pct = lambda x: f'{100 * x:5.1f}'
print('seed  select  standard  collide  stop@goal  unseen map   lidar  camera  no-cam   (success %, task seed 1000 unless noted)')
std_all = []
for s in C['seeds']:
    d = f"{R}/{C['runs']}/route_s{s}"; L = lambda n: json.load(open(f'{d}/{n}/summary.json'))
    sel, std, li, ca, nc = L('eval_sel'), L('eval_none'), L('eval_lidar'), L('eval_camera'), L('eval_nocam')
    std_all.append(std['success_rate'])
    print(f"s{s}    {pct(sel['success_rate'])}   {pct(std['success_rate'])}    {pct(std['collision_rate'])}    {pct(std['stopped_at_goal_rate'])}"
          f"      {pct(std['by_map']['uploadtest']['success_rate'])}     {pct(li['success_rate'])}  {pct(ca['success_rate'])}  {pct(nc['success_rate'])}")
print(f'5-seed mean, standard set: {100 * sum(std_all) / len(std_all):.1f}%')
ch = json.load(open(f"{R}/{C['runs']}/route/choice.json"))
print(f"chosen on the selection set (task seed 3000): {ch['chosen']}")
for s in C['seeds']:
    E = [json.loads(l) for l in open(f"{R}/{C['runs']}/route_s{s}/eval_none/episodes.jsonl")]
    bad = [e for e in E if not e['success']]
    if bad:
        print(f"s{s} failures on the standard set: {len(bad)}, of which reached the goal first: {sum(e['reached'] for e in bad)}, "
              f"had stopped at the goal: {sum(e['stopped_at_goal'] for e in bad)}")
