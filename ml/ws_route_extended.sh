#!/usr/bin/env bash
# Extended route-hint run on the Atlas ML workstation (coblenz: RTX A4500, i7-8700, 12 threads).
# vs the 9/25 cluster recipe: 2x the expert demos (2400 episodes) and 50 epochs instead of 25,
# same 5 seeds and the same seed selection (task seed 3000), so the numbers stay comparable.
#   nohup bash ml/ws_route_extended.sh > logs/ws_route_extended.log 2>&1 &
# Resumable: every stage skips work whose outputs already exist.
set -uo pipefail
cd "$(dirname "$0")/.."
export PY=${PY:-/opt/ml/bin/python} WORKERS=${WORKERS:-12} EPOCHS=${EPOCHS:-50} MORE=1 RUNS=runs_ext
mkdir -p logs data $RUNS
echo "=== $(date) prep"; bash ml/run_route.sh prep || exit 1
# Training loads the demos in pieces (train_policy.py build_cache): the 2400 episodes do not fit
# in this machine's 62 GB of RAM all at once. If a seed still fails, stop with exit 0 so the
# systemd unit (Restart=on-failure) does not retry it in a loop; the next boot tries again.
for s in ${SEEDS:-0 1 2 3 4}; do
  echo "=== $(date) seed $s"
  SEEDS=$s bash ml/run_route.sh seed || { echo "=== $(date) seed $s FAILED, stopping (see $RUNS/route_s$s.log)"; exit 0; }
done
echo "=== $(date) select"; bash ml/run_route.sh select
echo "=== $(date) ALL_DONE"
