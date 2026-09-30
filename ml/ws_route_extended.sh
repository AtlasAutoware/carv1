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
for s in ${SEEDS:-0 1 2 3 4}; do
  echo "=== $(date) seed $s"; SEEDS=$s bash ml/run_route.sh seed
done
echo "=== $(date) select"; bash ml/run_route.sh select
echo "=== $(date) ALL_DONE"
