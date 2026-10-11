#!/bin/bash
# Calibration sweep, unattended: for each target speed, back up to the start spot
# (return_go.sh), then run auto_calibrate at that speed (calib_go.sh --target). Stops at the
# first return or run that does not finish cleanly; backs up to the start at the end.
# Needs ~/calib/start_ref.json (return_start.py --save at the start spot).
#   ~/calib/sweep_go.sh 1.1 1.3 1.5          log: ~/calib/sweep_<time>.log
# The pilot page's calibration panel runs this too (and shows its output).
cd "$(dirname "$0")" || exit 1
S=$(date +%Y%m%d_%H%M%S); LOG="$HOME/calib/sweep_$S.log"
say() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$LOG"; }
quiet() { grep --line-buffered -v -i "warning"; }
back() {
  ./return_go.sh --delay 2 2>&1 | tee -a "$LOG" | quiet; rc=${PIPESTATUS[0]}
  if [ "$rc" -ne 0 ]; then say "return did not finish cleanly (exit $rc); stopping here"; exit 1; fi
}
trap 'say "stopped"; exit 130' INT TERM
n=0
for t in "$@"; do
  n=$((n + 1))
  say "back to the start"; back
  sleep 2
  out="$HOME/calib/run_sweep_${S}_${n}_$t.log"
  say "run $n at $t m/s -> $out"
  ./calib_go.sh --target "$t" 2>&1 | tee "$out" | quiet
  if ! grep -q "STEERING (" "$out" || grep -qE "stopped early|never cleared|budget used up|line the car up|BACKWARD|interrupted|Traceback" "$out"; then
    say "run $n at $t m/s did not finish cleanly; stopping here"; exit 1
  fi
  say "$(grep -E 'steering_angle_to_servo_(offset|gain)|speed_to_erpm_gain about|done:' "$out" | sed 's/^ *//' | paste -sd ';')"
done
say "back to the start"; back
say "sweep done"
