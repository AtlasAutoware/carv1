#!/bin/bash
# Back the car up to the calibration start spot (return_start.py --go) with web_pilot paused:
# web_pilot streams neutral /teleop at ~17 Hz while the page is open, and /teleop outranks
# /drive in ackermann_mux, so nothing on /drive moves the car while it runs. Started from the
# pilot page (CALIB_FROM_PILOT=1), web_pilot holds off that stream itself and is not paused.
source /opt/ros/jazzy/setup.bash
source ~/f1tenth_ws/install/setup.bash
WP=$(pgrep -f "lib/f1tenth_gym_ros/web_pilot" | head -1)
[ -n "$CALIB_FROM_PILOT" ] && WP=""
resume() { [ -n "$WP" ] && kill -CONT "$WP" && echo "web_pilot $WP resumed"; }
trap resume EXIT
if [ -n "$WP" ]; then kill -STOP "$WP"; echo "web_pilot $WP paused"; fi
cd "$(dirname "$0")" || exit 1
python3 -u return_start.py --go "$@"
rc=$?
exit $rc
