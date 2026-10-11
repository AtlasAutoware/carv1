#!/bin/bash
# Run auto_calibrate with web_pilot paused: web_pilot streams neutral /teleop commands at
# ~17 Hz, and /teleop outranks /drive in ackermann_mux, so nothing on /drive gets through.
# Started from the pilot page's calibration panel (CALIB_FROM_PILOT=1), web_pilot holds off
# that stream itself and must keep running to show progress, so it is not paused.
source /opt/ros/jazzy/setup.bash
source ~/f1tenth_ws/install/setup.bash
WP=$(pgrep -f "lib/f1tenth_gym_ros/web_pilot" | head -1)
[ -n "$CALIB_FROM_PILOT" ] && WP=""
resume() { [ -n "$WP" ] && kill -CONT "$WP" && echo "web_pilot $WP resumed"; }
trap resume EXIT
if [ -n "$WP" ]; then kill -STOP "$WP"; echo "web_pilot $WP paused"; fi
cd "$(dirname "$0")" && python3 -u auto_calibrate.py --go --delay 3 "$@"
