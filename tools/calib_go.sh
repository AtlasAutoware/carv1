#!/bin/bash
# Run auto_calibrate with web_pilot paused: web_pilot streams neutral /teleop commands at
# ~17 Hz, and /teleop outranks /drive in ackermann_mux, so nothing on /drive gets through.
source /opt/ros/jazzy/setup.bash
source ~/f1tenth_ws/install/setup.bash
WP=$(pgrep -f "lib/f1tenth_gym_ros/web_pilot" | head -1)
resume() { [ -n "$WP" ] && kill -CONT "$WP" && echo "web_pilot $WP resumed"; }
trap resume EXIT
if [ -n "$WP" ]; then kill -STOP "$WP"; echo "web_pilot $WP paused"; fi
cd "$(dirname "$0")" && python3 auto_calibrate.py --go --delay 3 "$@"
