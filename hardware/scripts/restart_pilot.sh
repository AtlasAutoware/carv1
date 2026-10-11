#!/bin/bash
# Restart only web_pilot (the pilot page server, :8080) with run_remote.sh's parameters, e.g.
# after editing f1tenth_gym_ros/web_pilot.py (symlink install: no rebuild needed). Everything
# else (bringup, camera, logger) keeps running.  Log: /tmp/web_pilot.log
#   ~/restart_pilot.sh
source /opt/ros/jazzy/setup.bash
source "$HOME/f1tenth_ws/install/setup.bash"
source "$HOME/atlas_ws/install/setup.bash"
export F1TENTH_CONTROLLER=f310
CAMERA="${CAMERA:-oakd}"
if [ "$CAMERA" = "oakd" ]; then CAM_TOPIC=/oakd/rgb; else CAM_TOPIC=/camera/color/image_raw; fi
export ATLAS_IMAGE_TOPIC="$CAM_TOPIC"
# [w] keeps this script's own command line from matching
for pid in $(pgrep -f "f1tenth_gym_ros[/ ][w]eb_pilot"); do kill -CONT "$pid" 2>/dev/null; kill "$pid" 2>/dev/null; done
for i in 1 2 3 4 5 6; do pgrep -f "f1tenth_gym_ros[/ ][w]eb_pilot" >/dev/null || break; sleep 0.5; done
setsid nohup ros2 run f1tenth_gym_ros web_pilot --ros-args -p timeout:=${PILOT_TIMEOUT:-0.25} \
    -p image_topic:=$CAM_TOPIC > /tmp/web_pilot.log 2>&1 < /dev/null &
echo "web_pilot restarting (log /tmp/web_pilot.log)"
