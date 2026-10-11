#!/bin/bash
# Restart only episode_logger (the demonstration recorder for milestone T13) with run_remote.sh's
# parameters, on whichever wheel-odometry topic actually has a publisher. run_remote.sh gives it
# /vesc/odom, which only the vesc-uart backend publishes; the f1tenth_stack bringup's vesc_to_odom
# publishes /odom. If the logger listens on the wrong one, every episode holds pose 0,0,0, and
# tools/check_episode.py reports "odometry MISSING". ODOM_TOPIC=/odom forces a topic.
# A recording in progress is closed with the label "interrupted".   Log: /tmp/episode_logger.log
#   ~/restart_logger.sh
source /opt/ros/jazzy/setup.bash
source "$HOME/f1tenth_ws/install/setup.bash"
source "$HOME/atlas_ws/install/setup.bash"
CAMERA="${CAMERA:-oakd}"
if [ "$CAMERA" = "oakd" ]; then CAM_TOPIC=/oakd/rgb; else CAM_TOPIC=/camera/color/image_raw; fi
pubs() { ros2 topic info "$1" 2>/dev/null | sed -n 's/^Publisher count: *//p'; }
if [ -z "$ODOM_TOPIC" ]; then
    for t in /vesc/odom /odom; do
        n=$(pubs "$t"); if [ "${n:-0}" -gt 0 ]; then ODOM_TOPIC=$t; break; fi
    done
fi
if [ -z "$ODOM_TOPIC" ]; then
    echo "nothing publishes /vesc/odom or /odom: is the bringup running? Using /odom."
    ODOM_TOPIC=/odom
fi
# [e] keeps this script's own command line from matching; SIGINT lets the logger close an episode
for pid in $(pgrep -f "f1tenth_gym_ros[/ ][e]pisode_logger"); do kill -INT "$pid" 2>/dev/null; done
for i in $(seq 1 10); do pgrep -f "f1tenth_gym_ros[/ ][e]pisode_logger" >/dev/null || break; sleep 0.5; done
for pid in $(pgrep -f "f1tenth_gym_ros[/ ][e]pisode_logger"); do kill "$pid" 2>/dev/null; done
setsid nohup ros2 run f1tenth_gym_ros episode_logger --ros-args -p root:=${EPISODE_ROOT:-$HOME/episodes} \
    -p image_topic:=$CAM_TOPIC -p odom_topic:=$ODOM_TOPIC -p imu_topic:=/oakd/imu \
    > /tmp/episode_logger.log 2>&1 < /dev/null &
echo "episode_logger restarting with odometry from $ODOM_TOPIC (log /tmp/episode_logger.log)"
