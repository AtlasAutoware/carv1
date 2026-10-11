#!/bin/bash
# Restart only the OAK-D node (oakd_camera) with run_remote.sh's parameters. Use it when the
# pilot page shows no video and /tmp/remote_camera.log says "0.0 fps" (10/9: an XLink error,
# "no request for this response: XLINK_READ_REL_RESP", stopped RGB and H.265 while the IMU
# kept going). The IMU (/oakd/imu) drops out for a few seconds.   Log: /tmp/remote_camera.log
source /opt/ros/jazzy/setup.bash
source "$HOME/f1tenth_ws/install/setup.bash"
source "$HOME/atlas_ws/install/setup.bash"
CFG="$HOME/atlas_ws/src/atlasautoware/config/hardware.yaml"
# [o] keeps this script's own command line from matching
for pid in $(pgrep -f "f1tenth_gym_ros[/ ][o]akd_camera"); do kill "$pid" 2>/dev/null; done
for i in $(seq 1 20); do pgrep -f "f1tenth_gym_ros[/ ][o]akd_camera" >/dev/null || break; sleep 0.5; done
for pid in $(pgrep -f "f1tenth_gym_ros[/ ][o]akd_camera"); do kill -9 "$pid" 2>/dev/null; done
sleep 1
setsid nohup ros2 run f1tenth_gym_ros oakd_camera --ros-args --params-file "$CFG" \
    -p video_kbps:=${VIDEO_KBPS:-1000} -p video_codec:=${VIDEO_CODEC:-h265} > /tmp/remote_camera.log 2>&1 < /dev/null &
echo "oakd_camera restarting (log /tmp/remote_camera.log)"
