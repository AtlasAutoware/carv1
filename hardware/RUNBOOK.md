# RoboRacer car runbook (updated 2026-09-02)

## Bring up everything
    ros2 launch f1tenth_stack bringup_launch.py
Starts: joy, joy_teleop, ackermann_to_vesc, vesc_to_odom, vesc_driver, ackermann_mux,
static base_link->laser TF, and the RPLIDAR C1 (rplidar_node). No separate lidar launch needed.

Quick health check (second shell):
    ros2 topic hz /scan            # ~10 Hz  (RPLIDAR C1, 460800 baud, /dev/sensors/rplidar)
    ros2 topic hz /sensors/core    # ~50 Hz  (VESC FW 6.6, /dev/sensors/vesc)
    ros2 topic echo /sensors/core --once | grep -E "voltage_input|fault_code"
    ros2 topic hz /odom            # only after the first joystick/servo command

## Hardware facts
- Lidar: RPLIDAR C1 (S/N 9B2F...), NOT an A2. A2/A3/S1 launches time out on it.
- VESC: Flipsky FSESC 6.7 Pro, HW 60, FW 6.06. udev: 0483:5740 -> /dev/sensors/vesc.
  VESC only enumerates on USB when the battery is connected.
- Motor: Castle 1415 2400Kv sensored, 4-pole (2 pole pairs). FDR 10.2, tire 0.109 m.
- Servo: Hitec D625MW.

## vesc.yaml (src/f1tenth_system/f1tenth_stack/config/vesc.yaml)
- speed_to_erpm_gain = 3575 (was 4614, the reference-car value). Used by odometry and by
  control_mode "speed"; NOT used by "erpm" mode. Verify: drive a measured 5 m, compare /odom.
- control_mode "erpm": stick fraction mapped between min_erpm 3000 and max_erpm 10000
  (about 0.8 to 2.8 m/s with this gearing). Written to avoid the sensorless stall/smoke.
  With hall sensors that stall zone is gone, so "speed" mode (true m/s closed loop, what
  autonomy nodes expect) is viable again - bench test on a stand before switching.
- STILL TO CALIBRATE with the D625MW: steering_angle_to_servo_offset (straight-ahead value),
  servo_min / servo_max (lock-to-lock), steering_angle_to_servo_gain. Current values are
  reference-car defaults.
- wheelbase (vesc_to_odom_node) is 0.25 by default: measure axle-to-axle on this chassis.
- static TF base_link->laser is x=0.27 z=0.11: measure the C1's real position.

## Config edits need a rebuild (install is a copy, not a symlink)
    cd ~/f1tenth_ws && colcon build --packages-select f1tenth_stack

## Backups
    config/vesc.yaml.bak, launch/bringup_launch.py.bak (pre-2026-09-02 versions)

## VESC gotchas learned the hard way (2026-09-02)
- The PPM header and the servo output are the SAME pin. "Enable Servo Output" (App Settings >
  General) flips it from RC input to servo output. A fresh Flipsky board ships with servo output
  OFF and App = PPM+UART, so the servo wire gets decoded as an RC input and the PPM app zeroes
  every USB command: motor commands do nothing, no fault code, R/L detection returns 0/0.
  Fix = Enable Servo Output + App to Use = UART (or No App), then REBOOT the VESC.
- After any VESC config change that touches the servo/PPM pin, reboot the VESC before testing.
- Limits set on this board: Motor 60/-60 A, Abs 120 A, Slow ABS Current Limit ON, Battery 99/-60 A.
- Config backup: ~/f1tenth_ws/vesc_backup/{mcconf,appconf}_good_2026-09-02.bin plus the python
  tools used to read/write them over USB without VESC Tool (vesc_mcconf3.py decodes the motor
  config; vesc_fix_app.py / vesc_fix_limits.py show how to write). Also save an XML from VESC Tool.
- Pad: Logitech F310, switch on X. LB = deadman, left stick = throttle, right stick = steer.
  Profile auto-selected (joy_teleop_f310.yaml). RB = autonomy deadman.
- Speed cap in erpm mode: max_erpm 10000 = ~2.8 m/s with this gearing. Raise max_erpm in vesc.yaml
  for faster laps (20000 = ~5.6 m/s), rebuild, test on a stand first.

## Camera perception (2026-09-02)
- YOLOv8n car detector runs directly through TensorRT/CUDA at 640 pixels. The
  ONNX file is used once to compile the device-specific engine; live inference
  does not load ONNX Runtime.
- First install/build: `cd ~/atlas_ws/src/atlasautoware && hardware/scripts/install_tensorrt.sh`.
  Re-run `hardware/scripts/build_tensorrt_engine.sh` after a model or JetPack update.
- Engine: `~/.cache/atlasautoware/car_yolov8_640.engine` (never copy an x86 or another Jetson's engine).
- Rebuild the ROS package after pulling the TensorRT code:
  `cd ~/atlas_ws && colcon build --packages-select f1tenth_gym_ros && source install/setup.bash`.
- Enable with `use_perception:=true`; feeds `/camera_opponents_poses` to `race_agent` only.
- Standalone check: `python3 tools/benchmark_camera_perception.py ~/.cache/atlasautoware/car_yolov8_640.engine`.
- Safe ROS check (no drive node and no autonomous controller):
  `ros2 launch f1tenth_gym_ros car_bringup_launch.py use_drive:=false use_racing:=false use_lidar:=false use_perception:=true`.
  The log must say `backend: tensorrt`; verify `/oakd/rgb` and `/camera_opponents_poses`
  are publishing. Run `tegrastats` in another shell to observe GPU activity.

## Remote pilot mode (2026-09-02)
- The car is its own hotspot: SSID AtlasCar (NetworkManager connection, autoconnect), car = 10.42.0.1.
  Since 2026-10-05 it runs on the USB TP-Link T3U (2.4 GHz ch 6, WPA3); see "Car link on two T3Us" below.
  Internet on the Jetson at the same time: the onboard card is free, carnet.sh client/home/tether.
- On the car: ~/run_remote.sh (or ~/restart_remote.sh to bounce it). Then open http://10.42.0.1:8080/
  (http://192.168.55.1:8080/ over USB-C): FPV stream, W/S throttle, A/D steer, max-throttle slider,
  gamepad via the browser (hold LB), lidar plot. Release everything = neutral; 250 ms watchdog on the car.
- Only the tab that is driving sends commands; other tabs are viewers. Python client: tools/remote_pilot.py (UDP 5005).
- Never pkill by a pattern that appears in your own command line (it kills the shell); use the scripts.

## Range, trim, robustness (2026-09-02, later)
- Steering trim: Q/E on the web page (stored on the car in ~/.atlascar_trim.json); for autonomy apply the equivalent to vesc.yaml steering_angle_to_servo_offset (see docs/REMOTE.md).
- Networks: hardware/scripts/carnet.sh  hotspot | hotspot5 | client <SSID> [pw] | home | tether | status.
- Anywhere/cellular: install_tailscale.sh once, then http://atlascar:8080/ with video=low and PILOT_TIMEOUT=0.6 run_remote.sh lowbw.
- vesc_driver/ackermann_to_vesc/joy respawn after a VESC USB blip (they used to die with std::system_error).

## Self-driving + track pictures from the web UI (2026-09-02)
- Panel on the pilot page: pick a raceline, speed cap, ENGAGE. Space/Esc or STOP stops it; closing the tab stops it (2 s heartbeat).
- Autonomy publishes /drive (mux priority 10); holding a key or LB (teleop, priority 100) always overrides.
- joy_teleop_f310.yaml lost its deadman-less 'default' block: it streamed zero teleop that masked autonomy in the mux. web_pilot now publishes that brake-to-zero itself, and suppresses it while engaged.
- Track picture -> map -> raceline in the same panel (see docs/REMOTE.md). Scale comes from the lane width you type in.

## Car link on two TP-Link T3Us (2026-10-05)
- Car: TP-Link Archer T3U (RTL8812BU) on a Jetson USB port hosts AtlasCar: 2.4 GHz ch 6, 20 MHz, WPA3-SAE with
  PMF required, AES-CCMP, US rules. NVIDIA's kernel has no rtw88: hardware/scripts/install_t3u_driver.sh builds
  lwfinger/rtw88 (rerun after a kernel update). The onboard card keeps a manual WPA2 fallback (AtlasCarOnboard,
  5 GHz ch 149), used by `carnet.sh hotspot` when no T3U is plugged in.
- Laptop: a second T3U. Its NetworkManager profile "AtlasCar" is bound to that adapter's MAC, never takes the
  default route or DNS, and is locked to the car T3U's BSSID: the lock stops background scans, which stalled
  the link for ~100 ms every few seconds. "AtlasCar-fallback" (any BSSID/band) covers the onboard fallback.
- Both adapters stay in USB 2 mode (rtw88 switch_usb_mode=n): USB 3 radiates noise into 2.4 GHz.
- Measured on the bench (adapters ~1 m apart): -32/-34 dBm, MCS 15 (144 Mbit/s PHY), iperf3 56 Mbit/s to the
  car and 40 Mbit/s back, ping p50 1.6 ms / p99 16 ms over 1,400 pings, 0% loss. Range not measured yet.
- Details and the reasoning: docs/REMOTE.md, "The T3U pair".
- Pilot video (2026-10-05): the OAK-D encodes H.264 itself (VIDEO_KBPS=1000 by default in run_remote.sh);
  the page decodes it with WebCodecs only on a secure page, so open it with tools/pilot_tunnel.sh
  (http://localhost:8081/). Plain http://10.42.0.1:8080/ still works with MJPEG.
- After a power cut the car's T3U once came back with constant USB errors (status -71) and no link;
  re-seating it fixed it. Check `journalctl -k | grep -c "status: -71"` if the link does not return.
