# carv1 — Atlas Autoware's first car

Everything for Atlas Autoware's first 1/10-scale autonomous race car in one
repo: the ROS 2 racing software, the Jetson setup that runs it, and the CAD
and electronics for the physical build. (The next car lives in
[carv2](https://github.com/AtlasAutoware/carv2).)

This repo combines two earlier ones, with their full commit history:
`atlasautoware` (the software, now the repo root) and `atlasautoware-cad`
(now [`cad/`](cad/)).

## What's where

| Path | What it holds |
|---|---|
| `f1tenth_gym_ros/`, `launch/`, `config/` | The ROS 2 racing stack: nodes, launch files, sim and car parameters |
| `racelines/`, `maps/` | Optimized racelines and track maps |
| `tools/`, `tests/`, `ui/`, `ml/`, `models/` | Tuning and benchmarking scripts, tests, the live dashboard, perception models |
| `hardware/` | What runs on the Jetson outside the ROS package: bringup scripts, udev rules, VESC config, race-day [RUNBOOK](hardware/RUNBOOK.md) |
| `docs/` | Design notes (MPC, racing tech, mapping, hardware wiring) and the project paper/report |
| `cad/Components/` | 3D models of the off-the-shelf parts (Jetson Orin Nano, RPLIDAR C1, PCA9685) |
| `cad/Mounts/` | Scripted (CadQuery) mounts and baseplate, with fit checks, fab exports and G-code — see [DESIGN.md](cad/Mounts/DESIGN.md) |
| `cad/Templates/` | The original clipless mounting system and baseplate the mounts are built on |
| `cad/Electronics/` | AtlasPower-4S, the power board that runs the car's electronics off the 4S drive pack — see its [README](cad/Electronics/README.md) |
| `cad/Jetson/` | JetPack rebuild and smoke-test scripts |

## The software stack

A complete ROS 2 racing stack for 1/10-scale autonomous racing: simulation
bridge, SLAM mapping, raceline optimization, an OSQP-based MPC racing
controller with lidar AEB and an IMU traction governor, camera opponent
detection, and a hardware bringup layer (Jetson + OAK-D Pro + RPLidar + VESC)
so the same nodes that win in sim drive the physical car.

Built on the [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros)
simulation bridge (ROS 2 Humble).

```
                 ┌── sim:  gym_bridge (f1tenth_gym) ──────────────┐
sensors ─────────┤                                                ├──► /scan, odom, /oakd/*
                 └── car:  rplidar_node · oakd_camera · drive_node┘
                                            ▲
map ─► slam_toolbox ─► raceline_optimizer ─► raceline_mpc (MPC + AEB + traction governor) ─► /drive
                                            │
                       camera_perception ───┘ (YOLO opponent detection, optional)
```

## The racing pipeline

1. **Map** the track: `ros2 launch f1tenth_gym_ros slam_mapping_launch.py`
   driving slowly (or `mapping_driver` autonomously) — see
   [docs/mapping.md](docs/mapping.md).
2. **Optimize** a raceline: `raceline_optimizer` / `track_learner` produce
   `racelines/*.csv` (x, y, heading, curvature, speed); inspect with
   `tools/draw_raceline.py`.
3. **Profile** the speeds: `tools/reprofile_raceline.py` (or
   `reprofile_speeds: true` at runtime) replaces the speed column with the
   TUMFTM friction-limited forward-backward profile, so commanded speeds
   provably fit the grip budget — see
   [docs/racing_tech.md](docs/racing_tech.md).
4. **Race**: `raceline_mpc` tracks the line with a kinematic LTV-MPC
   (persistent warm-started OSQP solve, ~0.6 ms/tick), falls back to the
   MAP controller (Becker et al., ICRA 2023 — tire-aware pursuit) on any
   solver hiccup, brakes for obstacles with a speed-aware AEB, and scales
   speed when the IMU says grip is running out — see
   [docs/mpc.md](docs/mpc.md) and [docs/racing_tech.md](docs/racing_tech.md).

Racing nodes: `raceline_mpc` (competition time-trial), `race_agent` /
`racing_agent` (full strategy + opponents), `opponent_driver`,
`camera_perception` ([docs/camera_perception.md](docs/camera_perception.md)).
Tooling: `tools/auto_tune.py`, `tools/benchmark_lap.py`, and a live dashboard
([docs/dashboard.md](docs/dashboard.md)).

## Quickstart — simulation

Supported: Ubuntu 22.04 native with ROS 2 Humble, or any OS via Docker
(NVIDIA GPU with `rocker`, or noVNC without).

**Native (Ubuntu 22.04 + ROS 2 Humble):**
```bash
# dependencies
git clone https://github.com/f1tenth/f1tenth_gym && cd f1tenth_gym && pip3 install -e . && cd ..
# workspace
mkdir -p $HOME/sim_ws/src && cd $HOME/sim_ws/src
git clone https://github.com/AtlasAutoware/carv1 atlasautoware   # the paths below expect this folder name
# point map_path in config/sim.yaml at <your_home>/sim_ws/src/atlasautoware/maps/levine
cd $HOME/sim_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build
source /opt/ros/humble/setup.bash && source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

**Docker (NVIDIA GPU):**
```bash
docker build -t f1tenth_gym_ros -f Dockerfile .
rocker --nvidia --x11 --volume .:/sim_ws/src/f1tenth_gym_ros -- f1tenth_gym_ros
```

**Docker (no GPU, noVNC):**
```bash
docker-compose up
# second terminal:
docker exec -it f1tenth_gym_ros-sim-1 /bin/bash
# browser: http://localhost:8080/vnc.html → Connect, then launch as above
```

Use `headless_bridge_launch.py` instead of `gym_bridge_launch.py` for
benchmarking/auto-tuning without rviz.

## Quickstart — real car

Jetson + OAK-D Pro (RGB + 200 Hz IMU) + RPLidar + VESC. Actuation goes
through a PCA9685 PWM board into the VESC's PPM input **or** direct VESC
UART — `drive_node` probes both at startup and uses what it finds:

```bash
pip3 install depthai rplidar-roboticia smbus2 pyserial
ros2 launch f1tenth_gym_ros car_bringup_launch.py
```

Install and compile the optional car detector once on the Jetson, then enable
it in bringup:

```bash
cd ~/atlas_ws/src/atlasautoware
hardware/scripts/install_tensorrt.sh
cd ~/atlas_ws
colcon build --packages-select f1tenth_gym_ros
source install/setup.bash
ros2 launch f1tenth_gym_ros car_bringup_launch.py use_perception:=true
```

Calibration (steering trim, `erpm_gain`, speed scaling) lives in
`config/hardware.yaml`. Wiring, VESC Tool setup, and the first-drive
calibration order are in [docs/hardware.md](docs/hardware.md). Both actuation
paths carry an arming hold, a command watchdog, and neutral-on-shutdown.
YOLO opponent detection uses a device-built TensorRT engine directly on the
Jetson GPU. The ONNX export is only an engine-build input; ONNX Runtime is not
loaded in the car process. See [docs/camera_perception.md](docs/camera_perception.md).

## Configuration

- `config/sim.yaml` — simulation: `map_path`, `num_agent` (1 or 2), start
  poses, `kb_teleop` (then `ros2 run teleop_twist_keyboard
  teleop_twist_keyboard`: `i`/`u`/`o` forward, `,`/`m`/`.` reverse, `k` stop).
- `config/hardware.yaml` — drive backend + sensor + racing parameters for the
  car.
- `config/slam_mapping.yaml` — mapping session settings.

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/scan` | LaserScan | sim bridge / rplidar_node → stack |
| `/ego_racecar/odom` (sim) · `/pf/pose/odom` (car) | Odometry | localization → controllers |
| `/oakd/rgb`, `/oakd/camera_info` | Image, CameraInfo | oakd_camera → camera_perception |
| `/oakd/imu` | Imu | oakd_camera → traction governor |
| `/vesc/odom` | Odometry | drive_node (UART backend) → particle filter |
| `/drive` | AckermannDriveStamped | controllers → sim bridge / drive_node |
| `/map`, `tf` | — | map server / SLAM |

Two-agent sim additionally has `/opp_scan`, `/opp_drive`,
`/opp_racecar/odom`, and mirrored `opp_odom` topics; reset poses via RViz's
*2D Pose Estimate* (`/initialpose`) and *2D Goal Pose* tools.

## Tests

```bash
python3 -m pytest tests/ -q        # pure-logic: MPC, race brain, hardware drivers
python3 tests/test_mpc.py          # closed-loop MPC validation + solve-time budget
```

`tests/test_hardware.py` covers the PCA9685 register/pulse maths, the VESC
UART protocol, drive-backend auto-detection, lidar scan binning, and the
traction governor — no hardware or ROS needed.

## License

MIT — see [LICENSE](LICENSE).
