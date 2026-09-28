#!/bin/bash
# Node-startup smoke test for the Jazzy rebuild, with no sensors or VESC connected.
#
# What this can and cannot prove, stated plainly: with nothing wired, no node can produce
# data, so this does NOT verify the car works. What it does verify is the thing a distro
# port actually breaks - that every node's shared libraries resolve, its plugins load and
# its parameters parse under Jazzy/Python 3.12. A node that reaches "cannot open
# /dev/ttyUSB0" has passed; a node that dies on an import, a symbol or a plugin has not.
#
# Nothing here publishes to a drive topic. Nodes are started, watched, and killed by PID.
#
# Note: no `set -u`. ROS's setup.bash reads variables it has not set (COLCON_TRACE,
# AMENT_TRACE_SETUP_FILES and friends), so `set -u` makes sourcing it abort the whole
# script instantly with no output, which is exactly what happened the first time this ran.
source /opt/ros/jazzy/setup.bash 2>/dev/null
source "$HOME/f1tenth_ws/install/setup.bash" 2>/dev/null
source "$HOME/atlas_ws/install/setup.bash" 2>/dev/null
: "${ROS_DISTRO:=unset}"

DUR=${DUR:-6}
PASS=0; FAIL=0

check() {   # check <label> <pkg> <exe> [args...]
    local label=$1 pkg=$2 exe=$3; shift 3
    local log; log=$(mktemp)
    timeout ${DUR}s ros2 run "$pkg" "$exe" "$@" > "$log" 2>&1 &
    local pid=$!
    sleep $((DUR - 1))
    kill -INT "$pid" 2>/dev/null; wait "$pid" 2>/dev/null
    # a hard failure is an import/symbol/plugin problem; a device problem is expected
    # An uncaught exception is a crash, and the first version of this script scored one as
    # a pass because it only looked for library errors. ackermann_to_vesc threw
    # UninitializedStaticallyTypedParameterException and was reported green.
    if grep -qiE "ModuleNotFoundError|ImportError|undefined symbol|cannot open shared object|error while loading shared libraries|Failed to load entry point|core dumped|GLIBC" "$log"; then
        echo "  FAIL $label -- library/ABI problem"
        grep -iE "ModuleNotFoundError|ImportError|undefined symbol|cannot open shared object|error while loading shared|GLIBC" "$log" | head -2 | sed 's/^/        /'
        FAIL=$((FAIL+1))
    elif grep -qiE "terminate called|what\(\):|Traceback \(most recent" "$log"; then
        echo "  FAIL $label -- uncaught exception"
        grep -iE "terminate called|what\(\):|Exception" "$log" | head -2 | sed 's/^/        /'
        FAIL=$((FAIL+1))
    elif grep -qiE "does not exist|No such file|failed to open|cannot open|Permission denied|not found|Failed to connect|serial" "$log"; then
        echo "  pass $label -- started, then reported the missing device (expected, nothing wired)"
        grep -iE "does not exist|No such file|failed to open|cannot open|Failed to connect" "$log" | head -1 | sed 's/^/        /'
        PASS=$((PASS+1))
    elif [ -s "$log" ]; then
        echo "  pass $label -- started and ran"
        head -1 "$log" | sed 's/^/        /'
        PASS=$((PASS+1))
    else
        echo "  pass $label -- started, silent"
        PASS=$((PASS+1))
    fi
    rm -f "$log"
}

echo "ROS_DISTRO=$ROS_DISTRO   python=$(python3 --version)"
echo
echo "node startup:"
# Jazzy parameters are statically typed: a node that declares a parameter without a
# default aborts unless it is given one, so the VESC nodes get their real config file
# rather than being run bare. That is how they run on the car anyway.
CFG="$HOME/f1tenth_ws/install/f1tenth_stack/share/f1tenth_stack/config/vesc.yaml"
check "vesc_driver"       vesc_driver    vesc_driver_node       --ros-args --params-file "$CFG"
check "vesc_to_odom"      vesc_ackermann vesc_to_odom_node      --ros-args --params-file "$CFG"
check "ackermann_to_vesc" vesc_ackermann ackermann_to_vesc_node --ros-args --params-file "$CFG"
check "rplidar"           rplidar_ros    rplidar_node
check "ackermann_mux"     ackermann_mux  ackermann_mux

echo
echo "TensorRT engines:"
python3 - <<'PY'
import os, glob
try:
    import tensorrt as trt
except Exception as e:
    print(f"  FAIL cannot import tensorrt: {e}"); raise SystemExit
lg = trt.Logger(trt.Logger.ERROR)
rt = trt.Runtime(lg)
for p in [os.path.expanduser('~/atlas_ws/src/atlasautoware/models/student.engine'),
          os.path.expanduser('~/.cache/atlasautoware/car_yolov8_640.engine')]:
    if not os.path.exists(p):
        print(f"  MISSING {os.path.basename(p)}"); continue
    with open(p, 'rb') as f:
        eng = rt.deserialize_cuda_engine(f.read())
    if eng is None:
        print(f"  FAIL {os.path.basename(p)} did not deserialise on TRT {trt.__version__}"); continue
    names = [eng.get_tensor_name(i) for i in range(eng.num_io_tensors)]
    print(f"  pass {os.path.basename(p)} loads on TRT {trt.__version__}, io={names}")
PY

echo
echo "$PASS passed, $FAIL failed"
exit $FAIL
