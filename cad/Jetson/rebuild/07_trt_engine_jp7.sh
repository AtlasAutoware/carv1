#!/bin/bash
# Rebuild the student policy's TensorRT engine on JetPack 7.2.1 / TensorRT 10.16.2.
#
# A TRT engine is tied to the exact TensorRT version and the GPU it was built on, so the
# engine from the JetPack 6.2 install is dead and this has to run on the car itself.
#
# The model has four inputs with a dynamic batch dimension, so trtexec needs an explicit
# optimisation profile; without one it refuses to build. Batch 1 is what the car runs, and
# the profile allows up to 4 so a batched bench test is still possible.
set -u
MODEL=${MODEL:-$HOME/atlas_ws/src/atlasautoware/models/student.onnx}
OUT=${OUT:-$HOME/atlas_ws/src/atlasautoware/models/student.engine}
SHAPES_MIN="front:1x3x96x128,bev:1x1x96x96,state:1x5,ids:1x24"
SHAPES_OPT="$SHAPES_MIN"
SHAPES_MAX="front:4x3x96x128,bev:4x1x96x96,state:4x5,ids:4x24"

echo "== TensorRT $(dpkg -l | awk '/libnvinfer-bin/{print $3}')"
echo "== model $MODEL"
[ -f "$MODEL" ] || { echo "FAIL: model not found"; exit 1; }

echo "== building FP16 engine (this takes a few minutes)"
trtexec --onnx="$MODEL" --saveEngine="$OUT" --fp16 \
        --minShapes="$SHAPES_MIN" --optShapes="$SHAPES_OPT" --maxShapes="$SHAPES_MAX" \
        --skipInference > /tmp/trt_build.log 2>&1
RC=$?
if [ $RC -ne 0 ]; then
    echo "FAIL: trtexec exited $RC"
    grep -iE "error|failed|could not|no such" /tmp/trt_build.log | tail -15
    exit $RC
fi
echo "built: $(ls -la "$OUT" | awk '{print $5" bytes"}')"

echo "== timing it at batch 1"
trtexec --loadEngine="$OUT" --shapes="$SHAPES_OPT" --iterations=200 --warmUp=500 \
        --avgRuns=100 > /tmp/trt_run.log 2>&1
grep -E "Latency:|Throughput:|GPU Compute Time: min|mean =" /tmp/trt_run.log | tail -6
echo "== done"
