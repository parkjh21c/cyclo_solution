#!/usr/bin/env bash
# TensorRT FP16 engines of the DYNAMIXEL YOLO models, next to the installed .pt files.
# Run inside the GPU-enabled demo container on the Jetson that runs them (engines are GPU and
# TensorRT version specific; not in git). object_detector loads MODEL.engine only while
# MODEL.engine.sha256 matches MODEL.pt, else the .pt. Restart full_demo afterwards.
set -euo pipefail
python="${PICK_PYTHON:-/opt/demo-pick-venv/bin/python3}"
models="${1:-/root/ros2_ws/install/ffw_vision/share/ffw_vision/models}"
imgsz="${IMGSZ:-640}"  # must equal imgsz / obb_imgsz of the task file
for name in dynamixel_seg_n640 dynamixel_obb_n640; do
  pt="$models/$name.pt"
  [[ -r "$pt" ]] || { echo "missing $pt" >&2; exit 1; }
  work="$(mktemp -d)"
  cp "$pt" "$work/"
  (cd "$work" && YOLO_AUTOINSTALL=false "$python" -c \
    "import sys; from ultralytics import YOLO; YOLO(sys.argv[1]).export(format='engine', half=True, imgsz=int(sys.argv[2]), device=0)" \
    "$name.pt" "$imgsz")
  mv "$work/$name.engine" "$models/$name.engine"
  sha256sum "$pt" | cut -d' ' -f1 > "$models/$name.engine.sha256"
  rm -rf "$work"
  echo "built $models/$name.engine"
done
echo "Restart full_demo to load them; no ROS commands were sent."
