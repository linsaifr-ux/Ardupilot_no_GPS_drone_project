#!/bin/bash
# Launch CSI camera driver on Jetson (IMX219, nvarguscamerasrc/ISP).
source /opt/ros/humble/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SENSOR_ID="${CAMERA_SENSOR_ID:-0}"
echo "[camera] Using CSI sensor-id=$SENSOR_ID (IMX219)"

exec python3 "$SCRIPT_DIR/csi_camera_node.py" --sensor-id "$SENSOR_ID" --width 1640 --height 1232 --fps 30
