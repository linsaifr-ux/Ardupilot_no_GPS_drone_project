#!/bin/bash
# Full system launch for real hardware contest flight.
#
# Usage:
#   bash control/launch_real_hw.sh                           (uses control/survey.waypoints)
#   bash control/launch_real_hw.sh --waypoint-file my.waypoints
#
# Terminal layout (recommend tmux):
#   Pane 0: MAVROS         (this script pane 0)
#   Pane 1: Camera         (pane 1)
#   Pane 2: HW Bridge      (pane 2)
#   Pane 3: AnyLoc         (pane 3)
#   Pane 4: YOLO           (pane 4)
#   Pane 5: GStreamer       (pane 5 — optional, streams to ground station)
#   Pane 6: Commander      (pane 6 — foreground)
#
# GStreamer stream (optional, REPLACES camera pane — opens /dev/video0 directly):
#   SKIP launch_camera.sh if using GStreamer — both cannot open /dev/video0.
#   Instead run: bash control/launch_gstreamer.sh --host <GROUND_IP>
# Receive on ground station:
#   gst-launch-1.0 udpsrc port=5000 ! \
#       application/x-rtp,encoding-name=H265,payload=96 ! \
#       rtph265depay ! h265parse ! avdec_h265 ! \
#       videoconvert ! autovideosink sync=false

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

source /opt/ros/humble/setup.bash

echo "=== Real Hardware Launch ==="
echo "Project: $PROJECT_DIR"

# 1. MAVROS
echo "[launch] Starting MAVROS ..."
bash "$SCRIPT_DIR/launch_mavros_real.sh" &
MAVROS_PID=$!
echo "[launch] MAVROS PID=$MAVROS_PID; waiting 6 s ..."
sleep 6

# 2. Camera
echo "[launch] Starting camera driver ..."
bash "$SCRIPT_DIR/launch_camera.sh" &
CAMERA_PID=$!
sleep 3

# 3. Hardware bridge
echo "[launch] Starting hardware bridge ..."
python3 "$SCRIPT_DIR/hw_bridge.py" &
BRIDGE_PID=$!
sleep 2

# 4. AnyLoc (headless — no display needed during flight)
echo "[launch] Starting AnyLoc localizer ..."
/home/jetson/venv/anyloc/bin/python3 -u "$PROJECT_DIR/anyloc/ros2_node.py" --headless &
ANYLOC_PID=$!
sleep 4

# 5. YOLO detector (headless)
echo "[launch] Starting YOLO detector ..."
/home/jetson/venv/yolo/bin/python3 -u "$PROJECT_DIR/detection/ros2_node.py" --headless &
YOLO_PID=$!
sleep 2

# 6. Commander (foreground — shows live flight log)
echo "[launch] Starting ArduPilot commander ..."
python3 "$SCRIPT_DIR/ardupilot_commander.py" "$@"
CMD_EXIT=$?

echo "[launch] Commander exited ($CMD_EXIT) — shutting down ..."
kill $YOLO_PID $ANYLOC_PID $BRIDGE_PID $CAMERA_PID $MAVROS_PID 2>/dev/null || true
exit $CMD_EXIT
