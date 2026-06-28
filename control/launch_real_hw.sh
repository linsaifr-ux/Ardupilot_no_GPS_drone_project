#!/bin/bash
# Full system launch for real hardware contest flight.
#
# Usage:
#   bash control/launch_real_hw.sh [--waypoint-file FILE] [--stream-host IP]
#
#   --stream-host IP     Enable ground view stream → direct UDP to this GS IP.
#                        Receive: gst-launch-1.0 udpsrc port=5000 ! \
#                            application/x-rtp,encoding-name=H265,payload=96 ! \
#                            rtph265depay ! h265parse ! avdec_h265 ! \
#                            videoconvert ! autovideosink sync=false
#   --stream-server IP   Enable ground view stream → RTSP push to MediaMTX relay.
#                        Watch: vlc rtsp://IP:8554/drone
#                               http://IP:8889/drone  (WebRTC browser)
#
# --stream-host and --stream-server are mutually exclusive.
# Without either flag the plain ROS2 camera driver runs (no ground stream).

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

source /opt/ros/humble/setup.bash

STREAM_HOST=""
STREAM_SERVER=""
COMMANDER_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --stream-host)   STREAM_HOST="$2";   shift 2 ;;
        --stream-server) STREAM_SERVER="$2"; shift 2 ;;
        *) COMMANDER_ARGS+=("$1"); shift ;;
    esac
done

if [[ -n "$STREAM_HOST" && -n "$STREAM_SERVER" ]]; then
    echo "ERROR: --stream-host and --stream-server are mutually exclusive" >&2
    exit 1
fi

echo "=== Real Hardware Launch ==="
echo "Project: $PROJECT_DIR"
[[ -n "$STREAM_HOST"   ]] && echo "Stream: ground view UDP  → $STREAM_HOST:5000"
[[ -n "$STREAM_SERVER" ]] && echo "Stream: ground view RTSP → rtsp://$STREAM_SERVER:8554/drone"

# 1. MAVROS
echo "[launch] Starting MAVROS ..."
bash "$SCRIPT_DIR/launch_mavros_real.sh" &
MAVROS_PID=$!
echo "[launch] MAVROS PID=$MAVROS_PID; waiting 6 s ..."
sleep 6

# 2. Camera — plain ROS2 driver OR ground view streamer (not both)
if [[ -n "$STREAM_HOST" ]]; then
    echo "[launch] Starting ground view streamer (UDP) → $STREAM_HOST ..."
    python3 -u "$PROJECT_DIR/tools/ground_view_stream.py" --host "$STREAM_HOST" &
    CAMERA_PID=$!
elif [[ -n "$STREAM_SERVER" ]]; then
    echo "[launch] Starting ground view streamer (RTSP) → $STREAM_SERVER ..."
    python3 -u "$PROJECT_DIR/tools/ground_view_stream.py" --stream-server "$STREAM_SERVER" &
    CAMERA_PID=$!
else
    echo "[launch] Starting camera driver ..."
    bash "$SCRIPT_DIR/launch_camera.sh" &
    CAMERA_PID=$!
fi
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
python3 "$SCRIPT_DIR/ardupilot_commander.py" "${COMMANDER_ARGS[@]}"
CMD_EXIT=$?

echo "[launch] Commander exited ($CMD_EXIT) — shutting down ..."
kill $YOLO_PID $ANYLOC_PID $BRIDGE_PID $CAMERA_PID $MAVROS_PID 2>/dev/null || true
exit $CMD_EXIT
