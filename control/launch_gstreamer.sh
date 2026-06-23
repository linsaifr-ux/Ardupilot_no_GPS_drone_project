#!/bin/bash
# Launch GStreamer H.265 streamer → ground station.
# Opens the camera directly with OpenCV (no ROS2).
#
# IMPORTANT: Do NOT run launch_camera.sh at the same time — both cannot open
#            /dev/video0 simultaneously ("Device or resource busy").
#
# Usage:
#   bash control/launch_gstreamer.sh                         # uses GROUND_IP env or default
#   bash control/launch_gstreamer.sh --host 192.168.1.50
#   bash control/launch_gstreamer.sh --host 192.168.1.50 --camera 0 --port 5000
#   GROUND_IP=192.168.1.50 bash control/launch_gstreamer.sh
#
# Receive on ground station:
#   gst-launch-1.0 udpsrc port=5000 ! \
#       application/x-rtp,encoding-name=H265,payload=96 ! \
#       rtph265depay ! h265parse ! avdec_h265 ! \
#       videoconvert ! autovideosink sync=false
#
# The right panel shows anyloc/latest_match.jpg — updated by anyloc/ros2_node.py
# Run alongside launch_real_hw.sh (which uses the AnyLoc + YOLO ROS2 nodes).
# In that case, launch_real_hw.sh must NOT start launch_camera.sh separately.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Kill any stale instance
pkill -f "gstreamer_stream.py" 2>/dev/null || true
sleep 1

echo "=== GStreamer streamer starting ==="
exec python3 -u "$PROJECT_DIR/tools/gstreamer_stream.py" "$@"
