#!/bin/bash
# Start MediaMTX relay server for drone video streaming.
# Jetson pushes to rtsp://THIS_PC_IP:8554/drone
# Ground station pulls from rtsp://THIS_PC_IP:8554/drone
#   or opens http://THIS_PC_IP:8888/drone in a browser (HLS)
#   or opens http://THIS_PC_IP:8889/drone in a browser (WebRTC)

set -e
cd "$(dirname "$0")"

echo "[mediamtx] Starting drone relay server ..."
echo "[mediamtx] RTSP  : rtsp://$(hostname -I | awk '{print $1}'):8554/drone"
echo "[mediamtx] WebRTC: http://$(hostname -I | awk '{print $1}'):8889/drone"
echo "[mediamtx] HLS   : http://$(hostname -I | awk '{print $1}'):8888/drone"
echo "[mediamtx] Press Ctrl+C to stop."

exec ./mediamtx mediamtx.yml
