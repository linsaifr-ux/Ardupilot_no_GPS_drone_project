#!/bin/bash
# Launch USB3 camera driver on Jetson (AP-IMX900-Mini-USB3-I5, usb_camera_node.py).
source /opt/ros/humble/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/ros2_env.sh"

# Auto-detect by USB descriptor name instead of a hardcoded /dev/videoN — the
# Jetson's other camera (IMX219, CSI) can be connected at the same time and
# /dev/videoN enumeration order isn't guaranteed stable across reboots/replugs.
# The AP-IMX900's capture node is always the first /dev/videoN listed under
# its v4l2-ctl block (subsequent ones are metadata-only, no formats). Set
# CAMERA_DEV explicitly to skip auto-detection.
CAMERA_DEV="${CAMERA_DEV:-}"
if [ -z "$CAMERA_DEV" ]; then
    CAMERA_DEV=$(v4l2-ctl --list-devices 2>/dev/null | awk '
        /APPROPHO|IMX900/ { found=1; next }
        found && /\/dev\/video/ { print $1; exit }
        /^[^[:space:]]/ { found=0 }
    ')
fi
if [ -z "$CAMERA_DEV" ]; then
    echo "[camera] WARNING: could not auto-detect the AP-IMX900 (no APPROPHO/IMX900 entry in 'v4l2-ctl --list-devices') — falling back to /dev/video0; set CAMERA_DEV=/dev/videoN if that's wrong" >&2
    CAMERA_DEV=/dev/video0
fi
echo "[camera] Using $CAMERA_DEV"

exec python3 "$SCRIPT_DIR/usb_camera_node.py" --device "$CAMERA_DEV" --width 1280 --height 960 --fps 30
