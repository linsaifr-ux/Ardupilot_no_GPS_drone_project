#!/bin/bash
# Launch USB camera driver on Jetson
source /opt/ros/humble/setup.bash

# Find camera device — AP-IMX900 typically appears as /dev/video0
CAMERA_DEV="${CAMERA_DEV:-/dev/video0}"
echo "[camera] Using $CAMERA_DEV"

ros2 run v4l2_camera v4l2_camera_node \
    --ros-args \
    -p video_device:="$CAMERA_DEV" \
    -p image_size:=[1280,720] \
    -p pixel_format:="YUYV" \
    -p output_encoding:="rgb8" \
    -p camera_frame_rate:=30.0 \
    -r /image_raw:=/drone/camera/image_raw \
    -r /camera_info:=/drone/camera/camera_info
