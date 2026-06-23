#!/bin/bash
source /opt/ros/humble/setup.bash
cd "$(dirname "$0")/.."
/home/jetson/venv/yolo/bin/python3 -u detection/ros2_node.py "$@"
