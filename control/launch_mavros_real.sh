#!/bin/bash
# MAVROS2 connected to real ArduPilot FC via USB-to-TTL adapter.
# Default device: /dev/ttyUSB0 — override with: FCU_DEV=/dev/ttyUSB1 bash launch_mavros_real.sh
set -e
source /opt/ros/humble/setup.bash

FCU_DEV="${FCU_DEV:-/dev/ttyUSB0}"

if [ ! -c "$FCU_DEV" ]; then
    echo "[mavros_real] ERROR: $FCU_DEV not found"
    echo "  Plug in USB-to-TTL adapter and check: ls /dev/ttyUSB*"
    exit 1
fi
pkill -f mavros_node 2>/dev/null || true; sleep 1

echo "[mavros_real] Connecting to ArduPilot FC at $FCU_DEV:921600 ..."
ros2 run mavros mavros_node \
    --ros-args \
    -p fcu_url:="serial://${FCU_DEV}:921600" \
    -p tgt_system:=1 \
    -p tgt_component:=1 \
    -p log_output:="screen" \
    -p fcu_protocol:="v2.0"
# plugin_denylist removed — param plugin enabled for real hardware
# Add -p gcs_url:="udp://@GCS_PC_IP:14550" for telemetry to Mission Planner
