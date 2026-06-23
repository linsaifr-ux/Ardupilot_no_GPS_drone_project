#!/bin/bash
# launch_mavros_px4.sh — MAVROS2 connected to PX4 SITL (no-GPS external vision).
#
# PX4 SITL onboard MAVLink: listens udp 14580, sends to 14540 (px4-rc.mavlink).
# MAVROS binds 14540 and auto-learns PX4's address.  MAVROS auto-detects PX4 and
# loads the px4 mode map (OFFBOARD, AUTO.*) — no apm plugin denylist needed.

source /opt/ros/humble/setup.bash
pkill -f mavros_node 2>/dev/null; sleep 1

ros2 run mavros mavros_node \
    --ros-args \
    -p fcu_url:="udp://:14540@127.0.0.1:14580" \
    -p tgt_system:=1 \
    -p tgt_component:=1
