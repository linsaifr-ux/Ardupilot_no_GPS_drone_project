#!/bin/bash
# Launch MAVROS2 connected to ArduPilot SITL via UDP.
#
# Port allocation:
#   TCP 5760   — MAVProxy master (SITL internal; sim_vehicle.py connects here)
#   UDP 14550  — MAVROS2 (this script); flight_commander.py EKF + altitude
#                monitoring also route through MAVROS2 via /mavros/mavlink/from+to
#
# Only ONE --out udp flag needed in the SITL command (14550 for MAVROS2).
# flight_commander.py no longer needs a separate pymavlink UDP port.
#
# Prerequisites (one-time):
#   sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-mavros-msgs
#   sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
#
# Run order:
#   Terminal 1: SITL         (sim_vehicle.py ... --out udp:127.0.0.1:14550)
#   Terminal 2: drone_sim    (python3 control/drone_sim.py)
#   Terminal 3: MAVROS2      (this script)
#   Terminal 4: AnyLoc node  (./anyloc/run_ros2_localizer.sh)
#   Terminal 5: Flight cmd   (python3 control/flight_commander.py)

source /opt/ros/humble/setup.bash

# Kill any stale MAVROS2 instance (prevents "Promise already satisfied" crash)
pkill -f mavros_node 2>/dev/null; sleep 1

# Wait for SITL to be ready before connecting (prevents race condition on startup)
echo "[mavros] Waiting 5 s for SITL to be ready..."
sleep 5

ros2 run mavros mavros_node \
    --ros-args \
    -p fcu_url:="udp://:14550@" \
    -p tgt_system:=1 \
    -p tgt_component:=1 \
    -p log_output:="screen" \
    -p fcu_protocol:="v2.0" \
    -p plugin_denylist:="['param']"
