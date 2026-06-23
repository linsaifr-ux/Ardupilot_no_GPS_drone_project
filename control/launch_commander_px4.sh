#!/bin/bash
# launch_commander_px4.sh — Run the PX4 autonomous flight commander.
#
# Prerequisites: Isaac Sim bridge (TCP 4560), PX4 SITL, and MAVROS must be running.
#   bash simulator/run_chiayi.sh --px4   (TCP 4560 bridge)
#   bash control/launch_px4_sitl.sh
#   bash control/apply_px4_params.sh     (first run only)
#   bash control/launch_mavros_px4.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash

echo "[Commander PX4] Starting px4_commander.py..."
PYTHONUNBUFFERED=1 python3 "$SCRIPT_DIR/px4_commander.py" "$@"
