#!/bin/bash
# launch_commander_ardupilot.sh — Run the ArduPilot autonomous flight commander.
#
# Prerequisites: Isaac Sim bridge (UDP 9002) or drone_sim.py, ArduPilot SITL
# via MAVProxy, and MAVROS must be running.
#   bash control/launch_sitl.sh [--wipe]    (UDP 9002 bridge + MAVProxy)
#   bash control/launch_mavros.sh
#
# First run after --wipe: type 'reboot' in the MAVProxy console to persist params.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash

echo "[Commander AP] Starting ardupilot_commander.py..."
PYTHONUNBUFFERED=1 python3 "$SCRIPT_DIR/ardupilot_commander.py" "$@"
