#!/bin/bash
# launch_commander.sh — Run the autonomous flight commander.
#
# Prerequisites: SITL and MAVROS2 must already be running.
#   Terminal 1: bash control/launch_sitl.sh --wipe   (first run)
#               bash control/launch_sitl.sh           (subsequent runs)
#               → type 'reboot' in MAVProxy on first run to save params
#   Terminal 2: bash control/launch_mavros.sh
#   Terminal 3: bash control/launch_commander.sh      ← this script

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

source /opt/ros/jazzy/setup.bash

echo "[Commander] Starting flight_commander.py..."
python3 "$SCRIPT_DIR/flight_commander.py"
