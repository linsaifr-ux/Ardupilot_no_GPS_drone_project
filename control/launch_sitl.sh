#!/bin/bash
# launch_sitl.sh — Start ArduPilot SITL + MAVProxy directly (bypasses sim_vehicle.py
# which corrupts the --out port argument on this build, causing MAVROS2 to receive
# no UDP packets).
#
# Usage:
#   First run (wipe eeprom, load params, then reboot in MAVProxy):
#     bash control/launch_sitl.sh --wipe
#
#   Subsequent runs (params already in eeprom):
#     bash control/launch_sitl.sh
#
# Run order:
#   Terminal 1: SITL    → bash control/launch_sitl.sh [--wipe]
#               On first run: type 'reboot' in MAVProxy prompt, wait for "Saved N params"
#   Terminal 2: MAVROS2 → bash control/launch_mavros.sh
#   Terminal 3: Flight  → python3 control/flight_commander.py

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
ARDUCOPTER="$PROJECT_DIR/third_party/ardupilot/build/sitl/bin/arducopter"
PARM_FILE="$SCRIPT_DIR/no_gps.parm"

# Home position
HOME_LAT=23.450868
HOME_LON=120.286135
HOME_ALT=28.17
HOME_HDG=0

# Kill any stale instances
pkill -f 'arducopter.*JSON' 2>/dev/null || true
pkill -f 'mavproxy' 2>/dev/null || true
sleep 1

WIPE_FLAG=""
if [[ "$1" == "--wipe" ]]; then
    WIPE_FLAG="-w"
    echo "[SITL] Starting with --wipe (params will be loaded from $PARM_FILE)"
    echo "[SITL] Deleting stale eeprom.bin and mav.parm to reset EKF state..."
    rm -f "$PROJECT_DIR/eeprom.bin" "$PROJECT_DIR/mav.parm"
    echo "[SITL] After MAVProxy connects, type 'reboot' to save params to eeprom."
else
    echo "[SITL] Starting with existing eeprom (no wipe)"
fi

# Start arducopter in background
echo "[SITL] Starting arducopter..."
"$ARDUCOPTER" \
    $WIPE_FLAG \
    --model JSON \
    --speedup 1 \
    --defaults "$PARM_FILE" \
    --sim-address=127.0.0.1 \
    --home "$HOME_LAT,$HOME_LON,$HOME_ALT,$HOME_HDG" \
    -I0 &
ARDU_PID=$!
echo "[SITL] arducopter PID: $ARDU_PID"

# Wait for arducopter TCP port 5760 to be ready
echo "[SITL] Waiting for TCP 5760..."
for i in $(seq 1 30); do
    nc -z 127.0.0.1 5760 2>/dev/null && break
    sleep 0.5
done
echo "[SITL] TCP 5760 ready"

# Start mavproxy with explicit --out udp:host:port
echo "[SITL] Starting MAVProxy → UDP 14550..."
mavproxy.py \
    --retries 5 \
    --master tcp:127.0.0.1:5760 \
    --sitl 127.0.0.1:5501 \
    --out udp:127.0.0.1:14550

# If mavproxy exits, also kill arducopter
kill $ARDU_PID 2>/dev/null || true
