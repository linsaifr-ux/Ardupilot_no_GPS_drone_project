#!/bin/bash
# launch_px4_sitl.sh — Start PX4 SITL for no-GPS external-vision drone.
#
# Run order:
#   1. PX4_SIM=1 python3 control/drone_sim.py     (bridge on TCP 4560 — MUST be first)
#   2. bash control/launch_px4_sitl.sh [--wipe]   (this script)
#   3. bash control/apply_px4_params.sh            (first-time only; then restart PX4)
#   4. bash control/launch_mavros_px4.sh
#   5. python3 control/px4_commander.py
#
# --wipe: delete saved parameters.bson so PX4 resets to autostart defaults.
#         Necessary the very first time or after changing SYS_AUTOSTART.
#
# No -d (daemon) flag: required so that px4-param IPC finds the socket in rootfs.
# Uses setsid+nohup so PX4 survives when this shell exits.

PX4_BIN="$HOME/PX4-Autopilot/build/px4_sitl_nolockstep/bin/px4"
PX4_ROOTFS="$HOME/PX4-Autopilot/build/px4_sitl_nolockstep/rootfs"
PX4_LOG="/tmp/px4_sitl.log"

# Kill any existing PX4 instances
if pgrep -x px4 >/dev/null 2>&1; then
    echo "[PX4SITL] Killing existing PX4 instance..."
    pkill -9 -x px4 2>/dev/null
    sleep 1
fi

if [[ "$1" == "--wipe" ]]; then
    echo "[PX4SITL] Wiping saved parameters (clean slate)..."
    rm -f "$PX4_ROOTFS/parameters.bson" "$PX4_ROOTFS/parameters_backup.bson"
fi

# Bridge MUST already be running on TCP 4560
if ! ss -tlnp 2>/dev/null | grep -q ":4560 "; then
    echo "[PX4SITL] ERROR: nothing listening on TCP 4560."
    echo "[PX4SITL]   Start bridge first:  PX4_SIM=1 python3 control/drone_sim.py"
    exit 1
fi

echo "[PX4SITL] Starting PX4 SITL (SYS_AUTOSTART=10016, no-lockstep)..."
echo "[PX4SITL] Log: $PX4_LOG"
cd "$PX4_ROOTFS" || { echo "[PX4SITL] ERROR: rootfs not found: $PX4_ROOTFS"; exit 1; }

# setsid + nohup: new session (survives terminal close) + ignore SIGHUP.
# NO -d flag: keeps working-dir=rootfs so px4-param IPC socket is resolvable.
setsid nohup env PX4_SYS_AUTOSTART=10016 "$PX4_BIN" > "$PX4_LOG" 2>&1 &
PX4_PID=$!
echo $PX4_PID > /tmp/px4_sitl.pid
echo "[PX4SITL] PX4 PID=$PX4_PID  (saved to /tmp/px4_sitl.pid)"

# Wait for simulator_mavlink to connect (TCP 4560 handshake) and UDP 14580 to bind
echo "[PX4SITL] Waiting for PX4 startup (up to 20 s)..."
for i in $(seq 1 40); do
    sleep 0.5
    grep -q "Simulator connected" "$PX4_LOG" 2>/dev/null && \
    ss -ulnp 2>/dev/null | grep -q ":14580 " && break
done

if ss -ulnp 2>/dev/null | grep -q ":14580 "; then
    echo "[PX4SITL] ✓ MAVLink ready  (UDP 14580 offboard / 18570 GCS)"
    echo "[PX4SITL] ✓ MAVROS: bash control/launch_mavros_px4.sh"
    echo "[PX4SITL]   Params:  bash control/apply_px4_params.sh  (first run only)"
else
    echo "[PX4SITL] WARNING: UDP 14580 not seen after 20 s — check $PX4_LOG"
    tail -20 "$PX4_LOG"
fi
