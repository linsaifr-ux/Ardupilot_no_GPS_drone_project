#!/bin/bash
# apply_px4_params.sh — set PX4 no-GPS + external-vision params on the running SITL.
# Uses the px4 client (px4-param) which talks to the px4 daemon.  Params persist in
# the rootfs parameters.bson, so a reboot after this keeps them.
#
# Usage: bash control/apply_px4_params.sh   (PX4 SITL must be running)

PX4_BIN="$HOME/PX4-Autopilot/build/px4_sitl_nolockstep/bin"
ROOTFS="$HOME/PX4-Autopilot/build/px4_sitl_nolockstep/rootfs"
PARAMS="$(dirname "$(readlink -f "$0")")/px4_no_gps.params"

cd "$ROOTFS" || exit 1
n=0
while read -r line; do
    # accept lines of the form: param set NAME VALUE   (# comments ignored)
    set -- $line
    if [ "$1" = "param" ] && [ "$2" = "set" ]; then
        "$PX4_BIN/px4-param" set "$3" "$4" >/dev/null 2>&1 && n=$((n+1))
    fi
done < "$PARAMS"
echo "[apply_px4_params] applied $n params"
"$PX4_BIN/px4-param" save >/dev/null 2>&1
echo "[apply_px4_params] saved to $ROOTFS/parameters.bson"

# Reboot PX4: kill the running instance; the bridge keeps TCP 4560 open so
# restarting PX4 re-connects automatically.  EKF2 params only take effect after
# a full restart.
PX4_PID=$(pgrep -x px4 2>/dev/null)
if [ -n "$PX4_PID" ]; then
    echo "[apply_px4_params] Rebooting PX4 (PID $PX4_PID)..."
    kill -9 $PX4_PID 2>/dev/null
    sleep 1
    cd "$ROOTFS" || exit 1
    setsid nohup env PX4_SYS_AUTOSTART=10016 "$PX4_BIN/px4" >> /tmp/px4_sitl.log 2>&1 &
    NEW_PID=$!
    echo "[apply_px4_params] PX4 restarted as PID $NEW_PID"
    # Wait for MAVLink to come back
    for i in $(seq 1 30); do
        sleep 0.5
        ss -ulnp 2>/dev/null | grep -q ":14580 " && break
    done
    ss -ulnp 2>/dev/null | grep -q ":14580 " && \
        echo "[apply_px4_params] ✓ PX4 MAVLink ready" || \
        echo "[apply_px4_params] WARNING: UDP 14580 not seen — check /tmp/px4_sitl.log"
else
    echo "[apply_px4_params] PX4 not running — start it before applying params."
fi
