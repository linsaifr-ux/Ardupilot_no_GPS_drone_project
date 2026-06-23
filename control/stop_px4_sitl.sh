#!/bin/bash
# stop_px4_sitl.sh — Stop the PX4 SITL process.
#
# Tries in order:
#   1. MAVLink MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN via MAVROS (graceful)
#   2. SIGTERM to PID saved in /tmp/px4_sitl.pid
#   3. SIGTERM via pkill (fallback)
#   4. SIGKILL if still alive
#
# Usage:
#   bash control/stop_px4_sitl.sh

ROS_SETUP="/opt/ros/humble/setup.bash"
PID_FILE="/tmp/px4_sitl.pid"

_px4_running() { pgrep -x px4 &>/dev/null; }

if ! _px4_running; then
    echo "[stop_px4] PX4 is not running."
    rm -f "$PID_FILE"
    exit 0
fi

# ── 1. Graceful MAVLink shutdown (requires MAVROS to be up) ──────────────────
if [[ -f "$ROS_SETUP" ]]; then
    echo "[stop_px4] Trying MAVLink shutdown (MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN)..."
    (source "$ROS_SETUP" && \
     timeout 5 ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong \
       "{command: 246, param1: 2.0}") &>/dev/null
    sleep 3
    if ! _px4_running; then
        echo "[stop_px4] PX4 stopped ✓  (MAVLink shutdown)"
        rm -f "$PID_FILE"
        exit 0
    fi
fi

# ── 2. SIGTERM to saved PID ───────────────────────────────────────────────────
if [[ -f "$PID_FILE" ]]; then
    PID=$(cat "$PID_FILE")
    if kill -0 "$PID" 2>/dev/null; then
        echo "[stop_px4] Sending SIGTERM to PID $PID..."
        kill -TERM "$PID"
        sleep 2
        if ! _px4_running; then
            echo "[stop_px4] PX4 stopped ✓  (SIGTERM)"
            rm -f "$PID_FILE"
            exit 0
        fi
    fi
    rm -f "$PID_FILE"
fi

# ── 3. pkill SIGTERM ──────────────────────────────────────────────────────────
echo "[stop_px4] Sending SIGTERM via pkill..."
pkill -TERM -x px4
sleep 2

if ! _px4_running; then
    echo "[stop_px4] PX4 stopped ✓  (pkill SIGTERM)"
    exit 0
fi

# ── 4. SIGKILL ────────────────────────────────────────────────────────────────
echo "[stop_px4] Still running — sending SIGKILL..."
pkill -9 -x px4
sleep 1

if ! _px4_running; then
    echo "[stop_px4] PX4 stopped ✓  (SIGKILL)"
else
    echo "[stop_px4] WARNING: could not stop PX4 — check with: pgrep -x px4"
    exit 1
fi
