#!/bin/bash
# MAVROS2 connected to real ArduPilot FC via USB-to-TTL adapter.
# Default device: /dev/ttyUSB0 — override with: FCU_DEV=/dev/ttyUSB1 bash launch_mavros_real.sh
# Set MAVLINK_RELAY=1 to also bridge the full MAVLink stream to
# control/mavlink_relay_client.py --role vehicle (must be started separately,
# or via launch_real_hw.sh --mavlink-relay) — see streaming/mavlink_relay_setup.md.
set -e
source /opt/ros/humble/setup.bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/ros2_env.sh"

FCU_DEV="${FCU_DEV:-/dev/ttyUSB0}"
MAVLINK_RELAY="${MAVLINK_RELAY:-0}"

if [ ! -c "$FCU_DEV" ]; then
    echo "[mavros_real] ERROR: $FCU_DEV not found"
    echo "  Plug in USB-to-TTL adapter and check: ls /dev/ttyUSB*"
    exit 1
fi
pkill -f mavros_node 2>/dev/null || true; sleep 1

echo "[mavros_real] Connecting to ArduPilot FC at $FCU_DEV:921600 ..."

# Request all data streams at 10 Hz once MAVROS connects.
# MAVROS sends REQUEST_DATA_STREAM rate=0 on startup which overrides ArduPilot SR* params,
# so we must explicitly re-request streams after connection is established.
(
  source /opt/ros/humble/setup.bash
  echo "[mavros_real] Waiting for MAVROS to connect..."
  until ros2 topic echo /mavros/state --once 2>/dev/null | grep -q "connected: true"; do
    sleep 1
  done
  echo "[mavros_real] Connected — requesting data streams at 10 Hz..."
  for id in 1 2 3 4 6 10 11 12; do
    ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate \
      "{stream_id: $id, message_rate: 10, on_off: true}" > /dev/null 2>&1
  done
  echo "[mavros_real] Data streams enabled."
) &

GCS_ARGS=()
if [ "$MAVLINK_RELAY" = "1" ]; then
    echo "[mavros_real] MAVLINK_RELAY=1 — bridging to control/mavlink_relay_client.py at tcp://127.0.0.1:5760"
    echo "  (start it separately, or use launch_real_hw.sh --mavlink-relay, or mavros will just retry the connection)"
    GCS_ARGS=(-p gcs_url:="tcp://127.0.0.1:5760")
fi

ros2 run mavros mavros_node \
    --ros-args \
    -p fcu_url:="serial://${FCU_DEV}:921600" \
    -p tgt_system:=1 \
    -p tgt_component:=1 \
    -p log_output:="screen" \
    -p fcu_protocol:="v2.0" \
    "${GCS_ARGS[@]}"
# plugin_denylist removed — param plugin enabled for real hardware
