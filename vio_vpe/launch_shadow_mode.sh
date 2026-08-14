#!/bin/bash
# Live VIO+VPE shadow-mode observer -- see the plan at
# ~/.claude/plans/robust-squishing-forest.md and vio_vpe/fusion_live_node.py's
# header for the invariant this exists to protect: this stack NEVER publishes
# to the flight controller. The pilot flies manually (RC/GCS) exactly as
# today; this just watches and logs.
#
# Deliberately does NOT launch control/ardupilot_commander.py -- that owns
# arming *and* the vision-injection thread, neither of which belongs in a
# passive observer. MAVROS/camera/hw_bridge are launched directly instead,
# modeled on control/launch_real_hw.sh's structure.
#
# Do NOT also run tools/record_field.py while this is running -- it opens
# /dev/video0 directly via OpenCV and will fight usb_camera_node.py for the
# same device (the existing project-wide constraint, see launch_camera.sh's
# device-detect comment). This stack's own CSV logs (vio_vpe/logs/) are the
# record of the run.
#
# Usage:
#   bash vio_vpe/launch_shadow_mode.sh [--sigma-vio M_S] [--sigma-vpe M]
#                                      [--map PATH]
#                                      [--stream-host IP | --stream-server IP]
#                                      [--openhd-ip IP] [--no-openhd]
#                                      [--no-stream] [--no-yolo] [--stream-fps N]
#
#   Also starts the YOLO detector (detection/ros2_node.py --headless) by
#   default, same node/venv as control/launch_real_hw.sh -- purely for the
#   ground crew's situational awareness (feeds tools/ground_view_stream.py's
#   top-left panel); not part of the localization pipeline, so it stopping
#   or crashing has no effect on VIO/VPE/fusion. --no-yolo disables it.
#
#   Streams by default, on BOTH of ground_view_stream.py's mode B and mode C
#   simultaneously (they run alongside each other, same as
#   tools/record_field.py's convention):
#     - RTSP push to the project's standard relay (118.232.160.227 --
#       "Frank's PC" running MediaMTX; same default
#       control/mavlink_relay_client.py's --relay-host uses, see streaming/).
#         Watch: vlc rtsp://118.232.160.227:8554/drone
#                http://118.232.160.227:8889/drone  (WebRTC browser)
#     - OpenHD (H.264 RTP/UDP) to 192.168.2.2 (record_field.py's own default
#       OpenHD ground-station IP).
#   --stream-server IP   Override the RTSP relay target.
#   --stream-host IP     Direct UDP instead of the RTSP relay (mode A, not B).
#   --openhd-ip IP        Override the OpenHD ground-station IP.
#   --no-openhd           Disable just the OpenHD leg.
#   --no-stream           Disable ALL streaming (RTSP/UDP AND OpenHD) --
#                         local logs only.
#   --stream-fps N        Compositing/encode rate for both stream legs
#                         (default 30 here, see the STREAM_FPS comment below).
# --stream-host and --stream-server are mutually exclusive.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
CONTROL_DIR="$PROJECT_DIR/control"

SIGMA_VIO="36.0"
SIGMA_VPE="10.0"
MAP_PATH="$SCRIPT_DIR/maps/imx900_survey47.vpemap"
STREAM_HOST=""
STREAM_SERVER="118.232.160.227"   # project's standard relay -- see streaming/
OPENHD_IP="192.168.2.2"           # record_field.py's own OpenHD default
YOLO_ENABLED="1"
# Back to tools/ground_view_stream.py's own 30fps default (2026-08-14: was
# temporarily lowered to 15 while diagnosing lag on this launcher, which adds
# VIO+VPE on top of the camera+MAVROS+dual-encode+YOLO load launch_real_hw.sh
# already carries -- an 8-core Orin NX running all of it hit load avg ~9).
# That's still true, but the actual growing-lag bug turned out to be
# ground_view_stream.py's PTS timestamps being nominal-rate rather than
# wall-clock-derived (fixed the same day) -- with that fixed, lag no longer
# accumulates regardless of fps, it just means less CPU headroom under load.
# Override with --stream-fps if 30 turns out to reintroduce visible stutter.
STREAM_FPS="30"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --sigma-vio)     SIGMA_VIO="$2";     shift 2 ;;
        --sigma-vpe)     SIGMA_VPE="$2";     shift 2 ;;
        --map)           MAP_PATH="$2";      shift 2 ;;
        --stream-host)   STREAM_HOST="$2"; STREAM_SERVER=""; shift 2 ;;
        --stream-server) STREAM_SERVER="$2"; shift 2 ;;
        --openhd-ip)     OPENHD_IP="$2";     shift 2 ;;
        --no-openhd)     OPENHD_IP="";       shift ;;
        --no-stream)     STREAM_HOST=""; STREAM_SERVER=""; OPENHD_IP=""; shift ;;
        --no-yolo)       YOLO_ENABLED="";    shift ;;
        --stream-fps)    STREAM_FPS="$2";    shift 2 ;;
        *) echo "unknown arg: $1" >&2; exit 1 ;;
    esac
done

if [[ -n "$STREAM_HOST" && -n "$STREAM_SERVER" ]]; then
    echo "ERROR: --stream-host and --stream-server are mutually exclusive" >&2
    exit 1
fi

source /opt/ros/humble/setup.bash
source "$CONTROL_DIR/ros2_env.sh"

echo "=== VIO+VPE Shadow-Mode Launch (no FC publish) ==="
echo "Project: $PROJECT_DIR"
echo "sigma_vio=$SIGMA_VIO m/s  sigma_vpe=$SIGMA_VPE m  map=$MAP_PATH"
[[ -n "$STREAM_HOST"   ]] && echo "Ground view: UDP  -> $STREAM_HOST:5000 @ ${STREAM_FPS}fps"
[[ -n "$STREAM_SERVER" ]] && echo "Ground view: RTSP -> rtsp://$STREAM_SERVER:8554/drone @ ${STREAM_FPS}fps"
[[ -z "$STREAM_HOST$STREAM_SERVER" ]] && echo "Ground view (mode A/B): NOT streamed"
[[ -n "$OPENHD_IP" ]] && echo "Ground view: OpenHD -> $OPENHD_IP:5601 @ ${STREAM_FPS}fps"
[[ -z "$OPENHD_IP" ]] && echo "Ground view (mode C / OpenHD): NOT streamed"
[[ -n "$YOLO_ENABLED" ]] && echo "YOLO detector: enabled (headless)"
[[ -z "$YOLO_ENABLED" ]] && echo "YOLO detector: disabled (--no-yolo)"

# Job control ON even though this script is usually run non-interactively --
# without it, `cmd &` shares this script's own process group, so killing a
# tracked PID doesn't reach anything IT spawns in turn (e.g. launch_mavros_real.sh
# -> `ros2 run mavros mavros_node`, itself run in ITS OWN foreground, one more
# level down). With job control on, each `&` becomes its own process-group
# leader, so `kill -- -$pid` (negative = process group) reliably takes the
# whole subtree with it. Found by testing: a plain non-interactive `kill` on
# this script left mavros_node and the ground-view streamer running.
set -m

PIDS=()
cleanup() {
    echo "[launch] shutting down ..."
    # Staged SIGINT -> SIGTERM -> SIGKILL, each to the process GROUP (-$pid).
    # Found by testing: rclpy-based nodes here (ground_view_stream.py,
    # fusion_live_node.py, ...) only act on SIGINT -- it's what maps to
    # Python's KeyboardInterrupt, which is what their `except KeyboardInterrupt`
    # shutdown paths actually catch; a bare SIGTERM left them running
    # indefinitely. Plain (non-ROS) processes like mavros_node/usb_camera_node
    # do stop on SIGTERM, so this covers both without needing to know which
    # is which. SIGKILL is a backstop for anything still alive after both.
    for pid in "${PIDS[@]}"; do
        kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
    done
    sleep 2
    for pid in "${PIDS[@]}"; do
        kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    done
    sleep 2
    for pid in "${PIDS[@]}"; do
        kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT INT TERM

# 1. MAVROS
echo "[launch] Starting MAVROS ..."
bash "$CONTROL_DIR/launch_mavros_real.sh" &
PIDS+=("$!")
sleep 6

# 2. Camera driver at the resolution the calibration/map were built at
#    (2048x1536) -- NOT launch_camera.sh's 1280x960 default. See the plan's
#    "Camera resolution" note: functionally any 4:3 res works (K_GSD scales
#    with width), but SuperPoint keypoint yield at 1280x960 is unmeasured, so
#    this stays at native res for the shadow-mode test.
CAMERA_DEV="${CAMERA_DEV:-}"
if [ -z "$CAMERA_DEV" ]; then
    CAMERA_DEV=$(v4l2-ctl --list-devices 2>/dev/null | awk '
        /APPROPHO|IMX900/ { found=1; next }
        found && /\/dev\/video/ { print $1; exit }
        /^[^[:space:]]/ { found=0 }
    ')
fi
if [ -z "$CAMERA_DEV" ]; then
    echo "[launch] WARNING: could not auto-detect the AP-IMX900 -- falling back to /dev/video0" >&2
    CAMERA_DEV=/dev/video0
fi
echo "[launch] Starting camera driver on $CAMERA_DEV at 2048x1536@30 ..."
python3 "$CONTROL_DIR/usb_camera_node.py" --device "$CAMERA_DEV" \
    --width 2048 --height 1536 --fps 30 &
PIDS+=("$!")
sleep 3

# 3. Ground view stream -- only if requested. Subscribes /drone/camera/image_raw
#    (already up) + /drone/pose + /drone/agl (from hw_bridge, started next --
#    subscribers just see nothing until hw_bridge is up, same as
#    launch_real_hw.sh's ordering). Overlays fusion_live_node's
#    vio_vpe/latest_estimate.json once that node starts (step 8) -- see
#    tools/ground_view_stream.py's _read_shadow_estimate(). Mode C (OpenHD)
#    runs alongside mode A/B, not instead of it -- both passed in one process.
STREAM_ARGS=(--fps "$STREAM_FPS")
[[ -n "$STREAM_HOST"   ]] && STREAM_ARGS+=(--host "$STREAM_HOST")
[[ -n "$STREAM_SERVER" ]] && STREAM_ARGS+=(--stream-server "$STREAM_SERVER")
[[ -n "$OPENHD_IP"     ]] && STREAM_ARGS+=(--stream-openhd "$OPENHD_IP")
if [[ -n "$STREAM_HOST$STREAM_SERVER$OPENHD_IP" ]]; then
    echo "[launch] Starting ground view streamer (${STREAM_ARGS[*]}) ..."
    python3 -u "$PROJECT_DIR/tools/ground_view_stream.py" "${STREAM_ARGS[@]}" &
    PIDS+=("$!")
fi

# 4. Hardware bridge (for /drone/agl; /mavros/global_position/* used directly
#    by fusion_live_node/vpe_live_node for GPS ground truth and heading)
echo "[launch] Starting hardware bridge ..."
python3 "$CONTROL_DIR/hw_bridge.py" &
PIDS+=("$!")
sleep 2

# 5. Live VIO (C++, links the OpenVINS build at ~/openvins_ws directly)
VIO_BIN="$SCRIPT_DIR/vio/build/run_video_msckf_live"
if [ ! -x "$VIO_BIN" ]; then
    echo "[launch] ERROR: $VIO_BIN not built. Run:" >&2
    echo "    cd $SCRIPT_DIR/vio && cmake -B build -S . && cmake --build build -j" >&2
    exit 1
fi
echo "[launch] Starting live VIO node ..."
"$VIO_BIN" "$SCRIPT_DIR/vio/config_imx900/estimator_config.yaml" &
PIDS+=("$!")
sleep 2

# 6. Live VPE (Python, needs the isolated venv -- see README on why: NVIDIA
#    Jetson torch build + LightGlue, never pip installed into system Python)
echo "[launch] Starting live VPE node ..."
/home/jetson/venv/vio_vpe/bin/python3 -u "$SCRIPT_DIR/vpe_live_node.py" \
    --map "$MAP_PATH" --sigma-vpe "$SIGMA_VPE" &
PIDS+=("$!")
sleep 2

# 7. YOLO detector (headless) -- same node/venv/flags as control/launch_real_hw.sh.
#    Independent of everything else here (publishes /yolo/detections, consumed
#    by tools/ground_view_stream.py's top-left panel and by nothing in the
#    VIO/VPE/fusion path) -- purely for the ground crew's situational
#    awareness, not part of the localization pipeline.
if [[ -n "$YOLO_ENABLED" ]]; then
    echo "[launch] Starting YOLO detector ..."
    /home/jetson/venv/yolo/bin/python3 -u "$PROJECT_DIR/detection/ros2_node.py" --headless &
    PIDS+=("$!")
    sleep 2
fi

# 8. Shadow-mode fusion -- also backgrounded (not a true foreground exec)
#    so that Ctrl+C AND a plain non-interactive `kill` on this script both
#    reach it via the trap above; `wait` on its PID specifically (not `wait`
#    bare) is what makes bash actually run the EXIT trap instead of sitting
#    in an uninterruptible foreground wait. This is also the node that
#    prints the SHADOW MODE banner and owns the never-publish-to-FC invariant.
echo "[launch] Starting shadow-mode fusion node ..."
/home/jetson/venv/vio_vpe/bin/python3 -u "$SCRIPT_DIR/fusion_live_node.py" \
    --sigma-vio "$SIGMA_VIO" &
FUSION_PID=$!
PIDS+=("$FUSION_PID")

wait "$FUSION_PID"
FUSION_EXIT=$?

echo "[launch] fusion_live_node exited ($FUSION_EXIT)"
exit $FUSION_EXIT
