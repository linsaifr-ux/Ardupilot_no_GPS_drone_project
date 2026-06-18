#!/bin/bash
# run.sh — Autonomous drone mission launcher.
#
# ArduPilot mode (default):
#   bash run.sh --tmux           — launch all services in tmux (existing EEPROM)
#   bash run.sh --tmux --wipe    — wipe EEPROM and reload params (first run)
#
# PX4 mode (with Isaac Sim):
#   bash run.sh --tmux --px4                        — full PX4 pipeline
#   bash run.sh --tmux --px4 --params               — + apply params (first run)
#   bash run.sh --tmux --px4 --wipe                 — wipe parameters.bson before starting
#   bash run.sh --tmux --px4 --anyloc               — + AnyLoc localizer (window 4)
#   bash run.sh --tmux --px4 --anyloc --detection   — + YOLO detection (window 5)
#
# PX4 headless mode (no Isaac Sim — kinematic physics only):
#   bash run.sh --tmux --px4 --headless          — headless bridge (drone_sim.py)
#   bash run.sh --tmux --px4 --headless --params — headless + apply params (first run)
#   bash run.sh --tmux --px4 --headless --wipe   — headless + wipe params
#   (--anyloc and --detection require Isaac Sim for camera frames; ignored in headless)
#
# PX4 Isaac Sim no-window mode (full camera/AnyLoc/YOLO, no display — faster rep.step):
#   bash run.sh --tmux --px4 --no-window                      — Isaac Sim headless
#   bash run.sh --tmux --px4 --no-window --anyloc             — + AnyLoc
#   bash run.sh --tmux --px4 --no-window --anyloc --detection — + YOLO detection
#
# Manual 3-terminal (ArduPilot):
#   bash control/launch_sitl.sh --wipe   # first run → type 'reboot' in MAVProxy
#   bash control/launch_sitl.sh          # subsequent runs
#   bash control/launch_mavros.sh
#   bash control/launch_commander.sh
#
# Manual 4-terminal (PX4):
#   bash simulator/run_chiayi.sh --px4   # Isaac Sim + bridge (TCP 4560) — first
#   bash control/launch_px4_sitl.sh
#   bash control/apply_px4_params.sh     # first run only
#   bash control/launch_mavros_px4.sh
#   bash control/launch_commander_px4.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

print_usage() {
    echo ""
    echo "Usage:"
    echo "  bash run.sh                                              — print this help"
    echo "  bash run.sh --tmux                                     — ArduPilot pipeline in tmux"
    echo "  bash run.sh --tmux --wipe                              — ArduPilot pipeline, wipe EEPROM"
    echo "  bash run.sh --tmux --px4                               — PX4 + Isaac Sim pipeline"
    echo "  bash run.sh --tmux --px4 --params                      — PX4 pipeline + apply params (first run)"
    echo "  bash run.sh --tmux --px4 --wipe                        — PX4 pipeline, wipe parameters.bson"
    echo "  bash run.sh --tmux --px4 --anyloc                      — PX4 + Isaac Sim + AnyLoc localizer"
    echo "  bash run.sh --tmux --px4 --anyloc --detection          — + YOLO detection"
    echo "  bash run.sh --tmux --px4 --headless                    — PX4 headless (no Isaac Sim)"
    echo "  bash run.sh --tmux --px4 --headless --params           — PX4 headless + apply params"
    echo "  bash run.sh --tmux --px4 --no-window                           — Isaac Sim headless (no display, full camera)"
    echo "  bash run.sh --tmux --px4 --no-window --anyloc                  — + AnyLoc"
    echo "  bash run.sh --tmux --px4 --no-window --anyloc --detection      — + YOLO detection"
    echo "  bash run.sh --tmux --px4 --no-window --rasterize               — headless + rasterization renderer (~20fps)"
    echo "  bash run.sh --tmux --px4 --rasterize                           — rasterization renderer (windowed)"
    echo ""
}

# ── Parse flags ────────────────────────────────────────────────────────────────
TMUX_MODE=0; USE_PX4=0; WIPE=""; PARAMS=""; HEADLESS=0; NO_WINDOW=0; RASTERIZE=0; ANYLOC=0; DETECTION=0
for arg in "$@"; do
    case "$arg" in
        --tmux)       TMUX_MODE=1 ;;
        --px4)        USE_PX4=1 ;;
        --wipe)       WIPE="--wipe" ;;
        --params)     PARAMS=1 ;;
        --headless)   HEADLESS=1 ;;
        --no-window)  NO_WINDOW=1 ;;
        --rasterize)  RASTERIZE=1 ;;
        --anyloc)     ANYLOC=1 ;;
        --detection)  DETECTION=1 ;;
        --help|-h)    print_usage; exit 0 ;;
    esac
done

# ── tmux mode ──────────────────────────────────────────────────────────────────
if [[ "$TMUX_MODE" == "1" ]]; then
    if ! command -v tmux &>/dev/null; then
        echo "[run.sh] tmux not found.  sudo apt install tmux"
        exit 1
    fi

    # ════════════════════════════════════════════════════════════════════════════
    # PX4 pipeline
    # ════════════════════════════════════════════════════════════════════════════
    if [[ "$USE_PX4" == "1" ]]; then
        SESSION="drone_px4"
        MAVROS_LOG="/tmp/drone_mavros_px4_$$.log"

        echo "[run.sh] Cleaning up old PX4 processes..."
        tmux kill-session -t "$SESSION" 2>/dev/null || true
        pkill -9 -f '/px4 |bin/px4$|mavros_node|px4_commander' 2>/dev/null || true
        sleep 2

        # ── Window 0: bridge (TCP 4560) — headless drone_sim or Isaac Sim ────
        if [[ "$HEADLESS" == "1" ]]; then
            echo "[run.sh] Starting headless bridge (PX4_SIM=1 drone_sim.py)..."
            tmux new-session -d -s "$SESSION" -x 220 -y 50
            tmux rename-window -t "$SESSION:0" "Bridge"
            tmux send-keys -t "$SESSION:0" \
                "source /opt/ros/jazzy/setup.bash && cd '$SCRIPT_DIR' && PX4_SIM=1 python3 control/drone_sim.py; exec bash" Enter

            echo -n "[run.sh] Waiting for headless bridge (TCP 4560)"
            WAITED=0
            while ! ss -tlnp 2>/dev/null | grep -q ":4560 "; do
                sleep 1; WAITED=$((WAITED+1)); echo -n "."
                if [[ $WAITED -ge 30 ]]; then
                    echo ""
                    echo "[run.sh] ERROR: TCP 4560 not ready after 30 s — check Bridge window."
                    tmux attach-session -t "$SESSION"; exit 1
                fi
            done
            echo " done."
        else
            NOWIN_ARG="";    [[ "$NO_WINDOW"  == "1" ]] && NOWIN_ARG="--no-window"
            RASTER_ARG="";   [[ "$RASTERIZE"  == "1" ]] && RASTER_ARG="--rasterize"
            WIN_LABEL="Isaac"; [[ "$NO_WINDOW" == "1" ]] && WIN_LABEL="Isaac(nw)"
            [[ "$RASTERIZE" == "1" ]] && WIN_LABEL="${WIN_LABEL}(r)"
            echo "[run.sh] Starting Isaac Sim (PX4_SIM=1${NO_WINDOW:+, no window}${RASTERIZE:+, rasterize}) — may take ~2 min to load..."
            tmux new-session -d -s "$SESSION" -x 220 -y 50
            tmux rename-window -t "$SESSION:0" "$WIN_LABEL"
            tmux send-keys -t "$SESSION:0" \
                "bash '$SCRIPT_DIR/simulator/run_chiayi.sh' --px4 $NOWIN_ARG $RASTER_ARG; exec bash" Enter

            echo -n "[run.sh] Waiting for Isaac Sim bridge (TCP 4560)"
            WAITED=0
            while ! ss -tlnp 2>/dev/null | grep -q ":4560 "; do
                sleep 3; WAITED=$((WAITED+3)); echo -n "."
                if [[ $WAITED -ge 300 ]]; then
                    echo ""
                    echo "[run.sh] ERROR: TCP 4560 not ready after 300 s — check Isaac window."
                    tmux attach-session -t "$SESSION"; exit 1
                fi
            done
            echo " done."
        fi

        # ── Window 1: PX4 SITL ──────────────────────────────────────────────
        echo "[run.sh] Starting PX4 SITL..."
        tmux new-window -t "$SESSION"
        tmux rename-window -t "$SESSION:1" "PX4"
        tmux send-keys -t "$SESSION:1" \
            "bash '$SCRIPT_DIR/control/launch_px4_sitl.sh' $WIPE; exec bash" Enter

        # Wait for UDP 14580 (PX4 MAVLink ready)
        echo -n "[run.sh] Waiting for PX4 SITL (UDP 14580)"
        WAITED=0
        while ! ss -ulnp 2>/dev/null | grep -q ":14580 "; do
            sleep 1; WAITED=$((WAITED+1)); echo -n "."
            if [[ $WAITED -ge 30 ]]; then
                echo ""
                echo "[run.sh] ERROR: UDP 14580 not ready — check PX4 window."
                tmux attach-session -t "$SESSION"; exit 1
            fi
        done
        echo " done."

        # ── Apply params (first run / --params flag) ─────────────────────────
        if [[ -n "$PARAMS" ]]; then
            echo "[run.sh] Applying PX4 params (will reboot PX4)..."
            bash "$SCRIPT_DIR/control/apply_px4_params.sh"
        fi

        # ── Window 2: MAVROS ─────────────────────────────────────────────────
        echo "[run.sh] Starting MAVROS..."
        tmux new-window -t "$SESSION"
        tmux rename-window -t "$SESSION:2" "MAVROS"
        tmux send-keys -t "$SESSION:2" \
            "bash '$SCRIPT_DIR/control/launch_mavros_px4.sh' 2>&1 | tee '$MAVROS_LOG'; exec bash" \
            Enter

        # Wait for MAVROS heartbeat
        echo -n "[run.sh] Waiting for MAVROS heartbeat"
        WAITED=0
        while ! grep -q "Got HEARTBEAT, connected" "$MAVROS_LOG" 2>/dev/null; do
            sleep 2; WAITED=$((WAITED+2)); echo -n "."
            if [[ $WAITED -ge 60 ]]; then
                echo ""
                echo "[run.sh] ERROR: MAVROS heartbeat not received — check MAVROS window."
                tmux attach-session -t "$SESSION"; exit 1
            fi
        done
        echo " done."
        sleep 2

        # ── Window 3: Commander ──────────────────────────────────────────────
        echo "[run.sh] Starting PX4 commander..."
        tmux new-window -t "$SESSION"
        tmux rename-window -t "$SESSION:3" "Commander"
        CMD_LOG="/tmp/drone_commander_px4_$$.log"
        tmux send-keys -t "$SESSION:3" \
            "bash '$SCRIPT_DIR/control/launch_commander_px4.sh' 2>&1 | tee '$CMD_LOG'; exec bash" Enter

        # ── Window 4: AnyLoc localizer (Isaac Sim only — needs camera) ───────
        WIN_LABELS="0/1/2/3 = Isaac/PX4/MAVROS/Commander"
        if [[ "$ANYLOC" == "1" && "$HEADLESS" == "0" ]]; then
            echo "[run.sh] Starting AnyLoc localizer (startup ~20 min — loading VLAD database)..."
            tmux new-window -t "$SESSION"
            tmux rename-window -t "$SESSION:4" "AnyLoc"
            tmux send-keys -t "$SESSION:4" \
                "bash '$SCRIPT_DIR/anyloc/run_ros2_localizer.sh'; exec bash" Enter
            WIN_LABELS="$WIN_LABELS · 4=AnyLoc"

            # ── Window 5: YOLO detection ─────────────────────────────────────
            if [[ "$DETECTION" == "1" ]]; then
                echo "[run.sh] Starting YOLO detection..."
                tmux new-window -t "$SESSION"
                tmux rename-window -t "$SESSION:5" "Detection"
                tmux send-keys -t "$SESSION:5" \
                    "bash '$SCRIPT_DIR/detection/run_ros2_detector.sh'; exec bash" Enter
                WIN_LABELS="$WIN_LABELS · 5=Detection"
            fi
        elif [[ "$ANYLOC" == "1" && "$HEADLESS" == "1" ]]; then
            echo "[run.sh] WARNING: --anyloc ignored in headless mode (no camera frames)"
        fi

        tmux select-window -t "$SESSION:0"
        echo "[run.sh] Commander log: $CMD_LOG"
        echo "[run.sh] PX4 pipeline running. Attaching tmux (Ctrl-B $WIN_LABELS)"
        if [ -t 1 ]; then
            tmux attach-session -t "$SESSION"
        else
            echo "[run.sh] Not a TTY — connect manually:  tmux attach -t $SESSION"
        fi
        exit 0
    fi

    # ════════════════════════════════════════════════════════════════════════════
    # ArduPilot pipeline (original)
    # ════════════════════════════════════════════════════════════════════════════
    SESSION="drone_mission"
    SITL_LOG="/tmp/drone_sitl_$$.log"
    MAVROS_LOG="/tmp/drone_mavros_$$.log"

    echo "[run.sh] Cleaning up old processes..."
    tmux kill-session -t "$SESSION" 2>/dev/null || true
    pkill -9 -f 'arducopter|mavproxy|mavros_node|flight_commander' 2>/dev/null || true
    sleep 2

    # ── Window 0: SITL ──────────────────────────────────────────────────────
    echo "[run.sh] Starting SITL${WIPE:+ (--wipe)}..."
    tmux new-session -d -s "$SESSION" -x 220 -y 50
    tmux rename-window -t "$SESSION:0" "SITL"
    tmux send-keys -t "$SESSION:0" \
        "bash '$SCRIPT_DIR/control/launch_sitl.sh' $WIPE 2>&1 | tee '$SITL_LOG'; exec bash" \
        Enter

    echo -n "[run.sh] Waiting for SITL params to load"
    WAITED=0
    while ! grep -q "Saved 1346 parameters" "$SITL_LOG" 2>/dev/null; do
        sleep 2; WAITED=$((WAITED+2)); echo -n "."
        if [[ $WAITED -ge 120 ]]; then
            echo ""
            echo "[run.sh] ERROR: Params not loaded after 120 s — check the SITL window."
            tmux attach-session -t "$SESSION"; exit 1
        fi
    done
    echo " done."

    if [[ -n "$WIPE" ]]; then
        echo "[run.sh] Sending 'reboot' to MAVProxy to persist params..."
        tmux send-keys -t "$SESSION:0" "reboot" Enter
        echo -n "[run.sh] Waiting for SITL reboot"
        WAITED=0
        while true; do
            sleep 2; WAITED=$((WAITED+2)); echo -n "."
            COUNT=$(grep -c "ArduPilot Ready" "$SITL_LOG" 2>/dev/null || echo 0)
            [[ "$COUNT" -ge 2 ]] && break
            if [[ $WAITED -ge 90 ]]; then
                echo ""
                echo "[run.sh] ERROR: SITL didn't come back after reboot — check the SITL window."
                tmux attach-session -t "$SESSION" 2>/dev/null || true; exit 1
            fi
        done
        echo " done."
    fi
    sleep 2

    # ── Window 1: MAVROS2 ────────────────────────────────────────────────────
    echo "[run.sh] Starting MAVROS2..."
    tmux new-window -t "$SESSION"
    tmux rename-window -t "$SESSION:1" "MAVROS2"
    tmux send-keys -t "$SESSION:1" \
        "bash '$SCRIPT_DIR/control/launch_mavros.sh' 2>&1 | tee '$MAVROS_LOG'; exec bash" \
        Enter

    echo -n "[run.sh] Waiting for MAVROS2 heartbeat"
    WAITED=0
    while ! grep -q "Got HEARTBEAT, connected" "$MAVROS_LOG" 2>/dev/null; do
        sleep 2; WAITED=$((WAITED+2)); echo -n "."
        if [[ $WAITED -ge 60 ]]; then
            echo ""
            echo "[run.sh] ERROR: MAVROS2 heartbeat not received — check the MAVROS2 window."
            tmux attach-session -t "$SESSION"; exit 1
        fi
    done
    echo " done."
    sleep 2

    # ── Window 2: Flight Commander ───────────────────────────────────────────
    echo "[run.sh] Starting flight commander..."
    tmux new-window -t "$SESSION"
    tmux rename-window -t "$SESSION:2" "Commander"
    tmux send-keys -t "$SESSION:2" \
        "bash '$SCRIPT_DIR/control/launch_commander.sh'; exec bash" Enter

    tmux select-window -t "$SESSION:0"
    echo "[run.sh] All services running. Attaching tmux (Ctrl-B 0/1/2 = SITL/MAVROS/Commander)"
    if [ -t 1 ]; then
        tmux attach-session -t "$SESSION"
    else
        echo "[run.sh] Not a TTY — tmux session '$SESSION' is running in background."
        echo "[run.sh] Connect manually with:  tmux attach -t $SESSION"
    fi
    exit 0
fi

# ── print instructions mode (default) ─────────────────────────────────────────
echo ""
echo "════════════════════════════════════════════════════════"
echo "  Drone Mission — Launch Sequence"
echo "════════════════════════════════════════════════════════"
echo ""
echo "  ArduPilot (tmux):         bash run.sh --tmux [--wipe]"
echo "  PX4 (tmux):               bash run.sh --tmux --px4 [--params] [--wipe]"
echo "  PX4 + AnyLoc:             bash run.sh --tmux --px4 --anyloc"
echo "  PX4 + AnyLoc + Detection: bash run.sh --tmux --px4 --anyloc --detection"
echo "  PX4 headless (tmux):      bash run.sh --tmux --px4 --headless [--params]
  PX4 no-window (tmux):    bash run.sh --tmux --px4 --no-window [--anyloc] [--detection]"
echo ""
echo "  Manual — ArduPilot:"
echo "    [T1]  bash control/launch_sitl.sh --wipe    # first run"
echo "          bash control/launch_sitl.sh            # subsequent runs"
echo "    [T2]  bash control/launch_mavros.sh"
echo "    [T3]  bash control/launch_commander.sh"
echo ""
echo "  Manual — PX4 headless:"
echo "    [T1]  PX4_SIM=1 python3 control/drone_sim.py"
echo "    [T2]  bash control/launch_px4_sitl.sh"
echo "          bash control/apply_px4_params.sh       # first run only"
echo "    [T3]  bash control/launch_mavros_px4.sh"
echo "    [T4]  bash control/launch_commander_px4.sh"
echo ""
echo "  Manual — PX4 (Isaac Sim):"
echo "    [T1]  bash simulator/run_chiayi.sh --px4"
echo "    [T2]  bash control/launch_px4_sitl.sh"
echo "          bash control/apply_px4_params.sh       # first run only"
echo "    [T3]  bash control/launch_mavros_px4.sh"
echo "    [T4]  bash control/launch_commander_px4.sh"
echo ""
echo "════════════════════════════════════════════════════════"
print_usage
