#!/bin/bash
# Run the Chiayi scene in Isaac Sim
# Pure Cesium ion 3D Tiles: World Terrain (asset 1) + OSM Buildings (asset 96188)
# Satellite imagery: ESRI World Imagery.  No OSM / SRTM patchwork.
# Centre: 23.450868, 120.286135 — Radius: 2 km

# Source ROS2 Jazzy so Isaac Sim can use system rclpy (Python 3.12 compatible).
# ROS2 env vars (AMENT_PREFIX_PATH, ROS_DISTRO, LD_LIBRARY_PATH) are inherited
# by the conda process, and cesium_scene.py adds the site-packages path to sys.path.
source /opt/ros/humble/setup.bash

cd "$(dirname "$0")"

# --px4: activate PX4 HIL bridge (TCP 4560) instead of ArduPilot JSON (UDP 9002)
USE_PX4=0
FWDARGS=()
for arg in "$@"; do
    [[ "$arg" == "--px4" ]] && USE_PX4=1 || FWDARGS+=("$arg")
done

if [[ "$USE_PX4" == "1" ]]; then
    DISPLAY=:2 OMNI_KIT_ACCEPT_EULA=Y PX4_SIM=1 conda run -n isaac_sim_test --no-capture-output \
        python cesium_scene.py "${FWDARGS[@]}"
else
    DISPLAY=:2 OMNI_KIT_ACCEPT_EULA=Y conda run -n isaac_sim_test --no-capture-output \
        python cesium_scene.py "${FWDARGS[@]}"
fi
