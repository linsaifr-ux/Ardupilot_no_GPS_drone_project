# simulator/ — Isaac Sim Physics Engine + Visualiser

`cesium_scene.py` is the **physics engine and visualiser** for the no-GPS drone project. It runs a 100 Hz background physics thread with a 6-DOF kinematic model and the autopilot SITL bridge, and publishes `/drone/state` for the flight commander and AnyLoc nodes. The render loop (~13 Hz) reads the physics state and updates the drone mesh in Cesium.

**`drone_sim.py` is not used when Isaac Sim is running.** Both publish `/drone/state` and own a bridge port — run only one at a time.

**Start `cesium_scene.py` before the autopilot SITL.** The bridge must be listening before SITL/PX4 connects, or it exits immediately.

---

## Launch

```bash
# ArduPilot (default)
cd simulator && ./run_chiayi.sh

# PX4
cd simulator && ./run_chiayi.sh --px4
```

Or via the top-level launcher:
```bash
bash run.sh --tmux --isaac                  # ArduPilot with Isaac Sim (--isaac flag)
bash run.sh --tmux --px4                    # PX4 (windowed)
bash run.sh --tmux --px4 --no-window        # PX4 headless (no display window, full camera)
bash run.sh --tmux --px4 --no-window --rasterize  # + FullRasterization renderer (experimental)
```

> ArduPilot's `run.sh --tmux` (without `--isaac`) uses `drone_sim.py` instead of Isaac Sim for fast headless testing. Pass `--isaac` to launch `run_chiayi.sh` (no `--px4` flag) in window 0.

---

## Autopilot Toggle: ArduPilot vs PX4

`cesium_scene.py` honours the `PX4_SIM` environment variable:

| Value | Bridge | Port | Protocol |
|-------|--------|------|----------|
| unset / 0 | `SITLBridge` | UDP 9002 | Binary servo in / JSON FDM out |
| `PX4_SIM=1` | `PX4SimBridge` | TCP 4560 | `HIL_ACTUATOR_CONTROLS` in / `HIL_SENSOR` out |

`run_chiayi.sh --px4` sets `PX4_SIM=1` automatically. The kinematic model below the motor-decode layer is identical for both autopilots.

For fast control-loop iteration without the Isaac Sim render overhead, use the headless `control/drone_sim.py` (also honours `PX4_SIM`).

---

## Architecture Inside cesium_scene.py

```
Background thread (100 Hz)                   Render loop (~5 Hz)
─────────────────────────────                ──────────────────────────────────────
bridge.step() ↔ ArduPilot/PX4 SITL          read _k* state under _kin_lock
kinematic 6-DOF integration                  update drone mesh position/orientation
publish /drone/state (ENU)                   2-axis gimbal: cam = conj(drone)×yaw_only
                                             HUD, rep.orchestrator.step() (~190 ms floor)
                                             _pub_q.put_nowait() → background publish thread
                                             ↓
                              Background publish thread (daemon)
                              ─────────────────────────────────
                              tobytes() + ROS2 serialise
                              publish /drone/camera/image_raw
                              publish /drone/pose, /drone/agl
```

> **Render loop FPS floor:** `rep.orchestrator.step()` has a fixed ~190 ms cost that is invariant to resolution, renderer (RTX/rasterization), headless mode, denoiser settings, and extra `simulation_app.update()` calls. It is internal to the replicator annotator pipeline. The render loop runs at ~5 fps as a result.

### Physics thread (100 Hz steps)

1. Call `bridge.step()` — send kinematic state to autopilot, receive motor commands
2. Decode motor outputs to roll/pitch torque targets:
   - **ArduPilot** (QUAD-X): ch1=FR(NE), ch2=RL(SW), ch3=RR(SE), ch4=FL(NW)
   - **PX4** (none_iris CA_ROTOR): control[0]=FR, [1]=RL, [2]=FL, [3]=RR
3. Integrate attitude:
   - **ArduPilot**: first-order response toward roll/pitch target (τ = 0.15 s)
   - **PX4**: second-order angular rate model — `dω/dt = K_ACCEL·mean_p·diff − K_DAMP·ω`, then `dθ/dt = ω`. This eliminates the 100 Hz motor oscillation the first-order model produces at fast step rates.
4. Compute body-frame accelerations → NED velocity → ENU position.  
   Horizontal thrust sign: `_kbfwd = -thrust * sin(pitch)` — PX4 FRD positive pitch is nose-UP (southward force), so the minus gives stable negative feedback.
5. Apply ground constraint (no movement below terrain elevation; zero horizontal velocity and angular rates on landing)
6. Update shared state (protected by `threading.Lock`)
7. Publish `/drone/state` (ENU `PoseStamped`, frame `local_enu`)
8. Write one row to flight trace CSV at 5 Hz (`simulator/flight_traces/trace_<ts>.csv`)

### Render loop (~13 Hz steps)

1. Read kinematic state from background thread (under `_kin_lock`)
2. Update drone mesh position (`drone_pos_op`) and orientation (`drone_orient_op`)
3. **2-axis gimbal:** `camera_local = conj(drone_quat) × yaw_only_quat` — cancels roll and pitch while preserving yaw, so the camera always looks straight down AND the top of the image follows the drone nose direction
4. HUD update (lat / lon / alt / AGL)
5. Capture nadir frame via Replicator annotator; publish `/drone/camera/image_raw`, `/drone/pose`, `/drone/agl`

### Why 100 Hz matters

The replicator render loop runs at ~5 fps. If the physics + bridge ran in the render loop, the autopilot would see 5 Hz physics replies. At 5 Hz, the altitude PID I-term accumulates too aggressively and the drone oscillates. At 100 Hz the control loop is stable and the drone tracks setpoints correctly.

---

## Kinematic Physics Model

| Parameter | Value | Notes |
|-----------|-------|-------|
| Gravity | 9.81 m/s² | |
| Max tilt | 0.35 rad (~20°) | From motor differential |
| Attitude τ (ArduPilot) | 0.15 s | First-order roll/pitch response |
| K_PITCH_ACCEL (PX4) | 80 rad/s² | Second-order angular acceleration constant |
| K_PITCH_DAMP (PX4) | 12 s⁻¹ | Angular rate damping (time constant ≈ 83 ms) |
| Aerodynamic drag | 0.35 s⁻¹ | Applied to all velocity components |
| Max velocity | 30 m/s | Clamp on NED components |
| Physics rate | 100 Hz | Background thread |
| Hover PWM | 1500 → p_norm=0.5 | Matches `MOT_THST_HOVER=0.5` |

Thrust model: `thrust = mean(p_norm_4) * 2.0 * g` — at hover, mean p_norm = 0.5 → thrust = g → zero vertical acceleration.

---

## ROS2 Topics Published

| Topic | Type | Rate | Content |
|-------|------|------|---------|
| `/drone/state` | `geometry_msgs/PoseStamped` | 100 Hz | ENU position (z = MSL altitude), heading quaternion |
| `/drone/camera/image_raw` | `sensor_msgs/Image` | ~5 Hz | Gimbal-stabilised nadir RGB 1024×768 (optics = AP-IMX900-Mini-USB3-I5: 88°×65.1°, EFL 3.1 mm) |
| `/drone/pose` | `geometry_msgs/PoseStamped` | ~5 Hz | Same as `/drone/state` at render rate |
| `/drone/agl` | `std_msgs/Float64` | ~5 Hz | Altitude above ground level (m) |

Subscribed: `/drone/reset` (`std_msgs/Bool`) — resets drone to origin.

---

## Cesium Scene

- **Terrain**: Cesium World Terrain (asset 1) — 2 km radius around 23.4509°N, 120.2861°E
- **Buildings**: Cesium OSM Buildings (asset 96188)
- **Imagery**: Taiwan NLSC PHOTO2 (zoom 18, ~0.60 m/px effective after MAX_TEX=16384 cap)
- **Home elevation**: loaded from `cesium_terrain_cache/` at startup; written to `control/home_elevation.json`

### Target Vehicles

Three USD sedan models (`make_car()`) are placed inside the YOLO detection zone for
end-to-end testing of the survey detection pipeline:

| Prim | NED north (m) | NED east (m) | Yaw | Strip area |
|------|--------------|-------------|-----|------------|
| `/World/Car_01` | +350 | −700 | 45° (NE) | Strip 3 |
| `/World/Car_02` | +150 | −900 | 270° (W) | Strip 1 |
| `/World/Car_03` | +450 | −1100 | 135° (SE) | Strip 4 |

Each model is a procedural white sedan built from USD `Cube`/`Cylinder` primitives
(body, cabin, hood, trunk, bumpers, windows, headlights, tail-lights, wheels, roof
identification panel). Dimensions approximate a Toyota Altis (4.64 m × 1.775 m × 1.45 m).

Car Z is set by `terrain_elev_at(car_x, car_y)` — a nearest-vertex lookup over all
loaded terrain tile vertices — so each car sits at the true terrain elevation at its
position rather than the home-origin elevation (`centre_elev`).

The terrain cache (`cesium_terrain_cache/`) and tile cache (`cesium_tile_cache/`) are pre-downloaded — no network required at runtime.

---

## Isaac Sim Setup

### Requirements

- NVIDIA GPU (RTX 2080 Ti or better, 8 GB+ VRAM)
- NVIDIA driver 520+ (tested with 580.142)
- Miniconda
- Ubuntu 22.04 / 24.04

### 1. Create conda environment

```bash
conda create -n isaac_sim_test python=3.12 -y
conda run -n isaac_sim_test python -m ensurepip --upgrade
```

### 2. Install Isaac Sim packages

```bash
OMNI_KIT_ACCEPT_EULA=Y conda run -n isaac_sim_test python -m pip install \
    isaacsim==6.0.0.0 \
    isaacsim-app==6.0.0.0 \
    isaacsim-core==6.0.0.0 \
    isaacsim-extscache-kit==6.0.0.0 \
    isaacsim-extscache-kit-sdk==6.0.0.0 \
    isaacsim-extscache-physics==6.0.0.0 \
    isaacsim-robot==6.0.0.0 \
    isaacsim-sensor==6.0.0.0 \
    isaacsim-asset==6.0.0.0 \
    isaacsim-replicator==6.0.0.0 \
    isaacsim-gui==6.0.0.0 \
    isaacsim-utils==6.0.0.0 \
    isaacsim-cortex==6.0.0.0 \
    isaacsim-rl==6.0.0.0 \
    isaacsim-code-editor==6.0.0.0 \
    --extra-index-url https://pypi.nvidia.com
```

> Total download ~20 GB. First run compiles CUDA kernels (~4 min); subsequent starts ~15 s.

### 3. Accept the EULA

```bash
echo "yes" > $(conda run -n isaac_sim_test python -c \
    "import isaacsim; import os; print(os.path.join(os.path.dirname(isaacsim.__file__), 'kit', 'EULA_ACCEPTED'))")
```

### 4. Create missing extension stubs (one-time)

```bash
OMNI_KIT_ACCEPT_EULA=Y conda run -n isaac_sim_test python create_stubs.py

sed -i 's/version = "1.0.0"/version = "2.0.0"/' \
    $(conda run -n isaac_sim_test python -c \
    "import isaacsim, os; print(os.path.join(os.path.dirname(isaacsim.__file__), \
     'extsUser', 'carb.graphics_optional', 'config', 'extension.toml'))")

ISAACSIM=$(conda run -n isaac_sim_test python -c \
    "import isaacsim, os; print(os.path.dirname(isaacsim.__file__))")
for ext in \
    isaacsim.examples.interactive isaacsim.exp.base \
    isaacsim.robot_setup.assembler isaacsim.robot_setup.collision_detector \
    isaacsim.robot_setup.gain_tuner isaacsim.robot_setup.grasp_editor \
    isaacsim.robot_setup.xrdf_editor; do
  mkdir -p "$ISAACSIM/extsUser/$ext/config"
  printf '[package]\nversion = "1.0.0"\ntitle = "%s stub"\n\n[dependencies]\n' "$ext" \
    > "$ISAACSIM/extsUser/$ext/config/extension.toml"
done
```

### Performance tips

```bash
# Set CPU governor to performance for better throughput
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

---

## Known Non-Fatal Warnings

| Warning | Cause | Impact |
|---------|-------|--------|
| `isaacsim.robot.manipulators.examples` failed to load | Stub has no Python code | None — example UI only |
| `omni.platforminfo` CPU core errors | Kernel 6.x topology reporting | None — cosmetic |
| `Unexpected reference count` on shutdown | Normal UsdStage teardown | None |
| BARO/MAG STALE at PX4 startup | Clock-settle transient (first few seconds) | Clears automatically |

---

## Files

```
simulator/
├── cesium_scene.py     # Physics + visualiser: 100 Hz kinematic thread + bridge
├── run_chiayi.sh       # Launch: sources ROS2, runs cesium_scene.py in conda [--px4]
├── drone_frames/       # Latest rendered frame (latest.jpg + latest_meta.json)
├── flight_traces/      # CSV flight traces written at 5 Hz (t_s, east_m, north_m, agl_m, vn_ms, ve_ms)
├── cesium_terrain_cache/  # Pre-downloaded terrain tiles
├── cesium_tile_cache/     # Pre-downloaded 3D building tiles
├── geo_utils.py        # Shared geo helpers (lat/lon ↔ metres, WMTS tile math)
├── test_isaac.py       # Headless test: create World, step 5 times
├── create_stubs.py     # One-time stub creation for missing Isaac extensions
└── city_scene.py       # Standalone GUI demo (not used in flight pipeline)
```
