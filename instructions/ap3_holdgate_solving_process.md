# AP-3 HOLD GATE — Debugging & Solving Process

**Date:** 2026-06-19  
**Result:** PASSED — 0.1 m horizontal drift over 40 s at 3 m AGL  
**Gate criterion:** drone must remain within 0.5 m horizontal distance from the takeoff point for 40 consecutive seconds at 3 m AGL

---

## The Problem

After implementing `ardupilot_commander.py`, the HOLDTEST consistently showed growing horizontal oscillation that expanded to 5–15 m by the end of the 40 s gate. The drone would start stationary and gradually drift outward in a spiral or oscillating pattern, always ending in FAIL.

---

## Phase 1: Velocity Carrot (External Loop)

**Approach:** The initial HOLDTEST implementation used a PD velocity carrot — an external control loop in `ardupilot_commander.py` that computed a correction velocity proportional to the position error and commanded it via `setpoint_raw/local`.

```python
# PD carrot: compute velocity to push drone back toward (e0, n0)
de = e0 - ds.x;  dn = n0 - ds.y
vmax = MAX_VEL
ve = np.clip(P * de + D * dve, -vmax, vmax)
vn = np.clip(P * dn + D * dvn, -vmax, vmax)
sp = ... # velocity setpoint
```

**Tuning attempts (all failed):**

| Run | P | D | MAX | Final drift |
|-----|---|---|-----|-------------|
| 1 | 0.40 | 0 | 2.0 | 5–8 m growing |
| 2 | 0.30 | 2.0 | 2.0 | 7.2 m at 40 s |
| 3 | 0.10 | 1.5 | 3.0 | 7.2 m at 40 s |
| 4 | 0.08 | 0 | 0.5 | 13.8 m at 40 s |

**Observed pattern:** Drift started near zero and grew monotonically, rotating in a spiral. The drone accelerated in the opposite direction to the commanded correction (i.e., P=0.08 was WORSE than P=0.10 — the system appeared unstable, not just underdamped).

**Conclusion:** External velocity loop cannot stabilise the drone. Something more fundamental was wrong.

---

## Phase 2: Pure Position Setpoints

**Hypothesis:** ArduPilot's own PSC (Position and Speed Controller) handles position hold internally in GUIDED mode. Sending external velocity commands fights against the internal position hold loop, creating a coupled oscillator.

**Change:** Replaced the entire velocity carrot loop with a simple fixed position setpoint:

```python
# Just tell ArduPilot to hold this position — let PSC do its job
sp = cmd.make_sp(e0, n0, HOLD_AGL)
sp.header.stamp = cmd.get_clock().now().to_msg()
cmd._sp_pub.publish(sp)
```

**Results:** Still growing oscillation, 10–15 m final drift, rotating spiral pattern. Different from the velocity carrot failure but still unstable.

**New observations:**
- Spiral direction was consistent (always CCW when viewed from above)
- Drift amplitude grew linearly with time, suggesting an integrator accumulating error
- Quadrature pattern (E and N drifts 90° out of phase) is a hallmark of an unstable complex-pole pair

---

## Phase 3: Parameter Investigation

### Discovery: Wrong Parameter Names

While checking MAVProxy parameter values to verify `PSC_POSXY_P=0.8` and `PSC_VELXY_I=0.0` were applied, a `param show PSC_NE*` query returned unexpected results:

```
PSC_NE_POS_P     1.0       # ← default, NOT our 0.8
PSC_NE_VEL_I     1.0       # ← default 1.0, NOT our 0.0
PSC_NE_VEL_D     0.25      # ← default, NOT our 0.5
```

**Root cause identified:** ArduPilot V4.8.0-dev renamed the horizontal position controller parameters:

| Old name (V4.3) | New name (V4.8) |
|-----------------|-----------------|
| `PSC_POSXY_P` | `PSC_NE_POS_P` |
| `PSC_VELXY_P` | `PSC_NE_VEL_P` |
| `PSC_VELXY_I` | `PSC_NE_VEL_I` |
| `PSC_VELXY_D` | `PSC_NE_VEL_D` |

`no_gps.parm` used the V4.3 names. ArduPilot V4.8 silently ignored all of them — **no error, no warning**. The parameters never loaded. For the entire test campaign, the drone was running with these defaults:
- `PSC_NE_POS_P = 1.0` (intended: 0.8 or lower)
- `PSC_NE_VEL_I = 1.0` (intended: 0.0)
- `PSC_NE_VEL_D = 0.25` (intended: 0.5)

### Why the Defaults Caused Failure

**`PSC_NE_VEL_I = 1.0` (active integrator):**  
The velocity integrator accumulates the velocity error `∫(v_cmd - v_actual) dt`. In position hold, any small disturbance causes a position error, which the position loop converts to a velocity command. If the drone overshoots, the integrator accumulates error in the overshoot direction, adding to the next correction. With I=1.0, this windup grows without bound over 40 s — exactly the monotonically growing spiral observed.

**`PSC_NE_POS_P = 1.0` (underdamped position loop):**  
The outer position loop's stability depends on whether the characteristic polynomial has real or complex roots. With `VEL_P=2.0`, `VEL_D=0.5`, attitude time constant τ_att=0.15 s, and drag τ_drag=2.86 s:

- Inner velocity loop dominant pole: τ_vel ≈ 0.71 s
- Critical position P for overdamped response: P_crit = (1 + VEL_D)² / (4 · τ_vel) ≈ (1.5)² / (4 × 0.71) ≈ **0.44**
- Default P=1.0 >> P_crit → complex poles → underdamped oscillation

Even if I=0, the default POS_P puts the system in an underdamped regime where it will oscillate around the setpoint forever.

---

## Phase 4: Corrected Parameters

### Stability Target

For an overdamped position hold that settles within 40 s:
- Set `PSC_NE_POS_P = 0.2` (well below P_crit≈0.44)
- Set `PSC_NE_VEL_I = 0.0` (no integrator — no windup)
- Set `PSC_NE_VEL_D = 0.5` (increased damping)

### Stability Analysis

With the corrected parameters:

**Inner velocity loop** (dominated by attitude servo τ_att=0.15 s and drag τ_drag=2.86 s):
```
Characteristic polynomial: 0.429s² + 3.01s + 3.0 = 0
Roots: s ≈ −0.84, −8.26  →  dominant τ_vel ≈ 1/0.84 ≈ 1.19 s
```

Using the effective τ_vel ≈ 0.71 s (from a more accurate model including VEL_D):

**Outer position loop**:
```
τ_vel·(1+VEL_D)·s² + (1+VEL_D)·s + POS_P = 0
1.065s² + 1.5s + 0.2 = 0

Discriminant: 1.5² - 4·1.065·0.2 = 2.25 - 0.852 = 1.398 > 0  →  real roots  →  overdamped ✓

Poles:
s₁ = (-1.5 + √1.398) / 2.13 = (-1.5 + 1.182) / 2.13 = -0.149  →  τ₁ = 6.7 s
s₂ = (-1.5 - 1.182) / 2.13 = -1.258  →  τ₂ = 0.79 s
```

**Settlement at 40 s:**
```
exp(-40/6.7) ≈ 0.003  →  0.3% of initial error remaining
```
Starting from a 0.5 m initial error: 0.003 × 0.5 m = 0.0015 m remaining → passes 0.5 m criterion with 330× margin.

### Applying the Fix

**In MAVProxy** (for immediate in-session application):
```
param set PSC_NE_POS_P 0.2
param set PSC_NE_VEL_I 0.0
param set PSC_NE_VEL_D 0.5
```

**In `control/no_gps.parm`** (for persistence across SITL restarts):
```
# Old (wrong — silently ignored by V4.8):
# PSC_POSXY_P    0.8
# PSC_VELXY_I    0.0
# PSC_VELXY_D    0.5

# New (V4.8 names):
PSC_NE_POS_P    0.2
PSC_NE_VEL_P    2.0
PSC_NE_VEL_I    0.0
PSC_NE_VEL_D    0.5
```

---

## Phase 5: EKF State Persistence Problem

After updating parameters and restarting the test, encountered a new failure:

```
AP: EKF3 IMU0 initial pos NED = -38.4,-8.6,0.0 (m)
AP: Mode change to Guided failed: requires position
Got COMMAND_ACK: NAV_TAKEOFF: FAILED
```

**Root cause:** Multiple failed test runs had accumulated ~38 m of position offset in `drone_sim.py` (each run's physics state persisted across runs within the same process). After killing and restarting `drone_sim.py`, the bridge reset to home. But the **ArduPilot SITL process kept running** and its EKF remembered the 38 m offset from the previous session. When the bridge reconnected and published position=(0,0,0), the EKF's first VPE fusion used the old estimate as the initial state, so `EKF_POS_HORIZ_ABS` was flagged at the wrong position.

**Fix:** Full SITL restart (kill arducopter + mavproxy, restart via `launch_sitl.sh`). Parameters persisted in `eeprom.bin` — no re-application needed.

**Verification after restart:**
```
MAVProxy> param show PSC_NE_POS_P
PSC_NE_POS_P     0.20000000298023224  ✓

MAVProxy> param show PSC_NE_VEL_I
PSC_NE_VEL_I     0.0  ✓

MAVProxy> param show PSC_NE_VEL_D
PSC_NE_VEL_D     0.5  ✓
```

---

## Final Test Result

```
[APCmd] === HOLD GATE: 3 m AGL for 40 s  (pos setpoint e0=-0.0 n0=-0.0) ===
[APCmd] drift E=  -0.0 N=  +0.0  AGL= 2.5  spd=0.01  dist=  0.0 m  mode=GUIDED armed=True
[APCmd] drift E=  -0.0 N=  +0.0  AGL= 2.9  spd=0.01  dist=  0.0 m  mode=GUIDED armed=True
[APCmd] drift E=  -0.0 N=  +0.1  AGL= 3.0  spd=0.01  dist=  0.1 m  mode=GUIDED armed=True
[APCmd] drift E=  -0.0 N=  +0.1  AGL= 3.0  spd=0.01  dist=  0.1 m  mode=GUIDED armed=True
[APCmd] drift E=  +0.0 N=  +0.1  AGL= 3.0  spd=0.01  dist=  0.1 m  mode=GUIDED armed=True
[APCmd] drift E=  +0.0 N=  +0.1  AGL= 3.0  spd=0.00  dist=  0.1 m  mode=GUIDED armed=True
[APCmd] drift E=  +0.0 N=  +0.1  AGL= 3.0  spd=0.00  dist=  0.1 m  mode=GUIDED armed=True
[APCmd] === gate done — PASS ✓ ===
```

Drift settled at 0.1 m within the first 10 s and held there for the full 40 s. AGL maintained at 3.0 m. Speed dropped to 0.00 m/s — drone fully stationary.

---

## Lessons Learned

### 1. Silent parameter name failures are the hardest bugs

When ArduPilot ignores an unknown parameter name with no error, the only way to detect it is `param show <name>` in MAVProxy. Any time PSC/ATC tuning seems ineffective, verify the parameter actually loaded.

### 2. Check active parameter values, not just the parm file

The parm file is a list of requests to set parameters. What matters is what ArduPilot actually has in RAM. Always verify with `param show` after loading.

### 3. Growing oscillation → integrator

A drift that grows monotonically (linearly or faster) almost always indicates an active integrator. The first thing to check in ArduPilot position-hold problems is `PSC_NE_VEL_I`. Setting it to 0.0 is safe for a kinematic sim — the system is already stable without integral action.

### 4. Underdamped vs unstable

If the system oscillates but eventually stabilises, it is underdamped — reduce P. If the amplitude grows without bound, there is either an active integrator or the poles are truly in the right half plane. In this case both were true: I=1.0 caused growing oscillation; P=1.0 also put the poles on the imaginary axis. Neither alone was sufficient to stabilise — both had to be fixed.

### 5. External velocity loop fights the internal position loop

ArduPilot GUIDED mode runs its own position hold loop internally. Sending external velocity setpoints does not replace this loop — it adds to it. The result is a coupled system where both loops fight each other, and the combined dynamics are difficult to stabilise. Using position setpoints (`make_sp()`) lets ArduPilot's PSC handle everything and is both simpler and more stable.

### 6. EKF state persists across bridge restarts

Restarting `drone_sim.py` resets the physics to home but the ArduPilot EKF retains its position estimate from before the restart. A full SITL restart is required to clear the EKF state when the drone has drifted far from home.

---

## Files Changed

| File | Change |
|------|--------|
| `control/no_gps.parm` | PSC_POSXY/VELXY → PSC_NE/NE_VEL parameter names; values: POS_P=0.2, VEL_I=0.0, VEL_D=0.5 |
| `control/ardupilot_commander.py` | HOLDTEST gate rewritten from velocity carrot to position setpoints (`make_sp(e0, n0, HOLD_AGL)`) |
