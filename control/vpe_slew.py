"""
Slew limiter for the VPE stream (see instructions/vpe_jump_runaway_diagnosis.md).

Localizer jumps (AnyLoc accepts, re-acquires, restarts) must not reach EKF3 as
position steps: a fused step becomes a position error *and* a phantom velocity
spike, and the controller fights both at full lean angle regardless of
WPNAV_SPEED.  Instead of publishing the localizer estimate directly, the
commander feeds it through this limiter, which moves the *published* point
toward the estimate at

    allowance = target speed + corr_rate      [m/s]

where "target speed" is the estimate's own recent motion (robust median of
its update-to-update rate, jump-sized outliers excluded).  The estimate
tracks real flight motion — VO integrates the camera — so cruise passes
through tick-for-tick and only the excess over real motion, a correction, is
limited to corr_rate (2.5 m/s default).  A 30 m re-acquire glides in over
~12 s.

Two closed-loop landmines this design survived (both found in SITL,
control/test_vpe_slew_sitl.py — do not reintroduce them):
  1. Dead-reckoning the published point with the EKF velocity *vector*
     diverges without bound: published follows EKF velocity, EKF position
     follows published, so any velocity error is self-confirming.
  2. Scaling the allowance with EKF ground *speed* stalls: with no velocity
     source (EK3_SRCx_VELXY=0) EKF speed derives from the published VPE, so
     lag → low EKF speed → smaller allowance → more lag (~60 m behind).
The target's own motion is the only speed reference that is not downstream
of what this limiter publishes.

Pure Python, no ROS imports — control/test_vpe_slew.py replays recorded
flights through it offline.
"""

import math
from collections import deque

VPE_SLEW_CORR_MPS = 2.5    # m/s — correction speed on top of real motion
_DT_MAX           = 0.5    # s  — clamp scheduler stalls: bounds one tick's glide
_SPEED_MAX        = 15.0   # m/s — cap on the target-speed allowance; target
                           # motion faster than this is a jump, not flight
_SPEED_WIN        = 20     # target-change samples in the speed median
_SPEED_HOLD_S     = 1.0    # target static this long → speed decays to 0


class VpeSlewLimiter:
    """Rate-limits the published VPE position toward a moving target."""

    def __init__(self, corr_rate: float = VPE_SLEW_CORR_MPS):
        self._corr = corr_rate
        self._e = self._n = self._t = None
        self._tgt = None            # (e, n, t) of last *distinct* target
        self._rates = deque(maxlen=_SPEED_WIN)

    def reset(self, east: float, north: float, now: float):
        """Snap the published point (Phase 1: position is ground truth)."""
        self._e, self._n, self._t = east, north, now

    def distance_to(self, tgt_e: float, tgt_n: float) -> float:
        """Metres from the currently published point to a target (0 if unset)."""
        if self._e is None:
            return 0.0
        return math.hypot(tgt_e - self._e, tgt_n - self._n)

    def _target_speed(self, tgt_e: float, tgt_n: float, now: float) -> float:
        """Robust recent speed of the target itself (jumps excluded)."""
        if self._tgt is None:
            self._tgt = (tgt_e, tgt_n, now)
            return 0.0
        le, ln, lt = self._tgt
        moved = math.hypot(tgt_e - le, tgt_n - ln)
        if moved > 1e-9:
            dt = now - lt
            if dt > 1e-3:
                rate = moved / dt
                if rate <= _SPEED_MAX:      # faster = jump → not flight motion
                    self._rates.append(rate)
            self._tgt = (tgt_e, tgt_n, now)
        elif now - lt > _SPEED_HOLD_S:      # target static: decay toward 0
            self._rates.append(0.0)
            self._tgt = (le, ln, now)
        if not self._rates:
            return 0.0
        return sorted(self._rates)[len(self._rates) // 2]

    def update(self, tgt_e: float, tgt_n: float, now: float):
        """Advance one tick toward (tgt_e, tgt_n); returns published (e, n)."""
        speed = self._target_speed(tgt_e, tgt_n, now)
        if self._e is None:
            self.reset(tgt_e, tgt_n, now)
            return self._e, self._n
        dt = min(max(now - self._t, 0.0), _DT_MAX)
        self._t = now
        allow = (speed + self._corr) * dt
        de, dn = tgt_e - self._e, tgt_n - self._n
        dist = math.hypot(de, dn)
        if dist <= allow:
            self._e, self._n = tgt_e, tgt_n
        else:
            self._e += de / dist * allow
            self._n += dn / dist * allow
        return self._e, self._n
