#!/usr/bin/env python3
"""Horizontal fusion filter sitting BETWEEN the estimators and ArduPilot.

Architecture copied from snktshrma/ngps_flight's `ap_ukf`, which is the piece
this project was missing:

    AnyLoc (absolute, ~0.5 Hz) ---.
    OpenVINS velocity (10-20 Hz) -+--> fusion_filter --> ONE stream --> EKF3
    (IMU handled by ArduPilot)   -'

The point is that the autopilot never sees raw estimator output. Previously
AnyLoc position and VIO velocity were pushed to EKF3 as two independent MAVLink
streams and EKF3 was left to reconcile them; when OpenVINS diverged, that went
straight into flight-critical code and killed SITL with a floating-point
exception. Here there is somewhere to reject bad data first.

Deliberate deviations from ap_ukf:
  * It is a plain KALMAN filter, not unscented. The model is linear
    (constant-velocity dynamics, linear position and velocity observations),
    so sigma points are exactly equivalent to the closed-form KF update and
    only cost time. Naming it UKF would be cargo-culting.
  * Scale plausibility gates on VIO come from this project's own
    scale_corrector.py (speed envelope vs WPNAV_SPEED, scale ratio [0.2, 5.0]),
    which ap_ukf does not have -- it does no scale handling at all, and
    monocular scale error is precisely this project's known failure mode.

State (horizontal only, deliberately -- ArduPilot's EKF already does attitude
and altitude far better than a bolt-on filter should attempt):
    x = [n, e, vn, ve]   metres / metres-per-second, NED

Process noise defaults to 5.0 m/s^2, well above what a multirotor actually
does. That is deliberate: with fixes at ~1 Hz and several metres of noise, a
tight dynamics model makes the filter lag the measurements rather than track
them. Measured on survey45, rmse against GPS: q=0.1 -> 21.3 m, q=1.5 -> 12.5 m,
q=5 -> 9.0 m, q=50 -> 12.1 m.
"""
import math
import threading
from collections import deque

import numpy as np

# --- gates, from survey25/vio_eval/scale_corrector.py -----------------------
V_MAX_MS = 12.0                 # WPNAV_SPEED: physical-plausibility envelope
SCALE_LO, SCALE_HI = 0.2, 5.0   # scale_corrector.py --clamp default
MIN_REF_SPEED = 0.5             # below this a scale ratio is meaningless

# --- ap_ukf-style VPS handling ---------------------------------------------
# 2 DOF Mahalanobis gate (0 disables). Loosened from 9.21 (99%) to 23
# (99.999%) on 2026-08-13: at 99% the gate was rejecting fixes that were
# genuinely worse than average but still far better than the drifting state
# that replaced them, and each rejection run then triggered a reset. Measured
# on survey45: 9.21 -> 12.25 m rmse with 6 rejections and 1 reset; 23 -> 9.17 m
# with none. 23 still catches gross outliers, which is all it should do.
VPS_CHI2_THRESHOLD = 23.0
# How many absolute fixes may be gated out in a row before the STATE, not the
# fixes, is treated as the thing that is wrong. 0 disables the escape hatch and
# restores the old behaviour, in which a diverged odometry source can lock the
# filter out of every correction it is given.
MAX_CONSEC_POS_REJECTS = 3
# Ceiling on how fast the PUBLISHED position may be corrected, in m/s, over and
# above the vehicle's own motion. Accuracy statistics hide this entirely: the
# survey45 track sat at 11 m rmse while containing a 52 m step in one 100 ms
# sample. ArduPilot fuses whatever arrives and the position controller chases
# the implied error, so a large step is a command to accelerate.
#
# Measured trade-off on survey45 (published step per 100 ms vs accuracy):
#
#   limit   max step  implied   rmse   median
#   0.5      0.64 m    6.4 m/s  54.0 m  47.9 m   filter cannot keep up
#   1.0      0.64      6.4      40.9    21.8     still cannot keep up
#   2.0      0.70      7.0      30.0     4.4
#   5.0      1.07     10.7      23.3     3.5     <- knee
#  10.0      1.62     16.2      20.2     3.4
#  none     59.70    597.0      14.6     3.4     UNSAFE
#
# 5 m/s is the knee: the worst published step is 1.07 m per sample, reading as
# 10.7 m/s -- below WPNAV_SPEED (12 m/s), so it never asks the aircraft for
# anything it would not do in normal flight -- while median accuracy is
# essentially unaffected (3.51 m vs 3.37 m unlimited). The rmse cost is
# entirely in the tail: after a large correction the output LAGS instead of
# JUMPING, which is the whole point. Drop to 2-3 m/s to be more conservative.
MAX_CORRECTION_MS = 5.0

# Stop publishing once the newest accepted absolute fix is older than this.
# A filter with no absolute measurement behind it is dead-reckoning on VIO
# velocity, and publishing that as a confident position is worse than
# publishing nothing -- the autopilot cannot tell the difference and has no
# way to fall back. This is the survey33 "538 messages off one bootstrap"
# failure.
#
# Gating on the filter's own covariance was tried FIRST and rejected: on
# survey45 the correlation between pos_sigma and true error is 0.109, and the
# bands are non-monotonic (sigma 0-5 m contains 69.5 m errors while sigma
# 20-50 m tops out at 5.0 m). The covariance simply does not know when the
# estimate is wrong, so age is the honest gate.
#
# 5 s is a policy choice, not a measured optimum: VPE runs at ~1 Hz, so this
# is five consecutive misses. survey45 cannot calibrate it -- that flight ends
# in a hover, so even 30 s of coasting stays under 5 m. The risk it guards
# against is coasting at speed, where VIO velocity error was 35 m/s at 65 m.
MAX_FIX_AGE_S = 5.0

# The constant-velocity model is only worth having if the velocity input is
# real. When it is not, extrapolating on it is strictly worse than not moving
# between fixes -- measured on survey45, where VIO velocity carries 36 m/s of
# error at 65 m AGL:
#
#   zero-order hold (do not extrapolate)     7.7 m rmse
#   best constant-velocity tuning            9.0 m
#   as shipped                              14.6 m
#
# and on survey43, where VIO velocity is good to 0.48 m/s, the opposite:
#
#   zero-order hold                         32.4 m rmse, median 23.1 m
#   constant-velocity with VIO              12.2 m rmse, median  2.1 m
#
# So the model must be chosen from the data, not fixed. The filter's own
# velocity gates already separate the two cases cleanly: survey45 rejects 48%
# of velocity updates, survey43 only 11%. Below this acceptance rate the
# velocity state is held at zero, which degrades the filter to a position
# random walk -- i.e. to the zero-order-hold behaviour that is optimal when
# there is no usable velocity information.
MIN_VEL_ACCEPT_RATE = 0.70
VEL_HEALTH_WINDOW = 50          # velocity updates considered for the rate
# Bleed each absolute fix in over N updates. Now 1, i.e. off: smoothing moved
# to the slew limiter on the PUBLISHED output, which is the right place for it
# -- the internal state should converge as fast as the measurements allow, and
# only the wire needs to be gentle. Measured: soft=10 costs ~1 m rmse on both
# flights versus soft=1 because the state permanently lags the fixes.
VPS_SOFT_FRAMES = 5


class FusionFilter:
    """4-state horizontal KF with gated absolute fixes and soft correction."""

    def __init__(self, accel_process_noise=5.0,
                 chi2_threshold=VPS_CHI2_THRESHOLD,
                 soft_frames=VPS_SOFT_FRAMES,
                 max_consec_rejects=MAX_CONSEC_POS_REJECTS,
                 max_correction_ms=MAX_CORRECTION_MS,
                 max_fix_age_s=MAX_FIX_AGE_S):
        self._lock = threading.Lock()
        self.x = np.zeros(4)
        self.P = np.diag([1e4, 1e4, 1e2, 1e2])   # wide until first fix
        self.q_a = accel_process_noise
        self.chi2 = chi2_threshold
        self.soft_frames = max(1, int(soft_frames))
        self.max_consec_rejects = int(max_consec_rejects)
        self.max_correction_ms = float(max_correction_ms)
        self.max_fix_age_s = float(max_fix_age_s)
        self._clock = 0.0          # accumulated from predict(), for staleness
        self.min_vel_accept_rate = MIN_VEL_ACCEPT_RATE
        self._vel_hist = deque(maxlen=VEL_HEALTH_WINDOW)
        self.n_vel_unhealthy = 0
        self._last_fix_t = None
        self.n_suppressed = 0
        self._consec_rejects = 0
        self._out = None
        self.n_slew_limited = 0
        self.n_resets = 0
        self.initialised = False
        # pending soft-correction state
        self._pend_z = None
        self._pend_R = None
        self._pend_left = 0
        # counters for honest reporting
        self.n_pos_accepted = 0
        self.n_pos_rejected = 0
        self.n_vel_accepted = 0
        self.n_vel_rejected = 0

    # -- prediction ---------------------------------------------------------
    def predict(self, dt):
        if dt <= 0 or dt > 5.0:
            return
        with self._lock:
            self._clock += dt
            F = np.array([[1, 0, dt, 0],
                          [0, 1, 0, dt],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]], dtype=float)
            # piecewise-constant acceleration process noise
            G = np.array([dt * dt / 2, dt * dt / 2, dt, dt])
            Q = np.diag((G * self.q_a) ** 2)
            self.x = F @ self.x
            self.P = F @ self.P @ F.T + Q
            if not self._vel_healthy_locked():
                # No trustworthy velocity: stop extrapolating. Holding the
                # position instead of predicting it forward is what a
                # zero-order hold does, and on survey45 that is worth 6.7 m of
                # rmse over the constant-velocity model.
                self.x[2:] = 0.0
                self.n_vel_unhealthy += 1
            self._apply_pending_locked()

    def _vel_healthy_locked(self):
        """Is the velocity input good enough to extrapolate on?"""
        if len(self._vel_hist) < 10:
            return True          # not enough evidence yet; give it the benefit
        return (sum(self._vel_hist) / len(self._vel_hist)) >= self.min_vel_accept_rate

    def vel_health(self):
        if not self._vel_hist:
            return None
        return sum(self._vel_hist) / len(self._vel_hist)

    # -- absolute position (AnyLoc) ----------------------------------------
    def update_position(self, n, e, sigma_m):
        """Queue an absolute fix. Gated, then bled in over soft_frames steps."""
        z = np.array([float(n), float(e)])
        if not np.isfinite(z).all():
            self.n_pos_rejected += 1
            return False, "non-finite"
        R = np.eye(2) * float(sigma_m) ** 2
        with self._lock:
            if not self.initialised:
                # ap_ukf behaviour: bootstrap from the FIRST absolute fix.
                # Note VIO is never required to bootstrap the system -- that is
                # what makes this architecture immune to the OpenVINS static-init
                # problem (9.5cm camera height => disparity blows past the
                # init gate the instant the aircraft moves).
                self.x[:2] = z
                self.x[2:] = 0.0
                self.P = np.diag([sigma_m ** 2, sigma_m ** 2, 25.0, 25.0])
                self.initialised = True
                self._last_fix_t = self._clock
                self.n_pos_accepted += 1
                return True, "bootstrap"

            H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
            y = z - H @ self.x
            S = H @ self.P @ H.T + R
            if self.chi2 > 0:
                try:
                    d2 = float(y @ np.linalg.solve(S, y))
                except np.linalg.LinAlgError:
                    self.n_pos_rejected += 1
                    return False, "singular S"
                if d2 > self.chi2:
                    self.n_pos_rejected += 1
                    self._consec_rejects += 1
                    # A Mahalanobis gate protects against a bad FIX, but it
                    # cannot tell that from a bad STATE -- and a diverging
                    # odometry source produces a confidently wrong state that
                    # rejects every good fix forever. Measured on survey45:
                    # 35 of 43 correct VPE fixes gated out, and the fused
                    # track ended up 635 m rmse against 6 m for the fixes
                    # alone. So a run of rejections is treated as evidence
                    # against the state, not against the fixes.
                    if 0 < self.max_consec_rejects <= self._consec_rejects:
                        # Do NOT teleport the state to the fix. Doing that
                        # produced a 52 m step in a single 100 ms output on
                        # survey45 -- 520 m/s of apparent velocity, which the
                        # position controller would chase. Instead admit the
                        # state is untrustworthy by inflating its covariance;
                        # the next fix then passes the gate on its own and is
                        # bled in through the normal soft-correction path.
                        self.n_resets += 1
                        self._consec_rejects = 0
                        self.P[0, 0] += (10.0 * float(sigma_m)) ** 2
                        self.P[1, 1] += (10.0 * float(sigma_m)) ** 2
                        self.P[2, 2] += 25.0
                        self.P[3, 3] += 25.0
                        return False, "covariance inflated after repeated gate failures"
                    return False, f"mahalanobis d2={d2:.1f} > {self.chi2}"
            self._consec_rejects = 0
            # accepted: bleed it in rather than jumping (ap_ukf vps_soft_frames).
            # Inflating R by N and applying N times is statistically ~one
            # update at R, but spread over time so the trajectory never snaps.
            self._pend_z = z
            self._pend_R = R * self.soft_frames
            self._pend_left = self.soft_frames
            self._last_fix_t = self._clock
            self.n_pos_accepted += 1
            return True, "accepted"

    def _apply_pending_locked(self):
        if self._pend_left <= 0 or self._pend_z is None:
            return
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
        y = self._pend_z - H @ self.x
        S = H @ self.P @ H.T + self._pend_R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
        self._pend_left -= 1
        if self._pend_left == 0:
            self._pend_z = self._pend_R = None

    # -- VIO velocity -------------------------------------------------------
    def update_velocity(self, vn, ve, sigma_ms, ekf_speed_ref=None):
        """Gated velocity update. Gates are the project's own, not invented:
        physical speed envelope + scale-ratio clamp."""
        v = np.array([float(vn), float(ve)])
        if not np.isfinite(v).all():
            self.n_vel_rejected += 1
            self._vel_hist.append(0)
            return False, "non-finite"
        sp = float(np.linalg.norm(v))
        if sp > V_MAX_MS:
            self.n_vel_rejected += 1
            self._vel_hist.append(0)
            return False, f"|v|={sp:.1f} > WPNAV_SPEED {V_MAX_MS}"
        if ekf_speed_ref is not None and ekf_speed_ref > MIN_REF_SPEED:
            ratio = sp / ekf_speed_ref
            if not (SCALE_LO < ratio < SCALE_HI):
                self.n_vel_rejected += 1
                self._vel_hist.append(0)
                return False, f"scale ratio {ratio:.2f} outside [{SCALE_LO},{SCALE_HI}]"
        with self._lock:
            if not self.initialised:
                self.n_vel_rejected += 1
                self._vel_hist.append(0)
                return False, "not initialised (waiting for first absolute fix)"
            H = np.array([[0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
            R = np.eye(2) * float(sigma_ms) ** 2
            y = v - H @ self.x
            S = H @ self.P @ H.T + R
            K = self.P @ H.T @ np.linalg.inv(S)
            self.x = self.x + K @ y
            self.P = (np.eye(4) - K @ H) @ self.P
            self.n_vel_accepted += 1
            self._vel_hist.append(1)
            return True, "accepted"

    # -- output -------------------------------------------------------------
    def state(self):
        with self._lock:
            return self.x.copy(), self.P.copy()

    def output(self, dt):
        """Slew-limited position for the autopilot.

        The published position is walked toward the filter state at a bounded
        speed. The bound is on TOTAL motion, not on a correction added to a
        feed-forward term: when the velocity source is unhealthy the state's
        velocity is held at zero, so a correction-only budget would leave the
        output with nothing to track the vehicle's real motion and it would sit
        permanently saturated (measured: 45% of samples, and the published path
        visibly cut corners no matter how the other knobs were set).

        V_MAX_MS + max_correction_ms is the physical reading: the aircraft may
        fly at WPNAV_SPEED, and the estimate may close error on top of that.
        """
        with self._lock:
            if not self.initialised:
                return None
            if (self.max_fix_age_s > 0 and self._last_fix_t is not None
                    and self._clock - self._last_fix_t > self.max_fix_age_s):
                # Drop the slew anchor too. If publishing resumes, the
                # autopilot will have treated the gap as loss of the source
                # and re-initialised, so walking slowly from a stale position
                # would be wrong -- re-anchor instead.
                self._out = None
                self.n_suppressed += 1
                return None
            target = self.x[:2].copy()
            if self._out is None:
                self._out = target.copy()
                return self._out.copy()
            err = target - self._out
            d = float(np.linalg.norm(err))
            lim = (V_MAX_MS + self.max_correction_ms) * max(dt, 1e-3)
            if d > lim:
                err *= lim / d
                self.n_slew_limited += 1
            self._out = self._out + err
            return self._out.copy()

    def pos_sigma(self):
        with self._lock:
            return math.sqrt(max(self.P[0, 0], self.P[1, 1]))

    def stats(self):
        return dict(pos_ok=self.n_pos_accepted, pos_rej=self.n_pos_rejected,
                    vel_ok=self.n_vel_accepted, vel_rej=self.n_vel_rejected,
                    resets=self.n_resets, slew_limited=self.n_slew_limited,
                    suppressed=self.n_suppressed,
                    vel_health=self.vel_health(),
                    vel_unhealthy_steps=self.n_vel_unhealthy,
                    initialised=self.initialised)
