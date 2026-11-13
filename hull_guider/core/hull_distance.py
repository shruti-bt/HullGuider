"""
Perpendicular distance from the DVL to the hull, from the two top beams.

No ROS imports, so it can be tested on its own (see test/test_hull_distance.py).

Why this exists
---------------
The old compute_distance_2 averaged d_i * cos(22.5 deg) over the valid top
beams. That is only correct when the DVL is square to the hull. When the ROV
is turned, the two beams measure different ranges, and when one of them drops
out the average switches to the other beam alone: a jump of several cm
(about 7 cm at 15 deg, more at larger angles).

What this does
--------------
1. Both top beams valid: the two hit points lie on the hull, so the line
   through them gives the hull angle in the DVL frame (no compass needed).
   The angle is smoothed and stored.
2. Each valid beam gives the true perpendicular distance D = d * (n . u),
   where n is the hull normal and u the beam direction. With a correct
   angle both beams give the same D, so losing one causes no jump.
3. One beam valid: use it with the last stored hull angle.
4. No top beam valid: use the lower beams (`backup_beams`, 0 and 1) with
   the stored hull angle, with a larger measurement noise. In the oct3026
   bags a lower beam was still valid in 8-120 messages per bag while both
   top beams were out.
5. No beam valid at all: predict only; after `timeout` seconds (3 s, longer
   than every dropout in the oct3026 bags) the output is marked invalid
   (instead of the old 0.01 m fallback).
   Readings at or beyond `max_range` (2.9 m: the bags flatten at the 3 m
   range limit) are treated as "no hull".
6. Gate: a measurement more than `gate` metres from the estimate is
   rejected. If rejected readings keep coming for `reset_time` seconds
   (at least `max_rejects` of them) and agree with each other within
   `gate`, the filter restarts from them, so it can never lock on an old
   value, but short reflections (seen for ~0.5-1 s in the oct3026 bags)
   are not accepted.
   If both beams are valid but imply a hull angle beyond `max_angle_deg`,
   they disagree (one is a reflection): only the beam that agrees with the
   current estimate is used.
   DVL velocity check: if the DVL velocity is given, the distance the ROV
   moved along the hull normal since the last accepted reading is summed.
   A rejected reading that matches that movement (within `gate`, and the
   ROV moved at least `gate`) is a real change and is accepted at once,
   without waiting `reset_time`.
7. A small constant-velocity Kalman filter (distance, rate) smooths it.

Assumption: the two top beams share the same DVL x component, so they only
see the hull angle about the DVL x axis (the ROV yaw relative to the hull).
Tilt about the other axis is assumed to be zero.
"""
import math

import numpy as np


def default_beam_dirs():
    """Unit beam directions in the DVL frame (same model as dvl_hull_node)."""
    a = math.radians(22.5)
    c = math.cos(a)
    s = math.sin(a) / math.sqrt(2)
    return np.array([
        [-s, s, c],
        [-s, -s, c],
        [s, -s, c],
        [s, s, c],
    ])


class HullDistanceEstimator:

    def __init__(self, beam_dirs=None, beam_pair=(2, 3),
                 max_angle_deg=35.0, angle_alpha=0.3,
                 gate=0.15, max_rejects=3, reset_time=1.5,
                 accel_std=0.1, meas_std_two=0.02, meas_std_one=0.04,
                 timeout=3.0, min_range=0.05, max_range=2.9, rate_hold=0.5,
                 backup_beams=(0, 1), meas_std_backup=0.06):
        self.u = np.asarray(beam_dirs if beam_dirs is not None else default_beam_dirs(), float)
        self.pair = tuple(beam_pair)
        self.backup = tuple(b for b in (backup_beams or ()) if b not in self.pair)
        self.meas_std_backup = meas_std_backup
        self.max_angle = math.radians(max_angle_deg)
        self.angle_alpha = angle_alpha
        self.gate = gate
        self.max_rejects = max_rejects
        self.reset_time = reset_time
        self.accel_std = accel_std
        self.meas_std_two = meas_std_two
        self.meas_std_one = meas_std_one
        self.timeout = timeout
        self.rate_hold = rate_hold
        self.min_range = min_range
        self.max_range = max_range
        self.reset()

    # ------------------------------------------------------------------
    def reset(self):
        self.angle = None          # hull angle about DVL x (rad), smoothed
        self.x = None              # Kalman state [D, D_dot]
        self.P = None
        self.rejects = 0
        self.rejected = []         # (time, value) of consecutive rejected readings
        self.clock = 0.0
        self.time_since_meas = float("inf")
        self.moved = 0.0           # distance change from DVL velocity since the last accepted reading
        self.moved_ok = False      # velocity was available for all of that time
        self.last_accepted = None
        self.last = {}             # diagnostics of the last update

    @property
    def normal(self):
        """Hull normal in the DVL frame (pointing from DVL to hull)."""
        a = self.angle or 0.0
        return np.array([0.0, math.sin(a), math.cos(a)])

    # ------------------------------------------------------------------
    def _valid_ranges(self, ranges, valid, beams=None):
        out = {}
        for i in (self.pair if beams is None else beams):
            d = float(ranges[i])
            if valid[i] and self.min_range < d < self.max_range and math.isfinite(d):
                out[i] = d
        return out

    def _angle_from_two(self, i, j, di, dj):
        """Hull angle from two hit points, in the DVL y-z plane."""
        pi = di * self.u[i]
        pj = dj * self.u[j]
        t = pj[1:] - pi[1:]                  # direction along the hull (y, z)
        n = np.array([-t[1], t[0]])          # perpendicular
        if n[1] < 0:                          # point towards the hull (+z)
            n = -n
        return math.atan2(n[0], n[1])

    def measure(self, ranges, valid):
        """Raw perpendicular distance from one DVL message.

        Returns (D, n_beams) or (None, 0). n_beams is 2 or 1 for the top
        beams, -1 when only the lower (backup) beams were used.
        """
        good = self._valid_ranges(ranges, valid)
        self.last = {"n_beams": len(good), "angle_meas": None, "per_beam": {}}
        if not good:
            backup = self._valid_ranges(ranges, valid, self.backup)
            if not backup:
                return None, 0
            n = self.normal
            per = {i: d * float(n @ self.u[i]) for i, d in backup.items()}
            self.last["per_beam"] = per
            self.last["n_beams"] = -1
            return float(np.mean(list(per.values()))), -1

        if len(good) == 2:
            (i, di), (j, dj) = sorted(good.items())
            a = self._angle_from_two(i, j, di, dj)
            self.last["angle_meas"] = a
            if abs(a) <= self.max_angle:
                if self.angle is None:
                    self.angle = a
                else:
                    self.angle += self.angle_alpha * (a - self.angle)
            else:
                # beams disagree: keep the one closest to the current estimate
                n = self.normal
                ref = self.x[0] if self.x is not None else min(di, dj) * float(n @ self.u[i])
                k = min(good, key=lambda b: abs(good[b] * float(n @ self.u[b]) - ref))
                self.last["dropped_beam"] = j if k == i else i
                good = {k: good[k]}
                self.last["n_beams"] = 1

        n = self.normal
        per = {i: d * float(n @ self.u[i]) for i, d in good.items()}
        self.last["per_beam"] = per
        return float(np.mean(list(per.values()))), len(good)

    # ------------------------------------------------------------------
    def update(self, ranges, valid, dt, velocity=None):
        """Feed one DVL message. Returns (distance or None, is_valid).

        velocity: DVL velocity (x, y, z) in the DVL frame, or None when it is
        not valid. Only used to confirm a sudden distance change.
        """
        dt = max(float(dt), 0.0)
        self.clock += dt
        self._predict(dt)
        self.time_since_meas += dt
        if velocity is None:
            self.moved_ok = False
        else:
            # moving along +n (towards the hull) makes the distance smaller
            self.moved -= float(self.normal @ np.asarray(velocity, float)) * dt

        z, nb = self.measure(ranges, valid)
        self.last["raw"] = z
        self.last["rejected"] = False

        if z is not None:
            if self.x is None:
                self._init(z)
            else:
                r = {2: self.meas_std_two, 1: self.meas_std_one}.get(nb, self.meas_std_backup)
                confirmed = (self.moved_ok and abs(self.moved) >= self.gate
                             and self.last_accepted is not None
                             and abs(z - (self.last_accepted + self.moved)) <= self.gate)
                self.last["velocity_confirmed"] = False
                if abs(z - self.x[0]) > self.gate and confirmed:
                    # the DVL velocity says the ROV really moved this much: accept now
                    self.last["velocity_confirmed"] = True
                    self._init(z)
                elif abs(z - self.x[0]) > self.gate:
                    self.rejects += 1
                    self.rejected.append((self.clock, z))
                    self.last["rejected"] = True
                    vals = [v for _, v in self.rejected]
                    lasting = self.clock - self.rejected[0][0] >= self.reset_time
                    consistent = max(vals) - min(vals) <= 2 * self.gate
                    if lasting and len(vals) >= self.max_rejects and consistent:
                        # persistent, self-consistent change: accept it (no lockout)
                        self._init(float(np.median(vals[-self.max_rejects:])))
                    elif lasting and not consistent:
                        # scattered rejections: keep only the recent ones
                        self.rejected = [(t, v) for t, v in self.rejected
                                         if self.clock - t < self.reset_time]
                else:
                    self.rejects = 0
                    self.rejected = []
                    self._correct(z, r)
                    self.time_since_meas = 0.0
                    self._accepted(z)

        is_valid = self.x is not None and self.time_since_meas <= self.timeout
        dist = float(self.x[0]) if self.x is not None else None
        return dist, is_valid

    # ------------------------------------------------------------------
    def _init(self, z):
        self.x = np.array([z, 0.0])
        self.P = np.diag([self.meas_std_one ** 2, 0.1 ** 2])
        self.rejects = 0
        self.rejected = []
        self.time_since_meas = 0.0
        self._accepted(z)

    def _accepted(self, z):
        self.last_accepted = z
        self.moved = 0.0
        self.moved_ok = True

    def _predict(self, dt):
        if self.x is None or dt == 0.0:
            return
        F = np.array([[1.0, dt], [0.0, 1.0]])
        q = self.accel_std ** 2
        Q = q * np.array([[dt ** 4 / 4, dt ** 3 / 2], [dt ** 3 / 2, dt ** 2]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        # no accepted reading recently: stop extrapolating the rate
        if self.time_since_meas > self.rate_hold:
            self.x[1] *= math.exp(-dt / self.rate_hold)

    def _correct(self, z, r):
        H = np.array([1.0, 0.0])
        S = H @ self.P @ H + r ** 2
        K = self.P @ H / S
        self.x = self.x + K * (z - H @ self.x)
        self.P = self.P - np.outer(K, H @ self.P)
