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
4. No beam valid: predict only; after `timeout` seconds the output is
   marked invalid (instead of the old 0.01 m fallback).
5. Gate: a measurement more than `gate` metres from the estimate is
   rejected, but after `max_rejects` rejections in a row the filter
   restarts from the new value, so it can never lock on an old value.
6. A small constant-velocity Kalman filter (distance, rate) smooths it.

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
                 gate=0.15, max_rejects=3,
                 accel_std=0.1, meas_std_two=0.02, meas_std_one=0.04,
                 timeout=1.0, min_range=0.05, max_range=5.0):
        self.u = np.asarray(beam_dirs if beam_dirs is not None else default_beam_dirs(), float)
        self.pair = tuple(beam_pair)
        self.max_angle = math.radians(max_angle_deg)
        self.angle_alpha = angle_alpha
        self.gate = gate
        self.max_rejects = max_rejects
        self.accel_std = accel_std
        self.meas_std_two = meas_std_two
        self.meas_std_one = meas_std_one
        self.timeout = timeout
        self.min_range = min_range
        self.max_range = max_range
        self.reset()

    # ------------------------------------------------------------------
    def reset(self):
        self.angle = None          # hull angle about DVL x (rad), smoothed
        self.x = None              # Kalman state [D, D_dot]
        self.P = None
        self.rejects = 0
        self.time_since_meas = float("inf")
        self.last = {}             # diagnostics of the last update

    @property
    def normal(self):
        """Hull normal in the DVL frame (pointing from DVL to hull)."""
        a = self.angle or 0.0
        return np.array([0.0, math.sin(a), math.cos(a)])

    # ------------------------------------------------------------------
    def _valid_ranges(self, ranges, valid):
        out = {}
        for i in self.pair:
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

        Returns (D, n_beams) or (None, 0).
        """
        good = self._valid_ranges(ranges, valid)
        self.last = {"n_beams": len(good), "angle_meas": None, "per_beam": {}}
        if not good:
            return None, 0

        if len(good) == 2:
            (i, di), (j, dj) = sorted(good.items())
            a = self._angle_from_two(i, j, di, dj)
            self.last["angle_meas"] = a
            if abs(a) <= self.max_angle:
                if self.angle is None:
                    self.angle = a
                else:
                    self.angle += self.angle_alpha * (a - self.angle)

        n = self.normal
        per = {i: d * float(n @ self.u[i]) for i, d in good.items()}
        self.last["per_beam"] = per
        return float(np.mean(list(per.values()))), len(good)

    # ------------------------------------------------------------------
    def update(self, ranges, valid, dt):
        """Feed one DVL message. Returns (distance or None, is_valid)."""
        dt = max(float(dt), 0.0)
        self._predict(dt)
        self.time_since_meas += dt

        z, nb = self.measure(ranges, valid)
        self.last["raw"] = z
        self.last["rejected"] = False

        if z is not None:
            if self.x is None:
                self._init(z)
            else:
                r = self.meas_std_two if nb == 2 else self.meas_std_one
                if abs(z - self.x[0]) > self.gate:
                    self.rejects += 1
                    self.last["rejected"] = True
                    if self.rejects >= self.max_rejects:
                        # persistent change: accept it instead of locking out
                        self._init(z)
                else:
                    self.rejects = 0
                    self._correct(z, r)
                    self.time_since_meas = 0.0

        is_valid = self.x is not None and self.time_since_meas <= self.timeout
        dist = float(self.x[0]) if self.x is not None else None
        return dist, is_valid

    # ------------------------------------------------------------------
    def _init(self, z):
        self.x = np.array([z, 0.0])
        self.P = np.diag([self.meas_std_one ** 2, 0.1 ** 2])
        self.rejects = 0
        self.time_since_meas = 0.0

    def _predict(self, dt):
        if self.x is None or dt == 0.0:
            return
        F = np.array([[1.0, dt], [0.0, 1.0]])
        q = self.accel_std ** 2
        Q = q * np.array([[dt ** 4 / 4, dt ** 3 / 2], [dt ** 3 / 2, dt ** 2]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def _correct(self, z, r):
        H = np.array([1.0, 0.0])
        S = H @ self.P @ H + r ** 2
        K = self.P @ H / S
        self.x = self.x + K * (z - H @ self.x)
        self.P = self.P - np.outer(K, H @ self.P)
