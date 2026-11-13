"""Tests for hull_guider.core.hull_distance (no ROS needed).

Run:  python3 -m pytest test/test_hull_distance.py -q
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "hull_guider", "core"))
from hull_distance import HullDistanceEstimator, default_beam_dirs  # noqa: E402

U = default_beam_dirs()


def ranges_for(standoff, yaw_deg):
    """Exact beam ranges to a flat hull at `standoff`, DVL turned `yaw_deg`."""
    a = math.radians(yaw_deg)
    n = np.array([0.0, math.sin(a), math.cos(a)])
    return [standoff / float(n @ u) for u in U]


def old_distance(ranges, valid, pair=(2, 3)):
    """The old compute_distance_2 (its yaw rotation has no effect)."""
    vals = [ranges[i] * U[i][2] for i in pair if valid[i] and ranges[i] > 0]
    return abs(float(np.mean(vals))) if vals else None


def test_square_to_hull():
    est = HullDistanceEstimator()
    d, ok = est.update(ranges_for(0.8, 0.0), [True] * 4, 0.1)
    assert ok and abs(d - 0.8) < 1e-9


def test_yaw_gives_true_distance_both_beams():
    for yaw in (-25, -15, -5, 5, 15, 25):
        est = HullDistanceEstimator()
        z, nb = est.measure(ranges_for(0.8, yaw), [True] * 4)
        assert nb == 2
        assert abs(z - 0.8) < 1e-9, (yaw, z)
        assert abs(math.degrees(est.angle) - yaw) < 1e-6


def test_one_beam_drop_no_jump():
    est = HullDistanceEstimator()
    r = ranges_for(0.8, 15.0)
    est.measure(r, [True] * 4)                       # learns the hull angle
    for valid in ([True, True, False, True], [True, True, True, False]):
        z, nb = est.measure(r, valid)
        assert nb == 1 and abs(z - 0.8) < 1e-9
    # the old method jumps by several cm in the same situation
    both = old_distance(r, [True] * 4)
    only2 = old_distance(r, [True, True, True, False])
    only3 = old_distance(r, [True, True, False, True])
    assert abs(only2 - both) > 0.05 and abs(only3 - both) > 0.05


def test_no_beams_becomes_invalid_not_one_cm():
    est = HullDistanceEstimator(timeout=0.5)
    est.update(ranges_for(0.8, 0.0), [True] * 4, 0.1)
    none = [False] * 4
    d, ok = est.update([0.0] * 4, none, 0.2)
    assert ok and abs(d - 0.8) < 0.01               # short gap: hold
    for _ in range(3):
        d, ok = est.update([0.0] * 4, none, 0.2)
    assert not ok                                    # long gap: invalid
    assert d is not None and d > 0.5                 # never 0.01 m


def test_before_first_measurement():
    est = HullDistanceEstimator()
    d, ok = est.update([0.0] * 4, [False] * 4, 0.1)
    assert d is None and not ok


def test_spike_rejected_and_no_lockout():
    est = HullDistanceEstimator(gate=0.15, max_rejects=3)
    for _ in range(10):
        est.update(ranges_for(0.8, 0.0), [True] * 4, 0.1)
    d, _ = est.update(ranges_for(2.0, 0.0), [True] * 4, 0.1)   # one bad spike
    assert abs(d - 0.8) < 0.02
    d, _ = est.update(ranges_for(0.8, 0.0), [True] * 4, 0.1)
    assert abs(d - 0.8) < 0.02
    # a short burst of long readings (reflection, < reset_time) is not accepted
    for _ in range(7):
        d, _ = est.update(ranges_for(1.4, 0.0), [True] * 4, 0.1)
    assert abs(d - 0.8) < 0.03
    for _ in range(5):
        d, _ = est.update(ranges_for(0.8, 0.0), [True] * 4, 0.1)
    # a real step change of 0.6 m that lasts: accepted after reset_time (old filter locks)
    for _ in range(17):
        d, _ = est.update(ranges_for(1.4, 0.0), [True] * 4, 0.1)
    assert abs(d - 1.4) < 0.02


def test_disagreeing_beams_use_the_consistent_one():
    est = HullDistanceEstimator()
    for _ in range(10):
        est.update(ranges_for(0.45, 0.0), [True] * 4, 0.2)
    r = ranges_for(0.45, 0.0)
    r[2] = 2.0                       # beam 2 hits something far (seen at 86.7 s in oct3026-1)
    d, ok = est.update(r, [True] * 4, 0.2)
    assert est.last["n_beams"] == 1 and abs(d - 0.45) < 0.01


def test_lower_beams_used_when_top_beams_lost():
    est = HullDistanceEstimator()
    r = ranges_for(0.8, 15.0)
    est.update(r, [True] * 4, 0.2)                    # learns the hull angle
    d, ok = est.update(r, [True, True, False, False], 0.2)
    assert est.last["n_beams"] == -1 and ok and abs(d - 0.8) < 0.01
    z, nb = est.measure(r, [False, True, False, False])
    assert nb == -1 and abs(z - 0.8) < 1e-9


def test_range_limit_is_no_hull():
    est = HullDistanceEstimator()
    z, nb = est.measure([2.95] * 4, [True] * 4)       # DVL range limit (bag 5)
    assert z is None and nb == 0


def test_timeout_covers_dropouts_seen_in_bags():
    est = HullDistanceEstimator()                     # default timeout 3 s
    est.update(ranges_for(0.8, 0.0), [True] * 4, 0.2)
    for _ in range(13):                                # 2.6 s, longest gap in the bags
        d, ok = est.update([0.0] * 4, [False] * 4, 0.2)
    assert ok and abs(d - 0.8) < 0.02
    for _ in range(3):
        d, ok = est.update([0.0] * 4, [False] * 4, 0.2)
    assert not ok


def test_velocity_confirms_a_real_jump_at_once():
    def run(with_velocity):
        est = HullDistanceEstimator()
        for _ in range(10):
            est.update(ranges_for(0.8, 0.0), [True] * 4, 0.2, velocity=[0, 0, 0])
        away = [0.0, 0.0, -0.3]                       # 0.3 m/s away from the hull
        for _ in range(5):                            # 1 s with no usable beam
            est.update([0.0] * 4, [False] * 4, 0.2, velocity=away if with_velocity else None)
        d, _ = est.update(ranges_for(1.1, 0.0), [True] * 4, 0.2,
                          velocity=away if with_velocity else None)
        return d, est
    d, est = run(True)
    assert est.last["velocity_confirmed"] and abs(d - 1.1) < 0.01
    d, est = run(False)                               # no velocity: rejected, waits reset_time
    assert est.last["rejected"] and abs(d - 0.8) < 0.05


def test_velocity_does_not_accept_a_reflection():
    est = HullDistanceEstimator()
    for _ in range(10):
        est.update(ranges_for(0.8, 0.0), [True] * 4, 0.2, velocity=[0, 0, 0])
    d, _ = est.update(ranges_for(1.4, 0.0), [True] * 4, 0.2, velocity=[0, 0, 0])
    assert est.last["rejected"] and abs(d - 0.8) < 0.02


def test_simulated_run_with_dropouts():
    """Hull following at 0.8 m, slowly varying yaw, noise and beam dropouts."""
    rng = np.random.default_rng(0)
    est = HullDistanceEstimator()
    dt, n = 0.1, 600
    err_new, jumps_old, jumps_new = [], [], []
    prev_old = prev_new = None
    for k in range(n):
        yaw = 15.0 * math.sin(2 * math.pi * k / 300)
        r = np.array(ranges_for(0.8, yaw)) + rng.normal(0, 0.01, 4)
        valid = [True] * 4
        if rng.random() < 0.2:
            valid[rng.choice([2, 3])] = False        # one top beam drops
        d_new, ok = est.update(r, valid, dt)
        d_old = old_distance(r, valid)
        if k > 20:
            err_new.append(d_new - 0.8)
            jumps_old.append(abs(d_old - prev_old))
            jumps_new.append(abs(d_new - prev_new))
        prev_old, prev_new = d_old, d_new
    rmse_new = math.sqrt(np.mean(np.square(err_new)))
    assert rmse_new < 0.02, rmse_new
    assert max(jumps_new) < 0.02, max(jumps_new)
    assert max(jumps_old) > 0.05, max(jumps_old)
