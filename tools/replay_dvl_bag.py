#!/usr/bin/env python3
"""
Replay /dvl/data from a recorded ROS 2 bag through the OLD and the NEW hull
distance code, and compare them. No ROS installation needed.

    pip install rosbags numpy matplotlib
    python3 tools/replay_dvl_bag.py /path/to/bag_folder --out replay_out

Outputs in --out:
    replay.csv   per DVL message: time, the 4 beam ranges + valid flags,
                 old distance, new raw / filtered distance, hull angle, valid
    replay.png   old vs new distance over time, beam validity underneath
    summary      printed: jumps per method, how often a top beam dropped out

If the bag does not store the dvl_msgs definitions and reading fails, point
--msg-dir at the dvl_msgs/msg folder of the driver used on the robot.
"""
import argparse
import csv
import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "hull_guider", "core"))
from hull_distance import HullDistanceEstimator, default_beam_dirs  # noqa: E402

# Water Linked dvl-a50-ros2 driver definitions, used only as a fallback.
FALLBACK_MSGS = {
    "dvl_msgs/msg/DVLBeam": """
int64 id
float64 velocity
float64 distance
float64 rssi
float64 nsd
bool valid
""",
    "dvl_msgs/msg/DVL": """
std_msgs/Header header
float64 time
geometry_msgs/Vector3 velocity
float64 fom
float64[9] covariance
float64 altitude
dvl_msgs/DVLBeam[] beams
bool velocity_valid
int64 status
string form
""",
}


# ----------------------------------------------------------------------------
# OLD method, copied from dvl_hull_node.py (compute_distance_2 + filter_distance
# + the 0.01 m fallback). The yaw rotation in compute_distance_2 has no effect,
# so it is left out here.
# ----------------------------------------------------------------------------
class OldDistance:
    def __init__(self, pair=(2, 3), jump=0.5, window=5):
        self.u = default_beam_dirs()
        self.pair = pair
        self.jump = jump
        self.window = window
        self.hist = []
        self.last_valid = None

    def raw(self, ranges, valid):
        vals = [ranges[i] * self.u[i][2] for i in self.pair if valid[i] and ranges[i] > 0]
        return abs(float(np.mean(vals))) if vals else None

    def update(self, ranges, valid):
        d = self.raw(ranges, valid)
        if d is None:
            out = self.last_valid
        elif len(self.hist) < 2:
            self.hist.append(d)
            self.last_valid = d
            out = d
        elif abs(d - np.mean(self.hist)) > self.jump:
            out = self.last_valid
        else:
            self.hist.append(d)
            if len(self.hist) > self.window:
                self.hist.pop(0)
            self.last_valid = d
            out = d
        return out if out else 0.01


# ----------------------------------------------------------------------------
def make_typestore(msg_dir):
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    add = {}
    if msg_dir:
        for f in Path(msg_dir).glob("*.msg"):
            add.update(get_types_from_msg(f.read_text(), f"dvl_msgs/msg/{f.stem}"))
    else:
        for name, text in FALLBACK_MSGS.items():
            add.update(get_types_from_msg(text, name))
    ts.register(add)
    return ts


def read_dvl(bag, topic, msg_dir):
    from rosbags.highlevel import AnyReader
    ts = make_typestore(msg_dir)
    rows = []
    with AnyReader([Path(bag)], default_typestore=ts) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            topics = sorted({c.topic for c in reader.connections})
            raise SystemExit(f"topic {topic} not in bag. Topics:\n  " + "\n  ".join(topics))
        for conn, t_ns, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            beams = list(msg.beams)
            if len(beams) < 4:
                continue
            rows.append((t_ns * 1e-9,
                         [float(b.distance) for b in beams[:4]],
                         [bool(b.valid) for b in beams[:4]]))
    return rows


def jumps(x, t, thr):
    x = np.asarray(x, float)
    dx = np.abs(np.diff(x))
    ok = np.isfinite(dx)
    return int(np.sum(dx[ok] > thr)), (float(np.nanmax(dx)) if ok.any() else float("nan"))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="ROS 2 bag folder (or ROS 1 .bag)")
    ap.add_argument("--topic", default="/dvl/data")
    ap.add_argument("--out", default="replay_out")
    ap.add_argument("--msg-dir", default=None, help="folder with DVL.msg and DVLBeam.msg")
    ap.add_argument("--beams", default="2,3", help="0-based top beam indices")
    ap.add_argument("--gate", type=float, default=0.15)
    ap.add_argument("--jump", type=float, default=0.05, help="step size counted as a jump (m)")
    ap.add_argument("--no-plot", action="store_true")
    a = ap.parse_args()

    pair = tuple(int(i) for i in a.beams.split(","))
    rows = read_dvl(a.bag, a.topic, a.msg_dir)
    if not rows:
        raise SystemExit("no DVL messages read")
    os.makedirs(a.out, exist_ok=True)

    old = OldDistance(pair=pair)
    new = HullDistanceEstimator(beam_pair=pair, gate=a.gate)
    t0 = rows[0][0]
    prev_t = None
    out = []
    for t, r, v in rows:
        dt = 0.0 if prev_t is None else t - prev_t
        prev_t = t
        d_old_raw = old.raw(r, v)
        d_old = old.update(r, v)
        d_new, ok = new.update(r, v, dt)
        info = new.last
        out.append(dict(
            t=t - t0, r0=r[0], r1=r[1], r2=r[2], r3=r[3],
            v0=int(v[0]), v1=int(v[1]), v2=int(v[2]), v3=int(v[3]),
            old_raw=d_old_raw, old=d_old,
            new_raw=info.get("raw"), new=d_new, new_valid=int(ok),
            new_rejected=int(bool(info.get("rejected"))),
            angle_deg=math.degrees(new.angle) if new.angle is not None else None,
            n_top=info.get("n_beams"),
        ))

    csv_path = os.path.join(a.out, "replay.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(out[0].keys()))
        w.writeheader()
        w.writerows(out)

    t = np.array([o["t"] for o in out])
    nan = float("nan")
    f = lambda k: np.array([nan if o[k] is None else o[k] for o in out], float)  # noqa: E731
    old_d, new_d, n_top = f("old"), f("new"), f("n_top")
    n = len(out)
    j_old, m_old = jumps(old_d, t, a.jump)
    j_new, m_new = jumps(new_d, t, a.jump)
    switches = int(np.sum(np.diff(n_top) != 0))

    print(f"DVL messages: {n}, duration {t[-1]:.1f} s, rate {n / max(t[-1], 1e-9):.1f} Hz")
    print(f"top beams valid: both {np.mean(n_top == 2) * 100:.0f}%, one {np.mean(n_top == 1) * 100:.0f}%, "
          f"none {np.mean(n_top == 0) * 100:.0f}%; number of changes: {switches}")
    print(f"old: 0.01 m fallback used {int(np.sum(old_d == 0.01))} times")
    print(f"steps > {a.jump * 100:.0f} cm between messages: old {j_old} (largest {m_old:.3f} m), "
          f"new {j_new} (largest {m_new:.3f} m)")
    print(f"new: rejected {int(f('new_rejected').sum())} readings, invalid for "
          f"{np.mean(f('new_valid') == 0) * 100:.0f}% of messages")
    ang = f("angle_deg")
    if np.isfinite(ang).any():
        print(f"hull angle from the two beams: median {np.nanmedian(ang):.1f} deg, "
              f"range {np.nanmin(ang):.1f} .. {np.nanmax(ang):.1f} deg")
    print(f"wrote {csv_path}")

    if not a.no_plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib not installed, skipping plot")
            return
        fig, ax = plt.subplots(3, 1, figsize=(12, 8), sharex=True,
                               gridspec_kw={"height_ratios": [3, 1.2, 1]})
        ax[0].plot(t, f("old"), color="#b91c1c", lw=1.2, label="old (published)")
        ax[0].plot(t, f("new_raw"), ".", color="#9ca3af", ms=3, label="new, per message (before filter)")
        ax[0].plot(t, new_d, color="#15803d", lw=1.8, label="new (published)")
        inv = f("new_valid") == 0
        if inv.any():
            ax[0].plot(t[inv], new_d[inv], "x", color="#6b7280", ms=4, label="new, marked invalid")
        ax[0].set_ylabel("distance to hull (m)")
        ax[0].legend(loc="upper right", fontsize=8)
        ax[0].grid(alpha=0.3)
        ax[1].plot(t, ang, color="#2563eb", lw=1)
        ax[1].set_ylabel("hull angle (deg)")
        ax[1].grid(alpha=0.3)
        for k, i in enumerate(range(4)):
            vv = f(f"v{i}")
            ax[2].fill_between(t, k, k + 0.8 * vv, step="post",
                               color="#15803d" if i in pair else "#d1d5db")
        ax[2].set_yticks([0.4, 1.4, 2.4, 3.4])
        ax[2].set_yticklabels([f"beam {i}" for i in range(4)])
        ax[2].set_xlabel("time (s)")
        ax[2].set_title("beam valid (green = used for distance)", fontsize=9)
        fig.tight_layout()
        png = os.path.join(a.out, "replay.png")
        fig.savefig(png, dpi=120)
        print(f"wrote {png}")


if __name__ == "__main__":
    main()
