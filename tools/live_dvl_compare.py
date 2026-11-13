#!/usr/bin/env python3
"""
Compare the OLD and NEW hull distance code LIVE, on the /dvl/data topic,
while the DVL driver is running (on the robot or in the pool), or while a
bag is being played with `ros2 bag play`. Same code as tools/replay_dvl_bag.py.

It does not send any command to the ROV. It only listens to /dvl/data.

    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash        # for dvl_msgs
    python3 tools/live_dvl_compare.py --out live_out
    python3 tools/live_dvl_compare.py --out live_out --show     # with a live plot window

Press Ctrl+C to stop. While it runs:
    - prints one line per second: old distance, new distance, valid, beams, hull angle
    - publishes /hull_compare (std_msgs/Float64MultiArray):
          [old, new, new_valid, hull_angle_deg, beams_used]
      so it can be plotted with rqt_plot or PlotJuggler
    - appends every DVL message to live_out/live.csv (saved as it goes)
When stopped:
    - prints the same summary as the bag replay
    - writes live_out/live.png

beams_used: 2 / 1 = top beams, -1 = lower beams only, 0 = none.
If dvl_msgs is not found, source the workspace that has the DVL driver.
"""
import argparse
import csv
import importlib
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from replay_dvl_bag import FIELDS, Comparison, draw, print_summary, save_plot  # noqa: E402


def load_msg_type(path):
    mod, _, name = path.rpartition(".")
    try:
        return getattr(importlib.import_module(mod), name)
    except (ImportError, AttributeError) as e:
        raise SystemExit(f"cannot import {path} ({e}).\n"
                         "Source the workspace with the DVL driver first, e.g.\n"
                         "  source ~/ros2_ws/install/setup.bash")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="/dvl/data")
    ap.add_argument("--msg-type", default="dvl_msgs.msg.DVL", help="Python path of the DVL message type")
    ap.add_argument("--out", default="live_out")
    ap.add_argument("--beams", default="2,3", help="0-based top beam indices")
    ap.add_argument("--gate", type=float, default=0.15)
    ap.add_argument("--jump", type=float, default=0.05, help="step size counted as a jump (m)")
    ap.add_argument("--no-velocity", action="store_true", help="do not pass the DVL velocity to the new method")
    ap.add_argument("--pub-topic", default="/hull_compare", help="'' to not publish")
    ap.add_argument("--show", action="store_true", help="live plot window (needs a display)")
    ap.add_argument("--window", type=float, default=60.0, help="seconds shown in the live plot")
    a, ros_args = ap.parse_known_args()

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Float64MultiArray

    DVL = load_msg_type(a.msg_type)
    pair = tuple(int(i) for i in a.beams.split(","))
    os.makedirs(a.out, exist_ok=True)
    csv_path = os.path.join(a.out, "live.csv")
    cmp_ = Comparison(pair=pair, gate=a.gate, use_velocity=not a.no_velocity)
    lock = threading.Lock()

    class LiveCompare(Node):
        def __init__(self):
            super().__init__("live_dvl_compare")
            # the DVL driver may publish as reliable or best effort; sensor QoS accepts both
            self.create_subscription(DVL, a.topic, self.cb, qos_profile_sensor_data)
            self.pub = self.create_publisher(Float64MultiArray, a.pub_topic, 10) if a.pub_topic else None
            self.f = open(csv_path, "w", newline="")
            self.w = csv.DictWriter(self.f, fieldnames=FIELDS)
            self.w.writeheader()
            self.last_print = 0.0
            self.get_logger().info(f"listening to {a.topic}, writing {csv_path}")

        def cb(self, msg):
            st = msg.header.stamp
            t = st.sec + st.nanosec * 1e-9
            if t == 0.0:                                     # no stamp: use the node clock
                t = self.get_clock().now().nanoseconds * 1e-9
            beams = list(msg.beams)
            if len(beams) < 4:
                return
            vel = None
            if getattr(msg, "velocity_valid", False):
                vel = [msg.velocity.x, msg.velocity.y, msg.velocity.z]
            with lock:
                row = cmp_.step(t, [float(b.distance) for b in beams[:4]],
                                [bool(b.valid) for b in beams[:4]], vel)
            self.w.writerow(row)
            self.f.flush()
            if self.pub:
                nan = float("nan")
                m = Float64MultiArray()
                m.data = [float(row["old"]) if row["old"] is not None else nan,
                          float(row["new"]) if row["new"] is not None else nan,
                          float(row["new_valid"]),
                          float(row["angle_deg"]) if row["angle_deg"] is not None else nan,
                          float(row["n_top"] or 0)]
                self.pub.publish(m)
            now = time.monotonic()
            if now - self.last_print >= 1.0:
                self.last_print = now
                fmt = lambda x: "  -- " if x is None else f"{x:5.2f}"  # noqa: E731
                ang = row["angle_deg"]
                self.get_logger().info(
                    f"t={row['t']:7.1f}s  old={fmt(row['old'])} m  new={fmt(row['new'])} m  "
                    f"valid={row['new_valid']}  beams={row['n_top']:2d}  "
                    f"angle={'--' if ang is None else f'{ang:.1f}'} deg"
                    f"{'  REJECTED' if row['new_rejected'] else ''}")

    rclpy.init(args=["live_dvl_compare"] + ros_args)
    node = LiveCompare()
    try:
        if a.show:
            import matplotlib.pyplot as plt
            def spin_quietly():
                try:
                    rclpy.spin(node)
                except (KeyboardInterrupt, ExternalShutdownException):
                    pass                      # Ctrl+C
            spin = threading.Thread(target=spin_quietly, daemon=True)
            spin.start()
            plt.ion()
            fig = plt.figure(figsize=(12, 8))
            while plt.fignum_exists(fig.number) and rclpy.ok():
                with lock:
                    rows = [r for r in cmp_.rows if cmp_.rows and r["t"] >= cmp_.rows[-1]["t"] - a.window]
                if len(rows) > 1:
                    draw(rows, pair, fig)
                plt.pause(1.0)
        else:
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                                  # Ctrl+C: write the summary below
    finally:
        node.f.close()
        print()
        with lock:
            rows = list(cmp_.rows)
        print_summary(rows, a.jump)
        print(f"wrote {csv_path}")
        if len(rows) > 1:
            save_plot(rows, pair, os.path.join(a.out, "live.png"))
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass                              # already shut down by Ctrl+C


if __name__ == "__main__":
    main()
