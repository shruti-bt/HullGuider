

COLMAP ignores priors unless configured

You must enable:

```sh
--Mapper.use_prior_position 1
```




## Help 1

Nice — this is exactly the right pipeline to build 👍
Let’s structure it cleanly so you can actually *use it with COLMAP without fighting it*.

---

# 🧭 Overview of what you want

You want a pipeline:

### Stage 1 (ROS2 logging)

Synchronize and save:

* Image filename
* Z prior (from VFR_HUD.alt)
* IMU (orientation)

→ write to a file (CSV / TXT)

---

### Stage 2 (offline conversion)

Take:

* image list (paths)
* your logged priors

→ generate **COLMAP-compatible priors**

---

### Stage 3 (COLMAP config)

Run mapper with priors enabled

---

# ⚙️ 1. ROS2 Time Synchronizer (Image + VFR_HUD + IMU)

We’ll use `ApproximateTimeSynchronizer` (3 topics).

---

## ✅ Node code

```python
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, Imu
from mavros_msgs.msg import VFR_HUD

from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge

import os
import csv
import cv2


class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')

        self.bridge = CvBridge()

        # Subscribers
        self.img_sub = Subscriber(self, Image, '/image_raw')
        self.imu_sub = Subscriber(self, Imu, '/mavros/imu/data')
        self.vfr_sub = Subscriber(self, VFR_HUD, '/mavros/vfr_hud')

        self.ts = ApproximateTimeSynchronizer(
            [self.img_sub, self.imu_sub, self.vfr_sub],
            queue_size=30,
            slop=0.05
        )
        self.ts.registerCallback(self.callback)

        # Output dirs
        self.img_dir = "images"
        os.makedirs(self.img_dir, exist_ok=True)

        self.csv_path = "priors.csv"

        # CSV header
        with open(self.csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "image_name",
                "timestamp",
                "z",
                "qx", "qy", "qz", "qw"
            ])

        self.counter = 0

    def callback(self, img_msg, imu_msg, vfr_msg):
        # ---- Save image ----
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')

        ts = img_msg.header.stamp.sec + img_msg.header.stamp.nanosec * 1e-9
        filename = f"{ts:.6f}.png"
        path = os.path.join(self.img_dir, filename)

        cv2.imwrite(path, cv_img)

        # ---- Extract priors ----
        z = vfr_msg.alt

        q = imu_msg.orientation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w

        # ---- Save CSV ----
        with open(self.csv_path, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                filename,
                ts,
                z,
                qx, qy, qz, qw
            ])

        self.get_logger().info(f"Saved {filename} | Z={z:.2f}")


def main():
    rclpy.init()
    node = DataLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

# 📄 Output format (important)

You now get:

```csv
image_name,timestamp,z,qx,qy,qz,qw
1678901234.123456.png,1678901234.123456,2.31,0.01,0.02,0.03,0.99
```

---

# 🔁 2. Convert to COLMAP priors

COLMAP expects:

```
IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID IMAGE_NAME
```

---

## ✅ Conversion script

```python
import csv

input_csv = "priors.csv"
output_txt = "images.txt"

with open(input_csv) as f, open(output_txt, "w") as out:
    reader = csv.DictReader(f)

    out.write("# IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID IMAGE_NAME\n\n")

    for i, row in enumerate(reader, start=1):
        name = row["image_name"]
        z = float(row["z"])

        # orientation
        qw = float(row["qw"])
        qx = float(row["qx"])
        qy = float(row["qy"])
        qz = float(row["qz"])

        # translation (only Z meaningful)
        tx, ty, tz = 0, 0, z

        line = f"{i} {qw} {qx} {qy} {qz} {tx} {ty} {tz} 1 {name}\n\n"
        out.write(line)
```

---

# ⚠️ IMPORTANT: This is NOT enough for COLMAP

COLMAP does NOT automatically use this file unless:

👉 you import into the **database** OR modify mapper

---

# 🚀 3. Proper way to use priors in COLMAP

There are 2 real options:

---

## ✅ Option A (Recommended): Use database priors

You must insert priors into COLMAP SQLite DB:

Table:

```
images.prior_qw, prior_qx, ..., prior_tx, ...
```

---

## 🔧 Quick hack (works but limited)

Run mapper with:

```bash
colmap mapper \
    --database_path database.db \
    --image_path images \
    --output_path sparse \
    --Mapper.use_prior_position 1 \
    --Mapper.use_prior_orientation 1 \
    --Mapper.prior_position_weight 1.0 \
    --Mapper.prior_orientation_weight 1.0
```

---

## ⚠️ Caveat

COLMAP will:

* treat priors as **soft constraints**
* still override them if reprojection disagrees

---

# 💡 Best practice (what I recommend for you)

For your setup (ROV + MAVROS):

### ✅ Use:

* Z from VFR_HUD (or better: pressure)
* IMU orientation → VERY important
* weak XY = 0

### ✅ Enable:

```bash
--Mapper.use_prior_position 1
--Mapper.use_prior_orientation 1
```

---

# 🧠 Critical insight

Your setup becomes:

* IMU → fixes rotation (removes 3 DOF)
* Z → partially fixes scale
* SfM → solves XY

👉 This is actually a **well-constrained system**

---

# 🔥 If you want the *correct* way (next level)

I can show you:

* how to write directly into COLMAP DB (best way)
* how to align IMU frame ↔ camera frame properly
* how to avoid MAVROS frame pitfalls (this WILL bite you)

Just tell me 👍





