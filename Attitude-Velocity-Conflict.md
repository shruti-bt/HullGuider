
msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED



👉 Then:

vx → global North
vy → global East
vz → global Down

✔ Independent of vehicle orientation



FRAME_BODY_NED


👉 Then:

vx → forward (relative to vehicle)
vy → right
vz → down

✔ Moves with the vehicle’s heading

Yaw is always:

👉 absolute heading in the global frame

0 rad → North
π/2 → East

Even if velocity is body-frame, yaw is still global






Great—this is exactly the kind of question that separates “it kind of works” from a solid controller 👍

Short answer first:

> ✅ You **can** send attitude targets in GUIDED
> ⚠️ But you generally should **NOT mix attitude + velocity setpoints at the same time**

Let’s unpack why.

---

# 🧭 1. How to send attitude targets via MAVROS

With **MAVROS**, you publish to:

```bash
/mavros/setpoint_raw/attitude
```

Message:

```bash
mavros_msgs/msg/AttitudeTarget
```

---

## ✅ Example (yaw + thrust control)

```python
from mavros_msgs.msg import AttitudeTarget
from geometry_msgs.msg import Quaternion

msg = AttitudeTarget()

msg.type_mask = (
    AttitudeTarget.IGNORE_ROLL_RATE |
    AttitudeTarget.IGNORE_PITCH_RATE |
    AttitudeTarget.IGNORE_YAW_RATE
)

# Set orientation (quaternion)
msg.orientation = desired_quaternion

# Thrust (0–1 normalized)
msg.thrust = thrust
```

👉 Publish at **≥10 Hz**

---

# ⚙️ 2. Why mixing attitude + velocity causes problems

This is the key issue you’re worried about—and you’re absolutely right.

---

## 🎯 Core conflict

Velocity commands are interpreted in a frame:

* `FRAME_LOCAL_NED` → world frame
* `FRAME_BODY_NED` → body frame

Now imagine:

### You send:

* velocity → “go forward”
* attitude → “rotate 90°”

---

## 💥 What happens?

If using **BODY frame velocity**:

```text
Forward (body) → depends on yaw
```

So when yaw changes:

* forward direction rotates
* trajectory bends or becomes unstable

---

If using **LOCAL frame velocity**:

```text
Velocity stays fixed in world frame
```

But now:

* vehicle rotates independently
* thrusters fight between:

  * holding velocity direction
  * achieving new attitude

👉 Leads to:

* drift
* oscillation
* “crab motion”

---

# 🧠 3. Fundamental rule (very important)

> ❗ **Only ONE high-level control interface should dominate at a time**

Choose ONE:

### Option A — Velocity control (recommended)

* control: vx, vy, vz
* control: yaw (or yaw rate)
* ❌ do NOT send attitude

---

### Option B — Attitude + thrust control

* control: orientation
* control: thrust
* ❌ do NOT send velocity

---

# 🧭 4. What ArduSub expects in GUIDED

With **ArduSub**:

* Velocity setpoints → go through position/velocity logic
* Attitude setpoints → go through attitude controller

👉 These are **different control pipelines**

They are **not designed to be blended externally**

---

# 🚀 5. Best practice for ROV (what you should do)

## ✅ Use velocity + yaw (NOT full attitude)

This is the sweet spot:

```text
vx, vy, vz + yaw
```

Why:

* stable
* intuitive
* works with external pose control
* avoids frame conflicts

---

## 🧪 Example (correct setup)

```python
msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

msg.type_mask = (
    IGNORE_PX | IGNORE_PY | IGNORE_PZ |
    IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ |
    IGNORE_YAW_RATE
)

msg.velocity.x = vx
msg.velocity.y = vy
msg.velocity.z = vz
msg.yaw = desired_yaw
```

---

# ⚠️ 6. When WOULD you use attitude targets?

Only if:

* You’re doing **low-level control**
* You want:

  * full control over orientation
  * custom controller (e.g. geometric control)

Examples:

* aggressive maneuvers
* research control systems

---

# 🧠 7. Key insight (this explains your instability)

If your system felt unstable before, it’s likely because:

👉 You were mixing:

* pose → velocity
* pose → attitude

at the same time

That creates:

```text
Two controllers fighting each other
```

---

# 🎯 Final recommendation (for your setup)

Since you have:

* EKF3 ExternalNav ✅
* Pose estimation ✅

👉 Do this:

### ✔ Control:

* velocity (vx, vy, vz)
* yaw (NOT roll/pitch)

### ❌ Avoid:

* AttitudeTarget (unless fully switching control mode)

---

# 🧭 Clean mental model

Think of it like:

```text
Pose controller (ROS)
    ↓
Velocity commands
    ↓
ArduSub stabilizes attitude automatically
```

👉 You don’t need to micromanage attitude

---

# If you want next step

I can help you:

* design a **pose → velocity PID tuned for underwater drag**
* fix frame mismatches (NED vs ENU — super common bug)
* or review your MAVROS messages (type_mask mistakes are VERY common)

Just drop your current code 👍
