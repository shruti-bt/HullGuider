# Hull distance from the DVL: what changed

![current vs proposed](distance_current_vs_proposed.png)

## The problem
The distance to the hull jumped by several centimetres from one DVL message to the next.

- The distance comes only from the two top beams (`top_beam_indices = [2, 3]`).
- The old `compute_distance_2` averaged `d × cos 22.5°` over the valid top beams. That is only correct when the DVL is square to the hull. Its yaw correction rotated about the DVL boresight, so it had no effect.
- When the ROV is turned, the two beams measure different ranges. When one beam drops out, the average switches to the other beam alone. At 15° that is a jump of about 7 cm, and more at larger angles.
- `filter_distance` only rejected changes larger than 0.5 m, so these jumps passed through to the controller. Rejected values never entered its history, so after a real change of more than 0.5 m it could stay locked on the old value.
- With no valid beam the distance became 0.01 m.

## The new method (`hull_guider/core/hull_distance.py`)
1. **Both top beams valid:** the two hit points lie on the hull. The line through them gives the hull angle, without the compass. The angle is smoothed and stored.
2. **Each beam gives the true perpendicular distance:** `D = d × (n · u)`, where `n` is the hull normal and `u` is the beam direction. Both beams give the same `D`, so losing one causes no jump.
3. **One beam valid:** use it with the last hull angle.
4. **No beam valid:** keep predicting. After 1 s the distance is marked invalid. There is no 0.01 m fallback.
5. **Gate:** reject a reading more than 0.15 m from the estimate. After 3 rejections in a row, accept the new value, so the filter cannot lock.
6. **Smoothing:** a small Kalman filter on distance and rate.

**Assumption:** the two top beams only measure the hull angle about the DVL x axis, which is the ROV yaw relative to the hull. Tilt about the other axis is assumed to be zero.

## Interface changes
- **`/hull/state` now has 5 values:** `[distance, yaw_error, nx, ny, valid]`. `valid` is 1.0 or 0.0.
- **`guided_controller`:** when `valid` is 0 it sends zero velocity and holds. It also starts its distance filter from the first real reading instead of from 0. A hull node that publishes only 4 values is treated as always valid.
- **Normal position update:** the position along the hull normal is only updated from a valid, filtered distance.
- **Logging:** the node logs the raw distance, the number of beams, the hull angle, whether the reading was rejected, and whether the distance is valid, for every DVL message.

**Tuning:** `gate`, `max_rejects` and `timeout` are set where `self.hull_dist` is created in `dvl_hull_node.py`. `max_angle_deg` (35°) limits which two-beam angle measurements are accepted.

## Testing

**Unit tests (no ROS):**
```bash
python3 -m pytest test/test_hull_distance.py -q
```

**Replay a recorded bag (no ROS needed):** this compares the old and new distance on the real `/dvl/data`.
```bash
pip install rosbags numpy matplotlib
python3 tools/replay_dvl_bag.py /path/to/bag_folder --out replay_out
# if the bag does not store the dvl_msgs definitions:
python3 tools/replay_dvl_bag.py /path/to/bag_folder --msg-dir ~/ros2_ws/src/dvl-a50/dvl_msgs/msg
```
It prints:
- how often each top beam was valid
- the number of steps larger than 5 cm for the old and the new method
- the range of the hull angle

It also writes:
- `replay_out/replay.csv`, one row per DVL message
- `replay_out/replay.png`, the old and new distance, the hull angle, and when each beam was valid

**Full node on a bag (ROS 2):**
- The bag must contain `/dvl/data`, `/mavros/imu/data` and `/mavros/global_position/rel_alt`.
- Do not run `guided_controller` while a real vehicle is connected.
```bash
colcon build --packages-select hull_guider && source install/setup.bash
ros2 run hull_guider dvl_hull        # terminal 1
ros2 bag play /path/to/bag_folder    # terminal 2
ros2 topic echo /hull/state          # terminal 3
```
