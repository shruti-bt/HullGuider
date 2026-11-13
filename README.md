# HullGuider

## Hull distance: old vs new method

The old method took the distance to the hull as the average of the two top DVL beams multiplied by cos 22.5°, which is only correct when the ROV is square to the hull. When the ROV was turned and one beam dropped out, the distance jumped by 5–20 cm. Its filter only rejected changes larger than 0.5 m, so these jumps reached the controller, and after a real change larger than 0.5 m it rejected every new reading and stayed locked on the old value. With no valid beam it reported 0.01 m, which made the controller back away from the hull. The new method first measures the hull angle from the two beam hit points, then converts each beam's range into the true perpendicular distance, so losing a beam no longer causes a jump. The lower beams are used as a backup when both top beams are lost. Readings more than 15 cm from the estimate are rejected, but a change that stays consistent for 1.5 s is accepted, so the filter cannot lock. A small Kalman filter smooths the result, and after 3 s without a reading the distance is marked invalid and the controller holds still instead of acting on a false value. The controller deadband was also reduced from 0.15 to 0.05 m and a small integral term added. On the five oct3026 bags, steps larger than 5 cm dropped from 1292 to 311. In simulation, after the ROV is pushed 1.2 m away from the hull, the old method stays at 2.1 m, while the new method is back within 15 cm of the 0.8 m target 8 s after the push and settles near 0.8 m (error RMS 81 cm → 19 cm). The new method does not fix the 20–40 s delay in `/dvl/data` seen in the bags, which must be fixed in the DVL driver, and it still needs to be confirmed in a pool test.

![old vs new method](docs/distance_current_vs_proposed.png)

![simulated trajectory, old vs new](docs/nav_trajectory_old_vs_new.png)

Details, figures and test results: [docs/hull_distance.md](docs/hull_distance.md).

### Run the new method

The new method is built into `dvl_hull` and `guided_controller`. There is nothing extra to switch on.

**On the robot** (each in its own terminal, after the DVL, mavros and robot state publisher steps in [Commands](#commands) below):
```sh
cd ~/ros2_ws
colcon build --packages-select hull_guider && source install/setup.bash

ros2 run hull_guider dvl_hull            # distance to the hull, publishes /hull/state
ros2 topic echo /hull/state              # check: [distance, yaw_error, nx, ny, valid]
ros2 run hull_guider guided_controller   # only when ready to move the ROV
```
- `/hull/state` now has 5 values. The last one, `valid`, is 1.0 when the distance can be trusted and 0.0 when it cannot. When it is 0.0, `guided_controller` holds still.
- `dvl_hull` logs one line per DVL message: raw distance, beams used, hull angle, whether the reading was rejected, and whether it is valid.

**On a recorded bag, with ROS** (do not run `guided_controller` with a real vehicle connected):
```sh
ros2 run hull_guider dvl_hull     # terminal 1
ros2 bag play rosbag/oct3026-1    # terminal 2
ros2 topic echo /hull/state       # terminal 3
```

**On a recorded bag, without ROS** (compares the old and new distance, writes a CSV and a plot):
```sh
pip install rosbags numpy matplotlib
python3 tools/replay_dvl_bag.py rosbag/oct3026-1 --out replay_out
```

**Live, from the topics: old vs new side by side** (on the robot or in the pool while the DVL driver runs, or while a bag plays). It only listens to `/dvl/data` and never sends commands to the ROV:
```sh
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash                       # workspace with dvl_msgs (DVL driver)
python3 tools/live_dvl_compare.py --out live_out          # Ctrl+C to stop
python3 tools/live_dvl_compare.py --out live_out --show   # same, with a live plot window
```
- Prints one line per second: old distance, new distance, valid, beams used, hull angle.
- Publishes `/hull_compare` = `[old, new, valid, hull_angle_deg, beams_used]` for `rqt_plot` or PlotJuggler, e.g. `ros2 run rqt_plot rqt_plot /hull_compare/data[0] /hull_compare/data[1]`.
- Saves every message to `live_out/live.csv` as it goes. On Ctrl+C it prints the same summary as the bag replay and writes `live_out/live.png`.
- Options: `--topic`, `--beams`, `--gate`, `--no-velocity`, `--window` (seconds in the live plot), `--pub-topic ''` (do not publish).
- If ROS 2 fails to import inside a conda environment, run it with `/usr/bin/python3` instead.

**Unit tests and simulation (no ROS):**
```sh
python3 -m pytest test/test_hull_distance.py -q
python3 tools/sim_plot_trajectory.py sim_out   # old vs new trajectory figure
python3 tools/sim_plot_return.py sim_out       # zoom on the return after a push
```

### Parameters you can adjust

The values are set in the Python files (they are not ROS parameters yet). Rebuild with `colcon build` after changing them.

**Distance to the hull**: `hull_guider/core/dvl_hull_node.py`, where `self.hull_dist = HullDistanceEstimator(...)` is created:

| Parameter | Default | What it does | When to change it |
|---|---|---|---|
| `top_beam_indices` | `[2, 3]` | The two beams that face the hull (0-based) | If the DVL is mounted differently |
| `gate` | `0.15` m | Readings further than this from the estimate are rejected | Larger if real changes get rejected; smaller if reflections get through |
| `timeout` | `3.0` s | Time without an accepted reading before the distance is marked invalid | Smaller to hold sooner; larger if the ROV holds too often |
| `backup_beams` | `(0, 1)` | Lower beams used when both top beams are lost | `()` to turn the backup off |
| `max_range` | `2.9` m | Readings at or beyond this count as "no hull" | Just below the DVL range limit (the range mode is set to 3 m) |
| `max_rejects` | `3` | Minimum number of rejected readings before a change can be accepted | Rarely |

More settings in `hull_guider/core/hull_distance.py` (pass them in the same call to change them):

| Parameter | Default | What it does |
|---|---|---|
| `reset_time` | `1.5` s | How long a sudden change must stay consistent before it is accepted. Shorter reacts faster but lets longer reflections through (reflections in the bags lasted 0.5–1 s) |
| `max_angle_deg` | `35` ° | Largest believable hull angle. Beyond it the two beams disagree and only the one closest to the estimate is used |
| `angle_alpha` | `0.3` | Smoothing of the hull angle (higher = follows turns faster, noisier) |
| `meas_std_two` / `meas_std_one` / `meas_std_backup` | `0.02` / `0.04` / `0.06` m | Expected noise with two top beams / one top beam / lower beams. Higher = smoother but slower |
| `accel_std` | `0.1` m/s² | How quickly the distance is expected to change. Higher = follows faster, noisier |

**Controller**: `hull_guider/core/guided_controller.py`:

| Parameter | Default | What it does |
|---|---|---|
| `distance_threshold` | `0.8` m | Target distance from the hull |
| `distance_deadband` | `0.05` m | No proportional correction within this distance of the target (was 0.15) |
| `k_dist` (k_p) | `0.5` | Speed toward or away from the hull per metre of error |
| `k_d` | `0.8` | Damping; reduces overshoot |
| `k_i` | `0.01` | Integral term; removes a steady offset (e.g. from a current). Set to `0` to turn it off |
| `i_max` | `0.1` m/s | Largest push from the integral term |
| `i_zone` | `0.3` m | The integral only builds up when this close to the target |
| `alpha_dist` | `0.15` | Extra smoothing of the distance in the controller |
| `k_tangent` | `0.3` m/s | Speed along the hull |
| `too_far_distance` / `tangent_dampen_factor` | `0.8` m / `0.3` | When further than this from the target, the speed along the hull is cut to 30% |

---


Visualize rviz2:

 "/viz/pose"

 "/dvl/points"

 "/viz/plane_normal"
 
 "/viz/perp_distance"
 
 

Get the GCS Client Link first from BlueOS

 
On QGroundControl side : set parameters :

Check value of parameters.


EK3_ENABLE = 1
EK2_ENABLE = 0   (optional but recommended)
AHRS_EKF_TYPE = 3

EK3_SRC1_POSXY = 6
<!-- EK3_SRC1_POSZ  = 6 --> depth can just use barometers
EK3_SRC1_VELXY = 0
EK3_SRC1_VELZ  = 0
EK3_SRC1_YAW   = 1
<!-- EK3_SRC1_ROLL  = 1 -->
<!-- EK3_SRC1_PITCH = 1  -->

6 - external

1 - Compass/IMU






Calibrate gyro if needed


{"command":"calibrate_gyro"}

echo '{"command":"calibrate_gyro"}' | nc -q0 192.168.2.95 16171


Terminal command to set range mode :

0.3 - 3 m :

echo '{"command":"set_config","parameters":{"range_mode":"=1"}}' | nc -q0 192.168.2.95 16171

0.0 - 3 m: 

echo '{"command":"set_config","parameters":{"range_mode":"0<=1"}}' | nc -q0 192.168.2.95 16171




## Commands

Start DVL pinger


```sh
cd ~/ros2_ws
python pinger.py
```

Publish DVL 

```sh
# start dvl ros topics
ros2 run dvl_a50 dvl_a50.py --ros-args -p dvl_address:='192.168.2.95' -p client_address:='192.168.2.5'
```


Launch mavros

```sh
ros2 launch mavros apm.launch \
    fcu_url:=udp://0.0.0.0:14551@192.168.2.2:14550 

# Launch mavros so that we can see the error messages in case guided mode fails. e.g. Cannot start 'guided' mode as there is no position given.
```

PUBLISH ROBOT STATE TO RVIZ


```sh
ros2 run rviz2 rviz2 --ros-args -d ~/.rviz2/guider.rviz

rviz2 -d ~/.rviz2/guider.rviz

# OR small windows with picture window

rviz2 -d ~/.rviz2/guider-small.rviz

```

RVIZ control
- left click : ORBIT
- right click : ZOOM
- shift + left click : PAN


```sh
ros2 run robot_state_publisher robot_state_publisher ~/simple_sub.urdf
```


RUN DVL HULL, should appear visualization if using the guider.rviz

```sh
ros2 run hull_guider dvl_hull
```



```sh
ros2 run hull_guider guided_controller
```



ROSBAG RECORD:

```sh
export IT=1
ros2 bag record /mavros/imu/static_pressure /mavros/imu/data /mavros/vfr_hud /mavros/global_position/rel_alt /image_raw /dvl/data -o $(date +%d-%m)-pool-$IT

ros2 bag record /mavros/imu/static_pressure /mavros/imu/data /mavros/vfr_hud /mavros/global_position/rel_alt /image_raw /dvl/data -o mar31_pool_1


```







Visualize :

"/viz/pose"

"/dvl/points"

"/viz/plane_normal"
 
"/viz/perp_distance"



## Playing back from rosbag



```sh
ros2 run robot_state_publisher robot_state_publisher ~/simple_sub.urdf
```

```sh
rviz2 -d ~/.rviz2/guider.rviz
```

```sh
ros2 bag play mar26_pool_2
```

```sh
ros2 run hull_guider dvl_hull
```




