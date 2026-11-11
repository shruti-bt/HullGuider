
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




