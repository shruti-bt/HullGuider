1. The position and IMU orientation does it reckon consistently or is it always twisting ??? 


2. perhaps the queue is too large. keep all at 1 or 2.
- FIXED

3. The lag that happens could be due to a very high queue or ... due to a lot of mavros activity happening . probably the queue? or the individual subscription?
- FIXED

4. max_point_buffer can be played with . . . 

5. how come the normal direction keeps changing and inconsistent with the point distribution
- FIXED


6. Reduce IMU rate !!!!! IMPORTANTTT
- NOT BOTTLENECK


```sh
#in qgroundcontrol!!!
SR0_EXTRA1 = 10   # IMU
SR0_POSITION = 5
```

OR

```sh
ros2 run mavros mavsys rate --imu 10  # (sets imu/data and imu/data_raw to 10Hz).
ros2 run mavros mavsys rate --all 10 # everything

```


7. do not synchronize high rate IMU with low rate DVL
- DONE



8. After investigating and refactoring code ... here are the observations:
- we got something going
- lag is solved
- flipping signs solved
- distance seems to be ok
- BUT ... the killer is that the beams are projected in wrong places when having extreme angles
- always keep a good reference of the plane
- DVL beams can't always be relied on .. (ibid)
    - SO ... the best solution for this is to use camera pose (ZED) + depth image : sample points to project and obtain hull normal from this. (however, DVL distance here is reliable within small yaw errors)
- HOWEVER we forgot to invalidate the beam. invalidate the beam first 
- invalidation still isn't clean because of reflections from the surface. the pool is too shallow


## Mar 27 controls evaluation

1. control loop running the entire time all the time -> not healthy

2. the yaw error being very noisy. we should smoothen it . [CANdo via code] 

3. the priority alignemnt and multi axis PID

4. is the PID tuning correct? 

5. maybe thruster control is not the way forward . . . we gotta use GUIDED mode.

6. USING fake joystick -

next : record /mavros/vfr_hud instead!!

and develop algorithm for with depth


## Mar 30 evaluation and possibilities . . . 

1. Bring back the Underwater GPS G2
- Need to bring a small LiPo battery and remember to charge
- will this require its own umbrella and shade? how to cool it down? 
- it may cause bottlenecks : look at the APIs, reduce rate / throughput?
- figure out where to put the baselines
- is GPS required or is relative ok?
- however, to make the relative baseline RELIABLE, we must disregard absolute GPS and rely on relative position; AND relative position of baselines to the robot must be consistent (boat cannot move too much; but this is contradictory to our design of having the boat follow the ROV)

2. Yaw correction - how?
- pose goal : requires advanced position hold etc etc use ardusub
- 


3. start planning for ZED
- it republishes the pose into mavros directly, but where should then mavros be? on the jetson? makes sense

4. 

5. TODO 1 : localization ; Initialize colmap with the positions and test on Lenovo

6. TODO 2 : fuse IMU and dvl based hull state (distance from DVL but ONLY if the angle is within the limits; angle, USE IMU (more reliable than DVL beam correction))
- use velocity control

## unrelated

TRT exec and the mask2former compilation via ONNX and how to compile the trt 








## Still very lagging

1. lagging
2. wrong point cloud somewhere
3. the points still converge to a wrong plane?



Now everything is correct, except there is lagging.

## Lagging

the hull state is working! however, there is currently a lag between the DVL readings and the actual state (reality). 

1. reduce the dvl a50 publishing queue to 1 or 2.
2. reduce subscription queue to 1 or 2.
3. do keep_last 
4. OR is there
5. the mavros setpoint controller would still work well in this case. it is keeping a stable velocity and attitude setpoint. the only problem is if distance is moving too fast - therefore, distance proportional should be reduced.


6. VERY IMPORTANT! RESPONSIBILITIES:
- MAPPING : LENOVO
- PINGER & PUBLISH DVL : LENOVO
- ROSBAG : LENOVO
- IMAGE PUBLISHING : GETAC 
- HULL STATE ESTIMATION : DELL
- AUTONOMOUS CONTROL : GETAC / DELL
- QGROUNDCONTROL + AUTONOMOUS : always the same? (GETAC / DELL)



### ASTAR INTEGRATION workflow

on LENOVO:

- `ros2 run image_saver image_streamer --ros-args -p topic_name:=/image_raw -p fps:=5 -p width:=1280 -p height:=720`
- `ros2 run image_saver prior_saver --ros-args -p img_dir:=~/saved_images/images_2 -p csv_path:=~/prior_paths/priors_1.csv -p save_images:=true -p image_topic:=/image_raw`
    - then, to choose images :
        - select images to choose and copy; create any text file and paste the complete list of images chosen inside this file. save the file
        - `python select_images.py --base_url http://localhost:8000 --output_dir ~/selected_images --colmap_path ~/datasets/trial_1apr/name_of_dataset` 
            - choose the text file of images above
            - this saves a `selected_images_{timestamp}.txt` text file containing a list of images chosen (absolute path)
            - this will also send the list of images to the ASTAR flask server
        - configure and run `auto_colmap_priored.sh` with correct arguments example:
```sh
./auto_colmap_priored.sh \
    ~/datasets_trial_1apr/propeller_1 \
    ~/selected_images/selected_images_141241241.txt \
    ~/prior_paths/priors_1.csv \
    0.2 
    # lag between prior data and image time, in seconds
```

    - once 3D map(s) are produced, paste .glb and/or .splat inside : {to be filled ...}

    

### Overview of responsibilities

- ASTAR INTEGRATION : LENOVO
    - python image_streamer.py
    - python prior_info_saver.py
    - then, to choose images :
        - python select_images.py
        - run auto_colmap_priored.sh with correct arguments 
- MAPPING : LENOVO
    - colmap OR auto_colmap_priored.sh
        - python prior_info_saver.py
        - python select_images.py
        - run auto_colmap_priored.sh with correct arguments 
    - gsplat
- PINGER & PUBLISH DVL : LENOVO
    - cd ~/ros2_ws && python pinger.py
    - ros2 run dvl_a50 dvl_a50.py --ros-args -p dvl_address:='192.168.2.95' -p client_address:='192.168.2.5'
- ROSBAG : LENOVO
    - ros2 bag record /mavros/imu/static_pressure /mavros/imu/data /mavros/vfr_hud /mavros/global_position/rel_alt /viz/pose /image_raw /dvl/data -o bag-name-here
- IMAGE PUBLISHING : GETAC
    - ros2 launch publish_image my_image_publisher.py
- MAVROS PUBLISHING : DELL
    - ros2 launch mavros apm.launch \
    fcu_url:=udp://0.0.0.0:14551@192.168.2.2:14550
- HULL STATE ESTIMATION : DELL
    - ros2 run hull_guider dvl_hull
- AUTONOMOUS CONTROL : GETAC / DELL
    - ros2 run hull_guider guided_controller
- QGROUNDCONTROL + AUTONOMOUS : always the same? (GETAC / DELL)


IMU
- where is the mavros publishing? : DELL

WIFI
- use Mobile hotspot, saves POWER STATION BATTERY

#### Lenovo

ASTAR INTEGRATION

```sh
    - python image_streamer.py
    - python prior_info_saver.py
    - then, to choose images :
        - python select_images.py
        - run auto_colmap_priored.sh with correct arguments 
```

PINGER & PUBLISH DVL

```sh
cd ~/ros2_ws
python pinger.py
```

```sh
ros2 run dvl_a50 dvl_a50.py --ros-args -p dvl_address:='192.168.2.95' -p client_address:='192.168.2.5'
```

MAPPING

```sh
colmap OR auto_colmap_priored.sh
    - python prior_info_saver.py
    - python select_images.py
    - run auto_colmap_priored.sh with correct arguments 
- gsplat
```

ROSBAG
```sh
ros2 bag record /mavros/imu/static_pressure /mavros/imu/data /mavros/vfr_hud /mavros/global_position/rel_alt /viz/pose /image_raw /dvl/data -o bag-name-here
```


#### Getac

- IMAGE PUBLISHING : GETAC
    - ros2 launch publish_image my_image_publisher.py

#### Dell

MAVROS PUBLISHING

```sh
ros2 launch mavros apm.launch \
    fcu_url:=udp://0.0.0.0:14551@192.168.2.2:14550
```

HULL STATE ESTIMATION

```sh
ros2 run hull_guider dvl_hull
```

AUTONOMOUS CONTROL

```sh
ros2 run hull_guider guided_controller
```

VISUALIZATION

```sh
ros2 run robot_state_publisher robot_state_publisher ~/simple_sub.urdf
```

```sh
rviz2 -d ~/.rviz2/guider.rviz
```

## Addresses

Addresses

GETAC
- 192.168.2.1

LENOVO
- 192.168.2.5
    - this is the client ip usually used in dvl a50 publishing

DELL
- 192.168.2.6




1. MANUAL CONTROL A STAR
- laptops used : LENOVO & GETAC

2. GUIDED MANUAL FULL SHIP RECORDING
- laptops used : GETAC (control), (DELL - hull state), LENOVO (mapping & UI)

3. RECORDED AUTONOMOUS
- laptops used : All


