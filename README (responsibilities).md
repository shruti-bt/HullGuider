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

IMAGE PUBLISHING

```sh
ros2 launch publish_image my_image_publisher.py
```

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