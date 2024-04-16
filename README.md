

`xhost +local:`を実行して置かないとrvizが使えないかも?

1. clone
submoduleごとclone
```
git clone --recursive https://github.com/YasuChiba/megarover-ros2.git
```

1. build
```
cd workspace
colcon build
colcon build --cmake-args -DBUILD_TESTING=OFF
source install/setup.sh
pip3 install transforms3d
```

2. run
```
ros2 launch c_megarover msg_MID360_launch.py
ros2 launch c_megarover create_3dmap_launch.py rviz:=true
ros2 launch c_megarover create_2dmap_launch.py simulator:=false rviz:=false
ros2 launch c_megarover create_3dmap_rosbag_launch.py  rviz:=true rosbag_path:=/home/user/workspace/rosbag/rosbag2_2024_04_09-03_27_26
ros2 launch c_megarover create_3dmap_rosbag_launch.py  rviz:=true rosbag_path:=/home/user/workspace/rosbag/rosbag2_2024_04_10-01_35_06/ | grep -v "Failed to find match for fiel"

```

micro ros agentの開始.これを実行することで、車体側とROS2経由で通信が出来るようになる。    
dockerコンテナをUSB接続前から立ち上げていると制御基板がコンテナ内から見えないかも.  
permissionで怒られたら`sudo chmod 666 /dev/ttyUSB0`を実行。  
```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 115200
```
見えるトピックは以下のはず  
- /rover_odo
- /rover_sensor
- /rover_twist



3. record/play

    ```
    ros2 bag record -a
    ros2 bag play hoge
    ```

4. export pcd
SLAMの結果の点群を、rosbagから取り出すには....
    ```
    ros2 run c_megarover_common pointcloud_to_pcd_node --ros-args -r input:=/Laser_map
    ```


## Localization

1. run
    ```
    ros2 launch c_megarover 3d_localization_launch.py simulator:=true rviz:=true map_file_path:=/home/user/workspace/pcd/sendagi.pcd

    ros2 bag play rosbag/rosbag2_2024_04_12-03_10_57/ --topics /livox/lidar /livox/imu --clock
    ```
    `--clock` is required when the `simulator:=true`.
    

## Navigation  

1. run
    ```
    ros2 launch c_megarover navigation_launch.py simulator:=true rviz:=false map_file_path:=/home/user/workspace/maps/sendagi.pcd map_2d_file_path:=/home/user/workspace/maps/sendagi.yaml

    ros2 bag play rosbag/rosbag2_2024_04_12-03_10_57/ --topic /livox/imu /livox/lidar /rover_odo --clock
    ```



# memo

## slam_toolbox
slam_toolboxはodomが必須なので、create_2dmap_launch.pyはodomがある環境で試す。


