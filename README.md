

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
SLAMの結果の点群を、rosbagから取り出すにはros2bag_toolsを利用。  
https://github.com/AIT-Assistive-Autonomous-Systems/ros2bag_tools#export

`export.config`  
```
cut --start 200
extract -t /Laser_map
```

`ros2 bag process -c export.config ../rosbag2_2024_03_20-02_49_47/ -o out.bag`  
`ros2 bag export --in ./out.bag/ -t /Laser_map pcd`






# memo

## slam_toolbox
slam_toolboxはodomが必須なので、create_2dmap_launch.pyはodomがある環境で試す。


