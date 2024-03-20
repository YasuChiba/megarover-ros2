

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
source install/setup.sh
```

2. run
```
ros2 launch c_megarover msg_MID360_launch.py
ros2 launch c_megarover create_3dmap_launch.py rviz:=true

```

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


