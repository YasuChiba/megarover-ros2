

`xhost +local:`を実行して置かないとrvizが使えないかも?


1. build
```
cd workspace
colcon build
source install/setup.sh
```

2. run
```
ros2 launch c_megarover msg_MID360_launch.py
```

3. record/play

```
ros2 bag record -a
ros2 bag play hoge
```