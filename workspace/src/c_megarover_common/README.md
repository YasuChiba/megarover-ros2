


## pointcloud_to_pcd

ref: https://ar-ray.hatenablog.com/entry/2024/04/06/170930  
ref: https://github.com/ros-perception/perception_pcl/blob/ros2/pcl_ros/tools/pointcloud_to_pcd.cpp  
```
ros2 run c_megarover_common pointcloud_to_pcd_node --ros-args -r input:=/Laser_map
```

## pcd_to_pointcloud

ref: https://github.com/ros-perception/perception_pcl/blob/ros2/pcl_ros/tools/pcd_to_pointcloud.cpp  
```
ros2 run c_megarover_common pcd_to_pointcloud_node --ros-args -p file_name:=pcd/sendagi.pcd -p tf_frame:=livox_frame
```

## pcd_to_occupancygrid_tool

convert PCD file to occupancy grid.  
ref: https://github.com/ANYbotics/grid_map/tree/humble/grid_map_pcl  


```
ros2 run c_megarover_common pcd_to_occupancygrid_tool --ros-args -p pcd_file_path:=/home/user/workspace/pcd/sendagi.pcd
```