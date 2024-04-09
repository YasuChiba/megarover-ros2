modified

```
        // intensity field
        output.fields[3].name = "reflectivity";
        output.fields[3].offset = 12;
        output.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        output.fields[3].count = 1;
```


# livox_to_pointcloud2

ROS2 node to convert customized pointcloud data in livox_ros_driver2 to sensor_msgs/PointCloud2 type messages
https://github.com/Livox-SDK/livox_ros_driver2
