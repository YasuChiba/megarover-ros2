import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    package_path = get_package_share_directory("c_megarover")
    default_lidar_config_path = os.path.join(package_path, "config", "MID360_config.json")

    xfer_format = LaunchConfiguration("xfer_format", default=0)
    lidar_config_path = LaunchConfiguration(
        "lidar_config_path", default=default_lidar_config_path
    )

    declare_xfer_format = DeclareLaunchArgument(
        "xfer_format",
        default_value="0",
        description="0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format",
    )
    declare_lidar_config_path = DeclareLaunchArgument(
        "lidar_config_path",
        default_value=default_lidar_config_path,
        description="Absolute path to LiDAR config json file",
    )
    
    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[
            {"xfer_format": xfer_format},
            {
                "multi_topic": 0
            },  # 0-All LiDARs share the same topic, 1-One LiDAR one topic
            {"data_src": 0},  # 0-lidar, others-Invalid data src
            {"publish_freq": 10.0},  # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
            {"output_data_type": 0},
            {"frame_id": "livox_frame"},
            {"user_config_path": lidar_config_path},
        ],
    )

    return LaunchDescription(
        [
            declare_xfer_format,
            declare_lidar_config_path,
            livox_driver,
        ]
    )
