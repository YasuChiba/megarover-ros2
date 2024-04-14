import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import launch
import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

import lifecycle_msgs.msg

launch_args_for_octomap = [
    DeclareLaunchArgument("pointcloud_map_topic", default_value="/pcd_map"),
    DeclareLaunchArgument("resolution", default_value="0.02"),
    DeclareLaunchArgument("frame_id", default_value="map"),
    DeclareLaunchArgument("base_frame_id", default_value="base_footprint"),
    DeclareLaunchArgument("height_map", default_value="True"),
    DeclareLaunchArgument("colored_map", default_value="False"),
    DeclareLaunchArgument("compress_map", default_value="True"),
    DeclareLaunchArgument("publish_free_space", default_value="False"),
]


def generate_launch_description():
    package_path = get_package_share_directory("c_megarover")
    config_dir_path = os.path.join(package_path, "config")
    launch_dir_path = os.path.join(package_path, "launch")

    use_sim_time = LaunchConfiguration("use_sim_time", default=False)
    rviz_use = LaunchConfiguration("rviz", default=True)
    use_simulator = LaunchConfiguration(
        "simulator", default=True
    )  # using simulator or rosbag to publish lidar data

    declare_rviz_cmd = DeclareLaunchArgument(
        "rviz", default_value="true", description="Use RViz to monitor results"
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        "simulator",
        default_value="true",
        description="Use Simulator/rosbag and do not use Livox LiDARs",
    )

    octmap_node = Node(
        package="octomap_server2",
        executable="octomap_server",
        output="screen",
        remappings=[
            ("cloud_in", LaunchConfiguration("pointcloud_map_topic")),
            ("projected_map", "projected_map"),
        ],
        parameters=[
            {
                "resolution": LaunchConfiguration("resolution"),
                "frame_id": LaunchConfiguration("frame_id"),
                "base_frame_id": LaunchConfiguration("base_frame_id"),
                "height_map": LaunchConfiguration("height_map"),
                "colored_map": LaunchConfiguration("colored_map"),
                "compress_map": LaunchConfiguration("compress_map"),
                "publish_free_space": LaunchConfiguration("publish_free_space"),
                "pointcloud_min_z": 0.0,
                "pointcloud_max_z": 1.0,
            }
        ],
    )


    # delay 3 sec to wait for robot to be ready
    rviz_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                condition=IfCondition(rviz_use),
                arguments=[
                    "-d",
                    os.path.join(
                        get_package_share_directory("c_megarover"),
                        "rviz",
                        "3d_localization.rviz",
                    ),
                ],
            )
        ],
    )

    return LaunchDescription(
        launch_args_for_octomap
        + [
            declare_rviz_cmd,
            declare_simulator_cmd,
            octmap_node,
            rviz_node,
        ]
    )
