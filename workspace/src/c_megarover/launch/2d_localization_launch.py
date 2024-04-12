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

    # launch robot_launch.py
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir_path, "/robot_launch.py"]),
        launch_arguments={"simulator": use_simulator}.items(),
    )

    # publish pointcloud from file using pcd_to_pointcloud_node
    pcd_to_pointcloud_node = Node(
        package="c_megarover_common",
        executable="pcd_to_pointcloud_node",
        output="screen",
        parameters=[
            {"file_name": "/home/user/workspace/pcd/sendagi.pcd"},
            {"tf_frame": "map"},
            {"publishing_period_ms": 10000},
        ],
        remappings=[("/cloud_pcd", LaunchConfiguration("pointcloud_map_topic"))],
    )

    octmap_node = Node(
        package="octomap_server2",
        executable="octomap_server",
        output="screen",
        remappings=[("cloud_in", LaunchConfiguration("pointcloud_map_topic"))],
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

    # create pointcloud_to_laserscan Node
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[('cloud_in','/livox/lidar'),
                        ('scan','/scan')],
        parameters=[{
            'target_frame': '',
            'transform_tolerance': 0.01,
            'min_height': 0.0,
            'max_height': 1.0,
            'angle_min': -3.1415,  # -M_PI/2
            'angle_max': 3.1415,  # M_PI/2
            'angle_increment': 0.0087,  # M_PI/360.0
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 20.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }]
    )

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    nav2_launch = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel',dst='/rover_twist'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
                launch_arguments={
                    'map': map_dir,
                    'use_sim_time': use_sim_time,
                    'params_file': param_dir
                }.items(),
            )
        ]
    )

    # delay 3 sec to wait for robot to be ready
    rviz_node = TimerAction(
        period=0.0,
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
            robot_launch,
            #pcd_to_pointcloud_node,
            #octmap_node,
            pointcloud_filter_node,
            pcl_localization,
            rviz_node,
        ]
    )
