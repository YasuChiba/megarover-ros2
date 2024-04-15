import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory("c_megarover")
    config_dir_path = os.path.join(package_path, "config")
    launch_dir_path = os.path.join(package_path, "launch")

    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
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
        launch_arguments={
            "simulator": use_simulator,
            "use_robot_odom": "false",
        }.items(),
    )

    # launch pointcloud_filter_node
    pointcloud_filter_node = Node(
        package="c_megarover_common",
        executable="pointcloud_filter_node",
        output="screen",
        remappings=[
            ("/in_cloud", "/livox/lidar"),
            ("/out_cloud", "/livox/filtered_lidar"),
        ],
    )

    # publish pointcloud from file using pcd_to_pointcloud_node
    pcd_to_pointcloud_node = Node(
        package="c_megarover_common",
        executable="pcd_to_pointcloud_node",
        output="screen",
        parameters=[
            {"file_name": "/home/user/workspace/pcd/sendagi.pcd"},
            {"tf_frame": "map"},
            {"publishing_period_ms": 1000},
        ],
        remappings=[("/cloud_pcd", "/global_map")],
    )

    # launch fast_lio node.`
    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        output="screen",
        parameters=[
            PathJoinSubstitution([config_dir_path, "3dlocalization.yaml"]),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("/Odometry", "/fastlio_odom"),
        ],
    )

    global_localization_node = Node(
        package="c_megarover_localization",
        executable="global_localization",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            # ("/initialpose", "/initialpose"),
            # ("/lio_odom", "/fastlio_odom"),
            # ("/keyframe_scan", "/livox/lidar"),
            ("/cloud_registered", "/cloud_registered"),  # sub
            ("/Odometry", "/fastlio_odom"),  # sub
            ("/map", "/global_map"),  # sub
            ("/cur_scan_in_map", "/cur_scan_in_map"),  # pub
            ("/submap", "/submap"),  # pub
            ("/map_to_odom", "/map_to_odom"),  # pub
        ],
    )

    transform_fusion_node = Node(
        package="c_megarover_localization",
        executable="transform_fusion",
        output="screen",
        remappings=[
            # ("/lio_odom", "/fastlio_odom"),
            # ("/map_to_odom", "/map_to_odom"),
            # ("/localization", "/localization"),
            ("/Odometry", "/fastlio_odom"),  # sub
            ("/map_to_odom", "/map_to_odom"),  # sub
            ("/localization", "/localization"),  # pub
        ],
    )

    # delay 3 sec to wait for robot to be ready
    rviz_node = TimerAction(
        period=0.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                condition=IfCondition(rviz_use),
                parameters=[
                    {"use_sim_time": use_sim_time},
                ],
                arguments=[
                    "-d",
                    os.path.join(
                        get_package_share_directory("c_megarover"),
                        "rviz",
                        "3d_localization_fastlio.rviz",
                    ),
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            declare_rviz_cmd,
            declare_simulator_cmd,
            robot_launch,
            pointcloud_filter_node,
            fast_lio_node,
            global_localization_node,
            transform_fusion_node,
            pcd_to_pointcloud_node,
            rviz_node,
        ]
    )
