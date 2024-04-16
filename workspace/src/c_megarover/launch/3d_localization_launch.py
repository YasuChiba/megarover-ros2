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

    rviz_use = LaunchConfiguration("rviz")
    use_simulator = LaunchConfiguration("simulator")
    map_file_path = LaunchConfiguration("map_file_path")

    declare_rviz_cmd = DeclareLaunchArgument(
        "rviz", default_value="true", description="Use RViz to monitor results"
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        "simulator",
        default_value="true",
        description="Use Simulator/rosbag and do not use actual Livox LiDARs. it also set to use_sim_time.",
    )

    declare_map_file_path = DeclareLaunchArgument(
        name="map_file_path",
        default_value="",
        description="path for map file (.pcd)",
    )

    # launch robot_launch.py
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir_path, "/robot_launch.py"]),
        launch_arguments={
            "simulator": use_simulator,
            "use_robot_odom": "true",
        }.items(),
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir_path, "/lidar_launch.py"]),
        launch_arguments={
            "xfer_format": "0",
            "lidar_config_path": os.path.join(config_dir_path, "MID360_config.json"),
        }.items(),
        condition=UnlessCondition(use_simulator),
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
            {"file_name": map_file_path},
            {"tf_frame": "map"},
            {"publishing_period_ms": 1000},
        ],
        remappings=[("/cloud_pcd", "/global_map")],
    )

    # launch fast_lio node.`
    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        output="screen",
        parameters=[
            PathJoinSubstitution([config_dir_path, "3dlocalization.yaml"]),
            {"use_sim_time": use_simulator},
        ],
        remappings=[
            ("/Odometry", "/fastlio_odom"),
        ],
    )

    global_localization_node = Node(
        package="c_megarover_localization",
        executable="global_localization",
        name="global_localization",
        output="screen",
        parameters=[
            PathJoinSubstitution([config_dir_path, "3dlocalization.yaml"]),
            {"map_file_path": map_file_path},
            {"use_sim_time": use_simulator},
        ],
        remappings=[
            ("/cloud_registered", "/cloud_registered"),  # sub
            ("/Odometry", "/fastlio_odom"),  # sub
            ("/cur_scan_in_map", "/cur_scan_in_map"),  # pub
            ("/submap", "/submap"),  # pub
            ("/map_to_odom", "/map_to_odom"),  # pub
        ],
    )

    transform_fusion_node = Node(
        package="c_megarover_localization",
        executable="transform_fusion",
        name="transform_fusion",
        output="screen",
        parameters=[
            PathJoinSubstitution([config_dir_path, "3dlocalization.yaml"]),
            {"use_sim_time": use_simulator},
        ],
        remappings=[
            ("/Odometry", "/fastlio_odom"),  # sub
            ("/map_to_odom", "/map_to_odom"),  # sub
            ("/localization", "/localization"),  # pub
        ],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([config_dir_path, "3dlocalization.yaml"]),
            {"use_sim_time": use_simulator},
        ],
        remappings=[
            ("/odometry/filtered", "/odometry/filtered"), # pub
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
                parameters=[
                    {"use_sim_time": use_simulator},
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
            declare_map_file_path,
            robot_launch,
            lidar_launch,
            pointcloud_filter_node,
            fast_lio_node,
            global_localization_node,
            transform_fusion_node,
            pcd_to_pointcloud_node,
            ekf_node,
            rviz_node,
        ]
    )
