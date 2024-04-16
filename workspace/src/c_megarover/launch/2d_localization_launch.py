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
from launch_ros.actions import Node, SetRemap
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
import lifecycle_msgs.msg


def generate_launch_description():
    package_path = get_package_share_directory("c_megarover")
    config_dir_path = os.path.join(package_path, "config")
    launch_dir_path = os.path.join(package_path, "launch")

    rviz_use = LaunchConfiguration("rviz", default=True)
    use_simulator = LaunchConfiguration(
        "simulator", default=True
    )  # using simulator or rosbag to publish lidar data

    map_file_path = LaunchConfiguration("map_file_path")
    map_2d_file_path = LaunchConfiguration("map_2d_file_path")

    declare_rviz_cmd = DeclareLaunchArgument(
        "rviz", default_value="true", description="Use RViz to monitor results"
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        "simulator",
        default_value="true",
        description="Use Simulator/rosbag and do not use Livox LiDARs",
    )

    declare_map_file_path = DeclareLaunchArgument(
        name="map_file_path",
        default_value="",
        description="path for map file (.pcd)",
    )

    declare_map_2d_file_path = DeclareLaunchArgument(
        name="map_2d_file_path",
        default_value="",
        description="path for map file (.yaml)",
    )

    # launch robot_launch.py
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir_path, "/robot_launch.py"]),
        launch_arguments={
            "simulator": use_simulator,
            "broadcast_robot_odom": "true",
        }.items(),
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

    # create pointcloud_to_laserscan Node
    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        remappings=[("cloud_in", "/livox/lidar"), ("scan", "/scan")],
        parameters=[
            {
                "target_frame": "livox_frame",
                "transform_tolerance": 0.01,
                "min_height": 0.0,
                "max_height": 1.0,
                "angle_min": -3.1415,  # -M_PI/2
                "angle_max": 3.1415,  # M_PI/2
                "angle_increment": 0.0087,  # M_PI/360.0
                "scan_time": 0.3333,
                "range_min": 0.1,
                "range_max": 20.0,
                "use_inf": True,
                "inf_epsilon": 1.0,
            }
        ],
    )

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch"
    )
    nav2_launch = GroupAction(
        actions=[
            SetRemap(src="/cmd_vel", dst="/rover_twist"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [nav2_launch_file_dir, "/bringup_launch.py"]
                ),
                launch_arguments={
                    "map": map_2d_file_path,
                    "use_sim_time": use_simulator,
                    "params_file": os.path.join(config_dir_path, "nav2_2d.yaml"),
                }.items(),
            ),
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
                        "2d_navigation.rviz",
                    ),
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            launch_ros.actions.SetParameter(name='use_sim_time', value=use_simulator),
            declare_rviz_cmd,
            declare_simulator_cmd,
            declare_map_file_path,
            declare_map_2d_file_path,
            robot_launch,
            pcd_to_pointcloud_node,
            pointcloud_to_laserscan,
            nav2_launch,
            rviz_node,
        ]
    )
