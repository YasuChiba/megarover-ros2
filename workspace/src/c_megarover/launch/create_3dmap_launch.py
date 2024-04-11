import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_path = get_package_share_directory("c_megarover")
    config_dir_path = os.path.join(package_path, "config")
    launch_dir_path = os.path.join(package_path, "launch")

    use_sim_time = LaunchConfiguration("use_sim_time", default=False)
    rviz_use = LaunchConfiguration("rviz", default=True)
    use_simulator = LaunchConfiguration('simulator', default=False) # using simulator or rosbag to publish lidar data

    declare_rviz_cmd = DeclareLaunchArgument(
        "rviz", default_value="true",
        description="Use RViz to monitor results"
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        "simulator", default_value="false",
        description="Use Simulator/rosbag and do not use Livox LiDARs"
    )

    # launch lidar_launch.py
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir_path, "/lidar_launch.py"]),
        launch_arguments={
            "xfer_format": "0",
            "lidar_config_path": os.path.join(config_dir_path, "MID360_config.json")
        }.items(),
        condition=UnlessCondition(use_simulator)
    )

    # launch robot_launch.py
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir_path, "/robot_launch.py"]),
        launch_arguments={
            "simulator": use_simulator
        }.items()
    )

    # launch pointcloud_filter_node
    pointcloud_filter_node = Node(
        package="c_megarover_common",
        executable="pointcloud_filter_node",
        output="screen",
        remappings=[
            ("/in_cloud", "/livox/lidar"),
            ("/out_cloud", "/livox/filtered_lidar")
        ]
    )

    # launch fast_lio node.`
    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        output="screen",
        parameters=[
            PathJoinSubstitution([config_dir_path, "3dslam_config.yaml"]),
            {"use_sim_time": use_sim_time},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(rviz_use),
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("c_megarover"), "config", "3dslam.rviz"
            ),
        ],
    )

    return LaunchDescription(
        [
            declare_rviz_cmd,
            declare_simulator_cmd,
            robot_launch,
            lidar_launch,
            pointcloud_filter_node,
            fast_lio_node,
            rviz_node
        ]
    )
