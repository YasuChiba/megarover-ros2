import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
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
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    package_path = get_package_share_directory("c_megarover")
    config_dir_path = os.path.join(package_path, "config")
    launch_dir_path = os.path.join(package_path, "launch")

    rviz_use = LaunchConfiguration("rviz", default=True)
    use_simulator = LaunchConfiguration(
        "simulator", default=False
    )  # using simulator or rosbag to publish lidar data

    map_file_path = LaunchConfiguration("map_file_path")
    map_2d_file_path = LaunchConfiguration("map_2d_file_path")

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

    declare_map_2d_file_path = DeclareLaunchArgument(
        name="map_2d_file_path",
        default_value="",
        description="path for map file (.yaml)",
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir_path, "/3d_localization_launch.py"]),
        launch_arguments={
            "rviz": "false",
            "simulator": use_simulator,
            "map_file_path": map_file_path,
        }.items(),
    )

    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_bringup_launch_dir = os.path.join(nav2_bringup_dir, "launch")

    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    use_composition = LaunchConfiguration("use_composition")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")
    use_respawn = LaunchConfiguration("use_respawn")
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )
    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace",
        default_value="false",
        description="Whether to apply a namespace to the navigation stack",
    )
    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Whether to use composed bringup",
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(config_dir_path, "nav2.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )
    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )
    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    params_file = ReplaceString(
        source_file=params_file,
        replacements={"<robot_namespace>": ("/", namespace)},
        condition=IfCondition(use_namespace),
    )
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    lifecycle_nodes = ["map_server"]

    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),
            SetRemap(src="/cmd_vel", dst="/rover_twist"),
            Node(
                condition=IfCondition(use_composition),
                name="nav2_container",
                package="rclcpp_components",
                executable="component_container_isolated",
                parameters=[
                    configured_params,
                    {"autostart": autostart},
                    {"use_sim_time": use_simulator},
                ],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                output="screen",
            ),
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="nav2_map_server",
                        executable="map_server",
                        name="map_server",
                        output="screen",
                        respawn=False,
                        respawn_delay=2.0,
                        parameters=[
                            {"yaml_filename": map_2d_file_path},
                            {"frame_id": "map"},
                            {"topic_name": "map_2d"},
                        ],
                    ),
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {"use_sim_time": use_simulator},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_launch_dir, "navigation_launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_simulator,
                    "autostart": autostart,
                    "params_file": params_file,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "container_name": "nav2_container",
                    "map_file_path": map_file_path,
                }.items(),
            ),
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        # condition=IfCondition(rviz_use),
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("c_megarover"),
                "rviz",
                "3d_navigation.rviz",
            ),
        ],
    )

    return LaunchDescription(
        [
            declare_rviz_cmd,
            declare_simulator_cmd,
            declare_map_file_path,
            declare_map_2d_file_path,
            declare_namespace_cmd,
            declare_use_namespace_cmd,
            declare_use_composition_cmd,
            declare_params_file_cmd,
            declare_autostart_cmd,
            declare_log_level_cmd,
            declare_use_respawn_cmd,
            localization,
            bringup_cmd_group,
            rviz_node,
        ]
    )
