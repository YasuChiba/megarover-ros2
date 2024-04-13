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
        launch_arguments={
            "simulator": use_simulator,
            "use_robot_odom": "false",
        }.items(),
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

    #octmap_node = Node(
    #    package="octomap_server2",
    #    executable="octomap_server",
    #    output="screen",
    #    remappings=[("cloud_in", LaunchConfiguration("pointcloud_map_topic"))],
    #    parameters=[
    #        {
    #            "resolution": LaunchConfiguration("resolution"),
    #            "frame_id": LaunchConfiguration("frame_id"),
    #            "base_frame_id": LaunchConfiguration("base_frame_id"),
    #            "height_map": LaunchConfiguration("height_map"),
    #            "colored_map": LaunchConfiguration("colored_map"),
    #            "compress_map": LaunchConfiguration("compress_map"),
    #            "publish_free_space": LaunchConfiguration("publish_free_space"),
    #            "pointcloud_min_z": 0.0,
    #            "pointcloud_max_z": 1.0,
    #        }
    #    ],
    #)

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


    pcl_localization_node = launch_ros.actions.LifecycleNode(
        name='lidar_localization',
        namespace='',
        package='lidar_localization_ros2',
        executable='lidar_localization_node',
        parameters=[PathJoinSubstitution([config_dir_path, "3dlocalization.yaml"])],
        remappings=[
            ('/velodyne_points','/livox/lidar'),
            ("/map", LaunchConfiguration("pointcloud_map_topic")),
            ("/odom", "/odom"),
            ("/imu", "/livox/imu")
        ],
        arguments=['--ros-args', '--log-level', 'warn'],
        output='screen'
    )

    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(pcl_localization_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    from_unconfigured_to_inactive = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=pcl_localization_node,
            goal_state='unconfigured',
            entities=[
                launch.actions.LogInfo(msg="-- Unconfigured --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(pcl_localization_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )

    from_inactive_to_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=pcl_localization_node,
            start_state = 'configuring',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(msg="-- Inactive --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(pcl_localization_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    pcl_localization = TimerAction(
        period=6.0,
        actions=[
            from_unconfigured_to_inactive,
            from_inactive_to_active,
            pcl_localization_node,
            to_inactive,
        ]
    )

    # launch lidar_launch.py
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir_path, "/lidar_launch.py"]),
        launch_arguments={
            "xfer_format": "0",
            "lidar_config_path": os.path.join(config_dir_path, "MID360_config.json"),
        }.items(),
        condition=UnlessCondition(use_simulator),
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
            lidar_launch,
            pcd_to_pointcloud_node,
            #octmap_node,
            pointcloud_filter_node,
            pcl_localization,
            rviz_node,
        ]
    )
