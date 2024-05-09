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
from launch_ros.descriptions import ComposableNode
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration


containerName = "megarover_container"

def generate_launch_description():
    package_path = get_package_share_directory("c_megarover")
    config_dir_path = os.path.join(package_path, "config")
    launch_dir_path = os.path.join(package_path, "launch")
    
    xfer_format = 0
    lidar_config_path = os.path.join(config_dir_path, "MID360_config.json")
    
    #use_simulator = "false"
    use_simulator = LaunchConfiguration("simulator", default=False)

    robot_model_path = os.path.join(get_package_share_directory("c_megarover_description"), "urdf", "mega3.xacro")
    broadcast_robot_odom = LaunchConfiguration("broadcast_tf", default=False)

    robot_usb_device = "/dev/ttyUSB0"
    
    map_file_path = "/home/user/workspace/maps/lab.pcd"
    map_2d_file_path = "/home/user/workspace/maps/lab.yaml"
    
    nav2_params_file = os.path.join(config_dir_path, "nav2.yaml")
    autostart = LaunchConfiguration("autostart", default=True)

    nodes = []
    
    nodes.extend(lidar(xfer_format, lidar_config_path))
    nodes.extend(robot(robot_model_path, use_simulator, broadcast_robot_odom, robot_usb_device))
    nodes.extend(realsense())
    nodes.extend(localization(map_file_path, config_dir_path, use_simulator))
    nodes.extend(navigation(nav2_params_file, use_simulator, map_2d_file_path, autostart))

    container = Node(
        name=containerName,
        package='rclcpp_components',
        executable='component_container',
        output='both',
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
                # "3d_localization_fastlio.rviz",
                "3d_navigation.rviz"
            ),
        ],
    )
    
    return LaunchDescription([container] + nodes + [rviz_node])



def lidar(xfer_format, lidar_config_path):
    
    bringup_cmd_group = GroupAction(
        [
            LoadComposableNodes(
                target_container=containerName,
                composable_node_descriptions=[
                    ComposableNode(
                        package="livox_ros_driver2",
                        plugin="livox_ros::DriverNode",
                        name="livox_lidar_publisher",
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
                    ),
                    ComposableNode(
                        package="c_megarover_common",
                        plugin="PointCloudFilter",
                        name="pointcloud_filter_node",
                        parameters=[
                            {"crop_enabled": False},
                        ],
                        remappings=[
                            ("/in_cloud", "/livox/lidar"),
                            ("/out_cloud", "/livox/filtered_lidar"),
                        ],
                    ),
                ],
            ),
        ]
    )
    
    return [bringup_cmd_group]


def robot(robot_model_path, use_simulator, broadcast_robot_odom, robot_usb_device):
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", robot_model_path]), value_type=str
                )
            }
        ]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher"
    )
    
    bringup_cmd_group = GroupAction(
        [
            LoadComposableNodes(
                target_container=containerName,
                composable_node_descriptions=[
                    ComposableNode(
                        package="c_megarover_common",
                        plugin="PubOdomNode",
                        name="pub_odom",
                        parameters=[
                            {"use_simulator": use_simulator},
                            {"broadcast_tf": broadcast_robot_odom},
                            {"odom_frame_id": "odom"},
                            {"base_frame_id": "base_footprint"},
                        ],
                        remappings=[
                            ("/odom", "/megarover_odom"), # pub
                            ("/rover_odo", "/rover_odo"), #sub
                        ],
                    ),
                ],
            ),
        ]
    )

    # ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 115200
    micro_ros_agent_node = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        arguments=["serial", "--dev", robot_usb_device, "--baudrate", "115200"],
        condition=UnlessCondition(use_simulator),
    )
    
    return [
        robot_state_publisher_node, 
        joint_state_publisher_node, 
        bringup_cmd_group, 
        micro_ros_agent_node,
    ]


def realsense():
    # launch realsense node isomg rs_launch.py file in realsense2_camera package.
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py"
            )
        )
    )
    return [realsense_launch]

def localization(map_file_path, config_dir_path, use_simulator):

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

    bringup_cmd_group = GroupAction(
        [
            LoadComposableNodes(
                target_container=containerName,
                composable_node_descriptions=[
                    ComposableNode(
                        package="c_megarover_common",
                        plugin="PCDPublisher",
                        name="pcd_to_pointcloud_node",
                        parameters=[
                            {"file_name": map_file_path},
                            {"tf_frame": "map"},
                            {"publishing_period_ms": 1000},
                        ],
                        remappings=[("/cloud_pcd", "/global_map")],
                    ),
                ],
            ),
        ]
    )
    return [
        bringup_cmd_group, 
        fast_lio_node, 
        global_localization_node, 
        transform_fusion_node, 
        #ekf_node,
    ]
    


def navigation(params_file, use_simulator, map_2d_file_path, autostart):
    lifecycle_nodes = [
        "map_server",
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]
    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
        ("/cmd_vel", "/rover_twist"),
    ]
    
    namespace = ""
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )
    
    log_level = "info"
    bringup_cmd_group = GroupAction(
        [
            Node(
                name="nav2_container",
                package="rclcpp_components",
                executable="component_container_isolated",
                parameters=[configured_params, {"autostart": autostart}, {'use_sim_time': use_simulator}],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
                output="screen",
            ),
            LoadComposableNodes(
                target_container="nav2_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="nav2_map_server",
                        plugin="nav2_map_server::MapServer",
                        name="map_server",
                        parameters=[
                            configured_params,
                            {"use_sim_time": use_simulator},
                            {"yaml_filename": map_2d_file_path},
                            {"frame_id": "map"},
                            {"topic_name": "map_2d"},
                        ],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="nav2_lifecycle_manager",
                        plugin="nav2_lifecycle_manager::LifecycleManager",
                        name="lifecycle_manager_localization",
                        parameters=[
                            {
                                "use_sim_time": use_simulator,
                                "autostart": autostart,
                                "node_names": lifecycle_nodes,
                            }
                        ],
                    ),
                    ComposableNode(
                        package="nav2_controller",
                        plugin="nav2_controller::ControllerServer",
                        name="controller_server",
                        parameters=[
                            configured_params,
                            {"use_sim_time": use_simulator},
                        ],
                        remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
                    ),
                    ComposableNode(
                        package="nav2_smoother",
                        plugin="nav2_smoother::SmootherServer",
                        name="smoother_server",
                        parameters=[
                            configured_params,
                            {"use_sim_time": use_simulator},
                        ],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="nav2_planner",
                        plugin="nav2_planner::PlannerServer",
                        name="planner_server",
                        parameters=[
                            configured_params,
                            {"use_sim_time": use_simulator},
                        ],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="nav2_behaviors",
                        plugin="behavior_server::BehaviorServer",
                        name="behavior_server",
                        parameters=[
                            configured_params,
                            {"use_sim_time": use_simulator},
                        ],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="nav2_bt_navigator",
                        plugin="nav2_bt_navigator::BtNavigator",
                        name="bt_navigator",
                        parameters=[
                            configured_params,
                            {"use_sim_time": use_simulator},
                        ],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="nav2_waypoint_follower",
                        plugin="nav2_waypoint_follower::WaypointFollower",
                        name="waypoint_follower",
                        parameters=[
                            configured_params,
                            {"use_sim_time": use_simulator},
                        ],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="nav2_velocity_smoother",
                        plugin="nav2_velocity_smoother::VelocitySmoother",
                        name="velocity_smoother",
                        parameters=[
                            configured_params,
                            {"use_sim_time": use_simulator},
                        ],
                        remappings=remappings
                        + [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
                    ),
                    ComposableNode(
                        package="nav2_lifecycle_manager",
                        plugin="nav2_lifecycle_manager::LifecycleManager",
                        name="lifecycle_manager_navigation",
                        parameters=[
                            {
                                "use_sim_time": use_simulator,
                                "autostart": autostart,
                                "node_names": lifecycle_nodes,
                            }
                        ],
                    ),
                ],
            ),
        ]
    )
    return [
        TimerAction(
            period=3.0,
            actions=[bringup_cmd_group],
        ),
    ]
    