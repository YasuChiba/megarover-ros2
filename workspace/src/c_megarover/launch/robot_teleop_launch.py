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
    #use_simulator = "false"
    use_simulator = LaunchConfiguration("simulator", default=False)

    robot_model_path = os.path.join(get_package_share_directory("c_megarover_description"), "urdf", "mega3.xacro")
    broadcast_robot_odom = LaunchConfiguration("broadcast_tf", default=False)

    robot_usb_device = "/dev/ttyUSB0"

    nodes = []
    
    nodes.extend(robot(robot_model_path, use_simulator, broadcast_robot_odom, robot_usb_device))
    
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

    teleop_client_node = Node(
        package="robot_teleop",
        executable="teleop_client",
        name="teleop_client",
        output="screen",
    )
    
    return LaunchDescription( nodes + [rviz_node, teleop_client_node])


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


