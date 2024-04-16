import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap
import launch
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


# launch robot related Nodes.
# - robot model
# - micro ros agent
def generate_launch_description():
    use_simulator = LaunchConfiguration("simulator")
    robot_usb_device = LaunchConfiguration("robot_usb_device", default="/dev/ttyUSB0")
    robot_model_path = LaunchConfiguration("robot_model_path")
    use_robot_odom = LaunchConfiguration("use_robot_odom", default="true")

    declare_simulator_cmd = DeclareLaunchArgument(
        "simulator",
        default_value="false",
        description="Use Simulator and do not use Livox LiDARs",
    )

    declare_robot_usb_device_cmd = DeclareLaunchArgument(
        "robot_usb_device",
        default_value="/dev/ttyUSB0",
        description="Robot's USB Device",
    )

    declare_use_robot_odom_cmd = DeclareLaunchArgument(
        "use_robot_odom",
        default_value="true",
        description="publish Robot's Odometry",
    )

    description_package_path = get_package_share_path("c_megarover_description")
    default_model_path = os.path.join(description_package_path,"urdf", "mega3.xacro")
    declare_robot_model_path = DeclareLaunchArgument(
        name="robot_model_path",
        default_value=str(default_model_path),
        description="Absolute path to robot urdf file",
    )


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

    pub_odom_node = Node(
        package="c_megarover_common",
        executable="pub_odom_node",
        name="pub_odom",
        condition=IfCondition(use_robot_odom),
        remappings=[
            ("/odom", "/megarover_odom"), # pub
            ("/rover_odo", "/rover_odo"), #sub
        ],
    )

    # ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 115200
    micro_ros_agent_node = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        arguments=["serial", "--dev", robot_usb_device, "--baudrate", "115200"],
        condition=UnlessCondition(use_simulator),
    )

    return LaunchDescription(
        [
            declare_simulator_cmd,
            declare_robot_usb_device_cmd,
            declare_use_robot_odom_cmd,
            declare_robot_model_path,
            joint_state_publisher_node,
            robot_state_publisher_node,
            pub_odom_node,
            micro_ros_agent_node,
        ]
    )
