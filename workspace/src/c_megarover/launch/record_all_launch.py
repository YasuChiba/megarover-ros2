import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap
import launch
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


# record lidar topics and robot topics (such as odom)
def generate_launch_description():
    package_path = get_package_share_directory("c_megarover")
    config_dir_path = os.path.join(package_path, "config")
    launch_dir_path = os.path.join(package_path, "launch")

    rviz_use = LaunchConfiguration("rviz", default=True)
    use_simulator = LaunchConfiguration(
        "simulator", default=False
    )  # using simulator or rosbag to publish lidar data

    declare_rviz_cmd = DeclareLaunchArgument(
        "rviz", default_value="true", description="Use RViz to monitor results"
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        "simulator",
        default_value="false",
        description="Use Simulator/rosbag and do not use Livox LiDARs",
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

    # launch robot_launch.py
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir_path, "/robot_launch.py"]),
        launch_arguments={"simulator": use_simulator}.items(),
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
                        "record_all.rviz",
                    ),
                ],
            )
        ],
    )

    # generate rosbag file name based on the date
    rosbagfilename = "rosbag_" + os.popen("date +'%Y-%m-%d_%H-%M-%S'").read().strip()
    rosbag_record = launch.actions.ExecuteProcess(
        condition=UnlessCondition(use_simulator),

        # exclude theora and compressed topics
        cmd=["ros2", "bag", "record", "-o", "/home/user/workspace/rosbag/" + rosbagfilename, "-a", "-x", "(.*)theora(.*)|(.*)compressed(.*)"],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_rviz_cmd,
            declare_simulator_cmd,
            robot_launch,
            lidar_launch,
            rosbag_record,
            rviz_node,
        ]
    )
