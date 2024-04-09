import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition


def generate_launch_description():

    package_path = get_package_share_directory('c_megarover')
    default_config_path = os.path.join(package_path, 'config')
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    rviz_use = LaunchConfiguration('rviz')
    rosbag_path = LaunchConfiguration('rosbag_path')

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )

    declare_rosbagpath_cmd = DeclareLaunchArgument(
        'rosbag_path', default_value='',
        description='rosbag path'
    )

 
    # add static_transform_publisher.
    static_transform_publisher1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'livox_frame'],
        output='screen'
    )

    # add static_transform_publisher.
    static_transform_publisher2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'camera_init'],
        output='screen'
    )

    # launch fast_lio node.`
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        output='screen',
        parameters=[PathJoinSubstitution([default_config_path, "3dslam_config_rosbag.yaml"]),
                    {'use_sim_time': use_sim_time}],
    )

    # launch livox_to_pointcloud2 node.
    livox_to_pointcloud2_node = Node(
        package='livox_to_pointcloud2',
        executable='livox_to_pointcloud2_node',
        output='screen',
        remappings=[('livox_pointcloud','/livox/lidar'),
                        ('converted_pointcloud2','/livox/pointcloud2')],
    )

    pointcloud2_filter = Node(
        package='ros2_rs_pcl',
        executable='rs_pcl_filter',
        output='screen',
        remappings=[('in_cloud','/livox/pointcloud2'),
                    ('out_cloud','/livox/filtered_pointcloud2')],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(rviz_use),
        arguments=['-d', os.path.join(get_package_share_directory('c_megarover'), 'config', '3dslam.rviz')],
    )


    rosbag_play = launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'play', rosbag_path, "--topics", "/livox/imu", "/livox/lidar", "-r", "1.5"],
            output='screen'
    )

    return LaunchDescription([
        declare_rviz_cmd,
        declare_rosbagpath_cmd,
        static_transform_publisher1,
        static_transform_publisher2,
        fast_lio_node,
        livox_to_pointcloud2_node,
        pointcloud2_filter,
        rosbag_play,
        rviz_node
    ])