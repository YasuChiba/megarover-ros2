import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

################### user configure parameters for ros2 start ###################
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    rviz_use = LaunchConfiguration('rviz')
    simulator_use = LaunchConfiguration('simulator')

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        'simulator', default_value='false',
        description='Use Simulator and do not use Livox LiDARs'
    )

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params,
        condition=UnlessCondition(simulator_use)
        )

    # create pointcloud_to_laserscan Node
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[('cloud_in','/livox/lidar'),
                        ('scan','/scan')],
        parameters=[{
            'target_frame': '',
            'transform_tolerance': 0.01,
            'min_height': 0.0,
            'max_height': 1.0,
            'angle_min': -3.1415,  # -M_PI/2
            'angle_max': 3.1415,  # M_PI/2
            'angle_increment': 0.0087,  # M_PI/360.0
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 20.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }]
    )
    
    # launch slam_toolbox's `online_sync_launch.py`
    #slam_toolboxの起動オプション設定
    slam_params_file = LaunchConfiguration('slam_params_file')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("c_megarover"),
                                   'config', '2dslam_config.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    #slam_toolboxの起動設定
    #start_async_slam_toolbox_node = Node(
    #    parameters=[
    #      slam_params_file,
    #      {'use_sim_time': use_sim_time}
    #    ],
    #    package='slam_toolbox',
    #    executable='sync_slam_toolbox_node',
    #    name='slam_toolbox',
    #    output='screen')

    slam_toolbox = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_sync_launch.py')]
            ),
            launch_arguments=[
                ('slam_params_file', slam_params_file),
                ('use_sim_time', use_sim_time),
            ])

    # add static_transform_publisher.
    static_transform_publisher1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'livox_frame'],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(rviz_use),
        arguments=['-d', os.path.join(get_package_share_directory('c_megarover'), 'config', '2dslam.rviz')],
    )

    

    return LaunchDescription([
        static_transform_publisher1,
        livox_driver,
        pointcloud_to_laserscan,
        declare_slam_params_file_cmd,
        slam_toolbox,
        rviz_node,
        declare_simulator_cmd,
        declare_rviz_cmd
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=livox_rviz,
        #         on_exit=[
        #             launch.actions.EmitEvent(event=launch.events.Shutdown()),
        #         ]
        #     )
        # )
    ])