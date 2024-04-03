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
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rviz_use = LaunchConfiguration('rviz')
    use_simulator = LaunchConfiguration('simulator')
    map_dir = LaunchConfiguration(
        'map',
        default='/home/user/workspace/map/sendagi2.yaml')
    
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('c_megarover'),
            'config',
            "nav2.yaml"))

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
        condition=UnlessCondition(use_simulator)
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
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(rviz_use),
        arguments=['-d', os.path.join(get_package_share_directory('c_megarover'), 'config', '2dslam.rviz')],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(rviz_use)
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(use_simulator)
    )


    description_package_path = get_package_share_path('c_megarover_description')
    default_model_path = description_package_path / 'urdf/mega3.xacro'

    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        condition=UnlessCondition(use_simulator)
    )

    pub_odom_node = Node(
        package='megarover3_bringup',
        executable='pub_odom',
        name='pub_odom',
        condition=UnlessCondition(use_simulator)
    )

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    nav2_launch = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel',dst='/rover_twist'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
                launch_arguments={
                    'map': map_dir,
                    'use_sim_time': use_sim_time,
                    'params_file': param_dir
                }.items(),
            )
        ]
    )

    


    

    

    return LaunchDescription([
        model_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        pub_odom_node,
        livox_driver,
        pointcloud_to_laserscan,
        rviz_node,
        declare_simulator_cmd,
        declare_rviz_cmd,
        nav2_launch,
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=livox_rviz,
        #         on_exit=[
        #             launch.actions.EmitEvent(event=launch.events.Shutdown()),
        #         ]
        #     )
        # )
    ])