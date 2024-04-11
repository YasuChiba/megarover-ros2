import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    container = ComposableNodeContainer(
            name='common_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # composable node for FilterNode
                ComposableNode(
                    package='c_megarover_common',
                    plugin='pointcloud_filter::FilterNode',
                    name='filter_node',
                    extra_arguments=[{'use_intra_process_comms': True}],
                    remappings=[('/in_cloud', '/livox/lidar')]
                ),
                # composable node for livox_to_pointcloud2
                ComposableNode(
                    package='c_megarover_common',
                    plugin='LivoxToPointCloud2',
                    name='LivoxToPointCloud2',
                    extra_arguments=[{'use_intra_process_comms': True}]
                )

            ],
            output='screen'
    )

    return launch.LaunchDescription([container])
