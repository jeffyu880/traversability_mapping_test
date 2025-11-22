from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'velodyne']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '1.570795', '0', '1.570795', 'map', 'camera_init']
        ),
        
        # Traversability mapping nodes
        Node(
            package='traversability_mapping',
            executable='traversability_filter',
            output='screen'
        ),
        Node(
            package='traversability_mapping',
            executable='traversability_map',
            output='screen'
        ),
        Node(
            package='traversability_mapping',
            executable='traversability_prm',
            output='screen'
        ),
        Node(
            package='traversability_mapping',
            executable='traversability_path',
            output='screen'
        )
    ])
