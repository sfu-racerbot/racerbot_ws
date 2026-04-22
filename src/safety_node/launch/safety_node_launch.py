from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch safety_node
        Node(
            package='safety_node',
            executable='safety_node',
            name='safety_node',
            output='screen',
        ),
    ])
