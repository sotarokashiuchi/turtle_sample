from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtle_executor',
            namespace='test',
            executable='turtle_executor',
            name='turtle_executor',
            output='screen'
        ),
    ])
