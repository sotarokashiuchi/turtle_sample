from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='test',
            executable='turtlesim_node',
            name='turtlesim_node',
            output='screen'
        ),
        Node(
            package='turtle_publish',
            namespace='test',
            executable='turtle_publish',
            name='turtle_publish',
            output='screen'
        ),
    ])
