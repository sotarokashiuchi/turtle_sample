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
            package='turtle_parameter',
            namespace='test',
            executable='turtle_parameter',
            name='turtle_parameter',
            parameters=[{
                'Kp': 0.2,
                'ref_x': 8.0,
                'ref_y': 1.0,
            }],
            output='screen'
        ),
    ])
