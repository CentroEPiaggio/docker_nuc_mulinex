from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_mulinex',
            executable='teleop_mulinex_node',
            name='teleop_mulinex',
            output='screen',
            prefix='xterm -e',
            parameters=[{
                'linear_step': 0.1,
                'angular_step': 0.1,
                'body_pos_step': 0.005,
                'body_ang_step': 0.02,
            }],
        ),
    ])
