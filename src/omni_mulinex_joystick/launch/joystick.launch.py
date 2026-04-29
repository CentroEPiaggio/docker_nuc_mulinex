import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('omni_mulinex_joystick'),
        'config',
        'joystick_params.yaml',
    )

    twist_cmd_topic = LaunchConfiguration('twist_cmd_topic')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'twist_cmd_topic',
                default_value='/omni_controller/twist_cmd',
                description='Twist command topic published by the joystick teleop node',
            ),
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
            ),
            Node(
                package='omni_mulinex_joystick',
                executable='omni_mulinex_joystick_node',
                name='omni_mulinex_joystick',
                parameters=[config],
                remappings=[('/omni_controller/twist_cmd', twist_cmd_topic)],
                output='screen',
            ),
        ]
    )
