import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('omni_mulinex_joystick'),
                'launch',
                'joystick.launch.py',
            )
        ),
        launch_arguments={
            'twist_cmd_topic': '/omni_controller/twist_cmd',
        }.items(),
    )

    qualisys_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('qualisys_driver'),
                'launch',
                'qualisys.launch.py',
            )
        ),
    )

    return LaunchDescription([
        joystick_launch,
        qualisys_launch,
    ])
