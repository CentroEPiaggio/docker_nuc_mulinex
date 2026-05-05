import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        remappings=[('/cmd_vel_out', '/omni_controller/twist_cmd')],
        parameters=[os.path.join(
            get_package_share_directory('omnirace_bringup'),
            'config',
            'twist_mux.yaml',
        )],
        output='screen',
    )

    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('omni_mulinex_joystick'),
                'launch',
                'joystick.launch.py',
            )
        ),
        launch_arguments={
            'twist_cmd_topic': '/omni_controller/joystick_cmd_twist',
        }.items(),
    )

    qualisys_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mocap4ros2_qualisys'),
                'launch',
                'qualisys.launch.py',
            )
        ),
    )

    return LaunchDescription([
        twist_mux,
        joystick_launch,
        qualisys_launch,
    ])
