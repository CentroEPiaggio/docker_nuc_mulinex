import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('omni_mulinex_joystick'),
                'launch',
                'joystick.launch.py',
            )
        )
    )

    ik_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ik_controller'),
                'launch',
                'ik_controller.launch.py',
            )
        )
    )

    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2',
            'bag',
            'record',
            '/omni_controller/joints_state',
            '/omni_controller/legs_cmd',
            '/omni_controller/twist_cmd',
            '/ik_controller/base_pose',
            '/joy',
        ],
        output='screen',
    )

    return LaunchDescription(
        [
            joystick_launch,
            ik_controller_launch,
            rosbag_record,
        ]
    )
