import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='false',
    )

    nuc_heartbeat = Node(
        package='omni_bringup',
        executable='nuc_heartbeat_node',
        name='nuc_heartbeat',
        parameters=[{'rate': 10.0}],
        output='screen',
    )

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
        condition=IfCondition(LaunchConfiguration('record_bag')),
        cmd=[
            'ros2',
            'bag',
            'record', 
            '/distributor_state_broadcaster/distributors_state',
            '/distributor_state_broadcaster/transition_event',
            '/events/write_split',
            '/ik_controller/base_pose',
            '/joy',
            '/joy/set_feedback',
            '/nuc_heartbeat',
            '/omni_controller/direct_wheels_cmd',
            '/omni_controller/joints_command',
            '/omni_controller/joints_state',
            '/omni_controller/legs_cmd',
            '/omni_controller/performance',
            '/omni_controller/safety_state',
            '/omni_controller/transition_event',
            '/omni_controller/twist_cmd',
            '/state_broadcaster/joints_state',
            '/state_broadcaster/performance_indexes',
            '/state_broadcaster/transition_event',
        ],
        output='screen',
    )

    return LaunchDescription([
        record_bag_arg,
        nuc_heartbeat,
        joystick_launch,
        ik_controller_launch,
        rosbag_record,
    ])
