from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    demo_share = get_package_share_directory("socialbot_demo0")
    navigation_launch = os.path.join(demo_share, "launch", "navigation.launch.py")
    default_script = os.path.join(demo_share, "config", "interview_script.yaml")

    script_file = LaunchConfiguration("script_file")
    audio_enabled = LaunchConfiguration("audio_enabled")
    mission_delay = LaunchConfiguration("mission_delay")

    mission = Node(
        package="socialbot_demo0",
        executable="interview_mission",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "script_file": script_file,
                "audio_enabled": ParameterValue(audio_enabled, value_type=bool),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("script_file", default_value=default_script),
            DeclareLaunchArgument("audio_enabled", default_value="true"),
            DeclareLaunchArgument("mission_delay", default_value="5.0"),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(navigation_launch)),
            TimerAction(period=mission_delay, actions=[mission]),
        ]
    )
