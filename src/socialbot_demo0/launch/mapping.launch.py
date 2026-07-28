from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    demo_share = get_package_share_directory("socialbot_demo0")
    slam_share = get_package_share_directory("slam_toolbox")
    nav2_share = get_package_share_directory("nav2_bringup")

    simulation_launch = os.path.join(demo_share, "launch", "simulation.launch.py")
    slam_launch = os.path.join(slam_share, "launch", "online_async_launch.py")
    slam_params = os.path.join(demo_share, "config", "slam_toolbox.yaml")
    rviz_config = os.path.join(nav2_share, "rviz", "nav2_default_view.rviz")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulation_launch),
                launch_arguments={"use_rviz": "false"}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                launch_arguments={
                    "use_sim_time": "true",
                    "slam_params_file": slam_params,
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
