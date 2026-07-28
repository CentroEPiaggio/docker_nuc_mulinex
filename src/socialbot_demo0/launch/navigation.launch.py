from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    demo_share = get_package_share_directory("socialbot_demo0")
    nav2_share = get_package_share_directory("nav2_bringup")

    simulation_launch = os.path.join(demo_share, "launch", "simulation.launch.py")
    nav2_launch = os.path.join(nav2_share, "launch", "bringup_launch.py")
    params_file = os.path.join(demo_share, "config", "nav2_params.yaml")
    map_file = os.path.join(demo_share, "maps", "demo0_map.yaml")
    rviz_config = os.path.join(nav2_share, "rviz", "nav2_default_view.rviz")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulation_launch),
                launch_arguments={"use_rviz": "false"}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    "use_sim_time": "true",
                    "autostart": "true",
                    # Nav2 Humble interpolates this value into a PythonExpression
                    # (`not <value>`), so it must be a Python boolean literal.
                    "use_composition": "False",
                    "map": map_file,
                    "params_file": params_file,
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
