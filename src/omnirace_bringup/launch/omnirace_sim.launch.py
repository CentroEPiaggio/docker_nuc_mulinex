import os

import launch
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _spawn_waypoints_and_activate(context, *args, **kwargs):
    """
    Callback to spawn waypoints and activate the controller.
    Runs after the simulator is ready.
    """
    actions = []

    # Spawn waypoints entity
    actions.append(
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-file', LaunchConfiguration('waypoints_file').perform(context),
                '-name', 'waypoints',
                '-x', '0',
                '-y', '0',
                '-z', '0',
            ],
            output='screen',
        )
    )


    return actions


def generate_launch_description():
    ld = launch.LaunchDescription()

    # ── Launch arguments ─────────────────────────────────────────────────────
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='omnicar',
        choices=['omnicar', 'omniquad12'],
        description='Robot to simulate: omnicar or omniquad12',
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='World SDF file name (looked up in omni_gazebo/worlds/)',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2',
    )

    # ── Resolve paths ────────────────────────────────────────────────────────
    omnirace_pkg_share = FindPackageShare('omnirace_bringup')
    gazebo_pkg_share = FindPackageShare('omni_gazebo')

    waypoints_file = PathJoinSubstitution([
        omnirace_pkg_share, 'worlds', 'waypoints.sdf',
    ])

    ld.add_action(robot_arg)
    ld.add_action(world_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(DeclareLaunchArgument(
        'waypoints_file',
        default_value=waypoints_file,
        description='Path to waypoints SDF file',
    ))

    # ── Include gz_sim.launch.py from omni_gazebo ────────────────────────────
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                gazebo_pkg_share, 'launch', 'gz_sim.launch.py',
            ])
        ),
        launch_arguments={
            'robot': LaunchConfiguration('robot'),
            'world': LaunchConfiguration('world'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )

    ld.add_action(gz_sim_launch)

    # ── Spawn waypoints and activate controller ───────────────────────────────
    # Use OpaqueFunction to spawn waypoints after a delay (to let gazebo start)
    ld.add_action(OpaqueFunction(
        function=_spawn_waypoints_and_activate,
    ))

    return ld
