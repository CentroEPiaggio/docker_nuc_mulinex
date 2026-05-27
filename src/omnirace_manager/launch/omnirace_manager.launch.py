import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('omnirace_manager')
    dashboard_dir = os.path.join(package_share, 'dashboard')

    telemetry_node = Node(
        package='omnirace_manager',
        executable='omnirace_telemetry',
        name='omnirace_telemetry',
        output='screen',
    )

    rosbridge_server = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'rosbridge_server', 'rosbridge_websocket',
            '--ros-args', '-p', 'port:=9091',
        ],
        output='screen',
    )

    dashboard_server = ExecuteProcess(
        cmd=[
            'python3', '-m', 'http.server', '8000',
            '--bind', '0.0.0.0',
            '--directory', dashboard_dir,
        ],
        output='screen',
    )

    return LaunchDescription([
        telemetry_node,
        rosbridge_server,
        dashboard_server,
    ])