"""Run a student Simulink controller in Gazebo and record a rosbag, all in one go.

    ros2 launch omnirace_bringup omnirace_record.launch.py model:=Omnirace_Demo

Simulink names the package after the model in lowercase and the executable after the
model, so model:=Omnirace_Demo runs `ros2 run omnirace_demo Omnirace_Demo`.

Each step starts when the previous condition holds, not after a fixed delay:
  Gazebo + bag recorder (sim time, writes from the first /clock)
  -> /omni_controller/activate_srv exists  -> start the student node
  -> /omni_controller/twist_cmd has a publisher -> call activate_srv
Ctrl+C stops everything and closes the bag (record:=false: no bag). The bag goes to
~/mulinex_ws/bags/<model>_<date> unless bag:=<path> is given.

The package is built by Simulink in /home/mulinex/student_ws, readable only by user
mulinex, so the node is started with sudo -u mulinex (passwordless in the container).
"""
import os
import time

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

TOPICS = ['/tf', '/tf_static', '/omni_controller/twist_cmd', '/clock']
WAIT_TIMEOUT = 180  # s; Gazebo can take long to start


def _wait_until(name, condition):
    """Process that exits 0 as soon as the shell condition holds, 124 on timeout."""
    return ExecuteProcess(
        name=name, output='screen',
        cmd=['timeout', str(WAIT_TIMEOUT), 'bash', '-c',
             f'until {condition}; do sleep 1; done'])


def _then(waiter, actions, what):
    """Run actions when waiter exits 0, otherwise log why nothing started."""
    def on_exit(event, _context):
        if event.returncode == 0:
            return actions
        return [LogInfo(msg=f'[omnirace_record] gave up waiting for {what} '
                            f'after {WAIT_TIMEOUT} s: nothing more will start')]
    return RegisterEventHandler(OnProcessExit(target_action=waiter, on_exit=on_exit))


def _record_and_run(context):
    model = LaunchConfiguration('model').perform(context)
    pkg = model.lower()
    record = LaunchConfiguration('record').perform(context).lower() in ('true', '1', 'yes')

    env = ' '.join(f'{k}={os.environ[k]}' for k in ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION')
                   if k in os.environ)
    student = ExecuteProcess(
        name=model, output='screen', sigterm_timeout='5',
        cmd=['bash', '-c',
             f'sudo -u mulinex -H env {env} bash -c "'
             'source /opt/ros/humble/setup.bash && '
             'source /home/mulinex/student_ws/install/setup.bash && '
             f'exec ros2 run {pkg} {model}"'])
    activate = ExecuteProcess(
        name='activate', output='screen',
        cmd=['ros2', 'service', 'call', '/omni_controller/activate_srv',
             'std_srvs/srv/SetBool', '{data: true}'])

    controller_ready = _wait_until(
        'wait_omni_controller',
        'ros2 service list | grep -x /omni_controller/activate_srv > /dev/null')
    # activate only once the node publishes, or the deadline check deactivates it again
    node_publishing = _wait_until(
        'wait_twist_publisher',
        "ros2 topic info /omni_controller/twist_cmd | grep 'Publisher count: [1-9]' > /dev/null")

    actions = [LogInfo(msg='[omnirace_record] record:=false, no bag')]
    if record:
        bag = LaunchConfiguration('bag').perform(context) or os.path.expanduser(
            f'~/mulinex_ws/bags/{pkg}_{time.strftime("%Y%m%d_%H%M%S")}')
        os.makedirs(os.path.dirname(bag), exist_ok=True)
        actions = [
            LogInfo(msg=f'[omnirace_record] bag: {bag}'),
            # --use-sim-time: stamps on the Gazebo clock, like the TF poses (needed by score.m)
            ExecuteProcess(name='bag', output='screen',
                           cmd=['ros2', 'bag', 'record', '--use-sim-time', '-o', bag] + TOPICS),
        ]

    return actions + [
        controller_ready,
        _then(controller_ready, [student, node_publishing], 'omni_controller'),
        _then(node_publishing, [activate], f'{model} to publish twist_cmd'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', description='Simulink model name, e.g. Omnirace_Demo'),
        DeclareLaunchArgument('record', default_value='true', choices=['true', 'false'],
                              description='Record a rosbag'),
        DeclareLaunchArgument('bag', default_value='', description='Bag output folder'),
        DeclareLaunchArgument('waypoints_file', default_value=PathJoinSubstitution(
            [FindPackageShare('omnirace_bringup'), 'worlds', 'waypoints.sdf'])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('omnirace_bringup'), 'launch', 'omnirace_sim.launch.py'])),
            launch_arguments={'waypoints_file': LaunchConfiguration('waypoints_file')}.items()),
        OpaqueFunction(function=_record_and_run),
    ])
