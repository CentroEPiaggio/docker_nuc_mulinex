"""Joystick for Omnirace: emergency stop / activate, and optionally drive the robot.

    ros2 launch omnirace_bringup joystick_estop.launch.py                 # A = stop, Y = activate
    ros2 launch omnirace_bringup joystick_estop.launch.py drive:=true     # also drive
    ros2 launch omnirace_bringup joystick_estop.launch.py estop_button:=1 # another stop button

drive:=false (default): no twist is published, so it never competes with the controller under
test (Simulink node, demo_run_node, ...).
drive:=true: left stick = translation (max_v), triggers = rotate left / right, speed
proportional to the press (up to max_w),
published on /omni_controller/twist_cmd. Do not run it together with another node publishing
twist_cmd. Indices: `ros2 topic echo /joy --field buttons` (or --field axes).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        ('estop_button', '0', int, 'Button index for the emergency stop (0 = A on the 8BitDo)'),
        ('activate_button', '4', int, 'Button index to activate the controller (4 = Y; -1: off)'),
        ('drive', 'false', bool, 'Also drive the robot: left stick + rotation buttons'),
        ('max_v', '0.3', float, 'drive:=true, full-stick linear speed [m/s]'),
        ('max_w', '0.6', float, 'drive:=true, rotation speed [rad/s]'),
        ('rot_left_axis', '4', int, 'drive:=true, left trigger axis: rotate counter-clockwise'),
        ('rot_right_axis', '5', int, 'drive:=true, right trigger axis: rotate clockwise'),
        ('rot_left_button', '8', int, 'drive:=true, used if rot_left_axis is -1 (on/off)'),
        ('rot_right_button', '9', int, 'drive:=true, used if rot_right_axis is -1 (on/off)'),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=desc) for n, d, _, desc in args]
        + [
            Node(package='joy', executable='joy_node', name='joy_node'),
            Node(
                package='omnirace_bringup',
                executable='joy_estop_node',
                name='joy_estop',
                parameters=[{n: ParameterValue(LaunchConfiguration(n), value_type=t)
                             for n, _, t, _ in args}],
                output='screen',
            ),
        ]
    )
