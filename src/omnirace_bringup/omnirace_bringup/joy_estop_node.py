#!/usr/bin/env python3
"""Joystick for Omnirace: emergency stop / activate buttons, optional driving.

Parameters (indices as in `ros2 topic echo /joy --field buttons` / `--field axes`):
    estop_button     default 0 (A on the 8BitDo) -> /omni_controller/emergency_srv (controller INACTIVE, zero commands)
    activate_button  default 4 (Y on the 8BitDo) -> /omni_controller/activate_srv; -1 disables it
    drive            default False: no twist published (another node drives the robot)
    max_v, max_w     drive=True: linear speed at full stick [m/s], rotation speed [rad/s]
    axis_x, axis_y   drive=True: left stick axes (up = +x forward, left = +y)
    rot_left_axis    drive=True: left trigger axis, rotate counter-clockwise (default 4)
    rot_right_axis   drive=True: right trigger axis, rotate clockwise (default 5)
                     rotation speed is proportional to how far the trigger is pressed
    trigger_rest, trigger_pressed   axis value with the trigger released / fully pressed
                     (default 1.0 / -1.0, as joy_node reports them)
    rot_left_button, rot_right_button   used instead of the axes when the axis is -1
                     (on/off rotation at max_w; defaults 8 / 9)

Buttons react to the press (0 -> 1), so holding one sends a single request.
With drive=True the twist goes to /omni_controller/twist_cmd at 20 Hz with the QoS the
controller expects (best effort, deadline 0.105 s); sticks at rest give zero.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool

NAMES = {0: 'A', 1: 'B', 3: 'X', 4: 'Y'}  # 8BitDo Ultimate 2 as seen by joy_node


class JoyEstop(Node):
    def __init__(self):
        super().__init__('joy_estop')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.estop_btn = p('estop_button', 0)
        self.activate_btn = p('activate_button', 4)
        self.drive = p('drive', False)
        self.max_v = p('max_v', 0.3)
        self.max_w = p('max_w', 0.6)
        self.axis_x = p('axis_x', 1)
        self.axis_y = p('axis_y', 0)
        self.rot_left = p('rot_left_button', 8)
        self.rot_right = p('rot_right_button', 9)
        self.rot_left_axis = p('rot_left_axis', 4)
        self.rot_right_axis = p('rot_right_axis', 5)
        self.trig_rest = p('trigger_rest', 1.0)
        self.trig_pressed = p('trigger_pressed', -1.0)
        # joy_node reports a trigger as 0.0 until it is first moved: ignore it until then
        self.trig_seen = set()
        self.deadzone = p('deadzone', 0.1)

        self.estop_cli = self.create_client(SetBool, '/omni_controller/emergency_srv')
        self.activate_cli = self.create_client(SetBool, '/omni_controller/activate_srv')
        self.prev = []
        self.cmd = Twist()
        self.create_subscription(Joy, 'joy', self.on_joy, 10)
        if self.drive:
            qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             deadline=Duration(seconds=0.105))
            self.pub = self.create_publisher(Twist, '/omni_controller/twist_cmd', qos)
            self.create_timer(0.05, lambda: self.pub.publish(self.cmd))

        green, reset = '\033[32m', '\033[0m'
        msg = (f'emergency stop = button {self.estop_btn} ({NAMES.get(self.estop_btn, "?")}), '
               f'activate = button {self.activate_btn} ({NAMES.get(self.activate_btn, "?")})')
        if self.drive:
            msg += (f' | DRIVE: left stick = translation (max {self.max_v} m/s), '
                    f'triggers (axes {self.rot_left_axis}/{self.rot_right_axis}) = rotate left/right, '
                    f'up to {self.max_w} rad/s')
        self.get_logger().info(green + msg + reset)

    def pressed(self, buttons, i):
        return 0 <= i < len(buttons) and buttons[i] and not (i < len(self.prev) and self.prev[i])

    def call(self, cli, what):
        if not cli.service_is_ready():
            self.get_logger().error(f'{what}: service not available')
            return
        cli.call_async(SetBool.Request(data=True)).add_done_callback(
            lambda f: self.get_logger().info(f'{what}: {f.result().message}'))

    def axis(self, axes, i):
        v = axes[i] if 0 <= i < len(axes) else 0.0
        return 0.0 if abs(v) < self.deadzone else max(-1.0, min(1.0, v))

    def held(self, buttons, i):
        return 0 <= i < len(buttons) and bool(buttons[i])

    def trigger(self, axes, i):
        """0 (released) .. 1 (fully pressed)."""
        if not 0 <= i < len(axes):
            return 0.0
        v = axes[i]
        if v != 0.0:
            self.trig_seen.add(i)
        if i not in self.trig_seen:
            return 0.0
        amount = (v - self.trig_rest) / (self.trig_pressed - self.trig_rest)
        amount = max(0.0, min(1.0, amount))
        return 0.0 if amount < self.deadzone else amount

    def rotation(self, msg, axis, button):
        return self.trigger(msg.axes, axis) if axis >= 0 else float(self.held(msg.buttons, button))

    def on_joy(self, msg):
        if self.pressed(msg.buttons, self.estop_btn):
            self.get_logger().warn('EMERGENCY STOP')
            self.call(self.estop_cli, 'emergency')
        if self.pressed(msg.buttons, self.activate_btn):
            self.call(self.activate_cli, 'activate')
        if self.drive:
            cmd = Twist()
            cmd.linear.x = self.max_v * self.axis(msg.axes, self.axis_x)
            cmd.linear.y = self.max_v * self.axis(msg.axes, self.axis_y)
            cmd.angular.z = self.max_w * (self.rotation(msg, self.rot_left_axis, self.rot_left)
                                          - self.rotation(msg, self.rot_right_axis, self.rot_right))
            self.cmd = cmd
        self.prev = list(msg.buttons)


def main():
    rclpy.init()
    rclpy.spin(JoyEstop())


if __name__ == '__main__':
    main()
