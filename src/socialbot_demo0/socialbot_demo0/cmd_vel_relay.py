from __future__ import annotations

import time
from math import isfinite
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def clamp(value: float, limit: float) -> float:
    """Clamp a finite value symmetrically; invalid values become zero."""
    if not isfinite(value):
        return 0.0
    if limit <= 0.0:
        return 0.0
    return max(-limit, min(limit, value))


class CmdVelRelay(Node):
    """Continuously relay safe velocity commands to the existing OmniController.

    The existing OmniController uses a DDS deadline on its Twist subscription and
    can become internally inactive after repeated missed deadlines. For this
    reason this node publishes at a fixed rate even when the latest input has
    expired; an expired input is converted to an explicit zero command.
    """

    def __init__(self) -> None:
        super().__init__("demo0_cmd_vel_relay")

        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/omni_controller/twist_cmd")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("input_timeout", 0.50)
        self.declare_parameter("max_linear_x", 0.30)
        self.declare_parameter("max_linear_y", 0.0)
        self.declare_parameter("max_angular_z", 0.70)

        self._publish_rate = float(self.get_parameter("publish_rate").value)
        self._input_timeout = float(self.get_parameter("input_timeout").value)
        self._max_x = float(self.get_parameter("max_linear_x").value)
        self._max_y = float(self.get_parameter("max_linear_y").value)
        self._max_z = float(self.get_parameter("max_angular_z").value)

        if self._publish_rate <= 0.0:
            raise ValueError("publish_rate must be greater than zero")
        if self._input_timeout <= 0.0:
            raise ValueError("input_timeout must be greater than zero")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        input_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            # OmniController requests a DDS deadline. Publishers without an
            # offered deadline are incompatible even when reliability matches.
            deadline=Duration(seconds=1.0 / self._publish_rate),
        )
        self._publisher = self.create_publisher(Twist, output_topic, output_qos)
        self._subscription = self.create_subscription(
            Twist, input_topic, self._on_cmd_vel, input_qos
        )

        self._latest_command = Twist()
        self._last_input_monotonic: Optional[float] = None
        self._input_was_stale = True

        # Use a steady clock so the motor-command keepalive remains deterministic
        # even when Gazebo simulation time is temporarily paused.
        steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(
            1.0 / self._publish_rate,
            self._publish_command,
            clock=steady_clock,
        )

        self.get_logger().info(
            f"Relaying {input_topic} -> {output_topic} at {self._publish_rate:.1f} Hz; "
            f"input timeout={self._input_timeout:.2f} s; limits "
            f"x={self._max_x:.2f}, y={self._max_y:.2f}, yaw={self._max_z:.2f}"
        )

    def _on_cmd_vel(self, message: Twist) -> None:
        output = Twist()
        output.linear.x = clamp(message.linear.x, self._max_x)
        output.linear.y = clamp(message.linear.y, self._max_y)
        output.linear.z = 0.0
        output.angular.x = 0.0
        output.angular.y = 0.0
        output.angular.z = clamp(message.angular.z, self._max_z)

        self._latest_command = output
        self._last_input_monotonic = time.monotonic()

        if self._input_was_stale:
            self.get_logger().info("Velocity input received; forwarding commands")
            self._input_was_stale = False

    def _publish_command(self) -> None:
        now = time.monotonic()
        input_is_fresh = (
            self._last_input_monotonic is not None
            and now - self._last_input_monotonic <= self._input_timeout
        )

        if input_is_fresh:
            self._publisher.publish(self._latest_command)
            return

        self._publisher.publish(Twist())
        if not self._input_was_stale:
            self.get_logger().warn(
                "Velocity input timed out; continuously publishing a zero command"
            )
            self._input_was_stale = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Launch may already have invalidated the ROS context on SIGINT.
        if rclpy.ok():
            node._publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
