from __future__ import annotations

from copy import deepcopy

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class GroundTruthOdomAdapter(Node):
    """Reframe Gazebo ground-truth odometry as odom -> base_link for Nav2."""

    def __init__(self) -> None:
        super().__init__("demo0_ground_truth_odom")
        self.declare_parameter("input_topic", "/gz/odom_raw")
        self.declare_parameter("output_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        # Gazebo's input is sensor-data QoS, but Nav2's odometry consumers
        # request reliable delivery.
        output_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(Odometry, output_topic, output_qos)
        self._subscription = self.create_subscription(
            Odometry, input_topic, self._on_odometry, qos_profile_sensor_data
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info(
            f"Reframing {input_topic} -> {output_topic} as "
            f"{self._odom_frame} -> {self._base_frame}"
        )

    def _on_odometry(self, source: Odometry) -> None:
        message = deepcopy(source)
        message.header.frame_id = self._odom_frame
        message.child_frame_id = self._base_frame

        if not any(message.pose.covariance):
            message.pose.covariance[0] = 0.0025
            message.pose.covariance[7] = 0.0025
            message.pose.covariance[35] = 0.005
        if not any(message.twist.covariance):
            message.twist.covariance[0] = 0.01
            message.twist.covariance[7] = 0.01
            message.twist.covariance[35] = 0.02

        self._publisher.publish(message)

        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = self._odom_frame
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundTruthOdomAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
