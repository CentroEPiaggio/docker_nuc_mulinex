#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class OmniGoToStart(Node):
    def __init__(self):
        super().__init__('omni_go_to_start')

        # Target pose in world frame.
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = 0.0

        # Motion constraints and tolerances.
        self.max_v_xy = 0.4
        self.max_v_theta = 0.8
        self.xy_tolerance = 0.05
        self.theta_tolerance = 0.10

        # Proportional gains.
        self.kp_xy = 1.8
        self.kp_theta = 2.0

        # TF listener for world -> base_link.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Twist publisher.
        cmd_vel_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        cmd_vel_qos.deadline = Duration(seconds=0, nanoseconds=105000000)
        self.cmd_vel_pub = self.create_publisher(Twist, '/omni_controller/twist_cmd', cmd_vel_qos)

        # Activation client: /omni_controller/activate_srv SetBool{data: true}
        self.activate_client = self.create_client(SetBool, '/omni_controller/activate_srv')
        self.activate_future = None
        self.activation_ok = False
        self.activation_failed = False

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Go-to-start node initialized; waiting to activate controller.')
        self.request_activation()

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def get_yaw_from_quaternion(q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def request_activation(self):
        if not self.activate_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn(
                'Waiting for /omni_controller/activate_srv...',
                throttle_duration_sec=2.0,
            )
            return

        if self.activate_future is None:
            req = SetBool.Request()
            req.data = True
            self.activate_future = self.activate_client.call_async(req)
            self.get_logger().info('Sent activation request to /omni_controller/activate_srv.')
            return

        if not self.activate_future.done():
            return

        try:
            result = self.activate_future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Activation service call failed: {exc}')
            self.activation_failed = True
            return

        if result.success:
            self.activation_ok = True
            self.get_logger().info(f'Controller activated: {result.message}')
        else:
            self.get_logger().error(f'Controller activation rejected: {result.message}')
            self.activation_failed = True

    def control_loop(self):
        # if self.activation_failed:
        #     self.stop_and_exit('Exiting due to controller activation failure.')
        #     return

        # if not self.activation_ok:
        #     self.request_activation()
        #     return

        try:
            t = self.tf_buffer.lookup_transform('map', 'Omnicar', rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(f'Waiting for TF world->base_link: {ex}', throttle_duration_sec=2.0)
            return

        robot_x = t.transform.translation.x
        robot_y = t.transform.translation.y
        robot_theta = self.get_yaw_from_quaternion(t.transform.rotation)

        error_x_global = self.target_x - robot_x
        error_y_global = self.target_y - robot_y
        error_theta = self.normalize_angle(self.target_theta - robot_theta)

        distance = math.hypot(error_x_global, error_y_global)
        if distance <= self.xy_tolerance and abs(error_theta) <= self.theta_tolerance:
            self.stop_and_exit('Reached (0, 0, 0), sending zero twist and exiting.')
            return

        # Transform XY error into base_link frame before applying velocity control.
        error_x_local = error_x_global * math.cos(robot_theta) + error_y_global * math.sin(robot_theta)
        error_y_local = -error_x_global * math.sin(robot_theta) + error_y_global * math.cos(robot_theta)

        cmd_vx = max(-self.max_v_xy, min(self.max_v_xy, self.kp_xy * error_x_local))
        cmd_vy = max(-self.max_v_xy, min(self.max_v_xy, self.kp_xy * error_y_local))
        cmd_vtheta = max(-self.max_v_theta, min(self.max_v_theta, self.kp_theta * error_theta))

        msg = Twist()
        msg.linear.x = float(cmd_vx)
        msg.linear.y = float(cmd_vy)
        msg.angular.z = float(cmd_vtheta)
        self.cmd_vel_pub.publish(msg)

    def stop_robot(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def stop_and_exit(self, reason):
        # Publish zero twist twice to reduce chance of dropping the final stop command.
        self.stop_robot()
        self.stop_robot()
        self.get_logger().info(reason)
        self.timer.cancel()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = OmniGoToStart()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_robot()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
