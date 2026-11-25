#!/usr/bin/env python3
"""Constant twist for a few seconds, then zero. For wheel-direction tests with the robot lifted.

    python3 twist_test.py                 # vx = +0.1 m/s for 5 s
    python3 twist_test.py 0 0.1 0         # vy = +0.1 m/s (left)
    python3 twist_test.py 0 0 0.3 5       # wz = +0.3 rad/s (counter-clockwise) for 5 s
    python3 twist_test.py 0.1 0 0 3 nostop  # exit without sending zeros (simulates a crashed
                                            # node): the wheels must stop by the controller timeout

Publishes with the QoS omni_controller expects (best effort, deadline 0.105 s): plain
`ros2 topic pub` on Humble cannot set the deadline, so its messages are rejected.
It starts publishing at once and never calls the controller services:
the robot moves only while the controller is active (Y on the joystick). It runs for the
given time and sends zeros for 1 s (unless nostop).
"""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy




def main():
    nostop = 'nostop' in sys.argv[1:]
    nums = [a for a in sys.argv[1:] if a != 'nostop'][:4]
    vx, vy, wz, secs = [float(a) for a in nums] + [0.1, 0.0, 0.0, 5.0][len(nums):]
    assert abs(vx) <= 0.5 and abs(vy) <= 0.5 and abs(wz) <= 1.0, 'stay within 0.5 m/s, 1 rad/s'

    rclpy.init()
    node = rclpy.create_node('twist_test')
    qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.BEST_EFFORT, deadline=Duration(seconds=0.105))
    pub = node.create_publisher(Twist, '/omni_controller/twist_cmd', qos)

    def send(x, y, z, duration):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = x, y, z
        end = time.monotonic() + duration
        while time.monotonic() < end:
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.05)                       # 20 Hz

    node.get_logger().warn('Publishing now: the robot moves only if the controller is active (Y = activate, A = stop)')

    node.get_logger().info(f'cmd vx={vx} vy={vy} wz={wz} for {secs} s')
    send(vx, vy, wz, secs)
    if nostop:
        node.get_logger().warn('exit WITHOUT zero command: wheels must stop within 0.5 s')
    else:
        node.get_logger().info('stop')
        send(0.0, 0.0, 0.0, 1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
