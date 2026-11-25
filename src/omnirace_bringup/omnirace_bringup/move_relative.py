#!/usr/bin/env python3
"""Closed-loop small move with the Qualisys pose: go to (current pose + dx, dy) in the world
frame, keeping the current heading. For the first real-robot tests.

    python3 move_relative.py            # +0.10 m along world x
    python3 move_relative.py 0 0.10     # +0.10 m along world y

Slow on purpose: 0.025..0.05 m/s and max 0.2 rad/s, stops within 1 cm, gives up after 15 s or if
the pose is lost/NaN for 0.5 s or jumps by more than 0.3 m. Publishes with the controller
QoS (best effort, deadline 0.105 s), sends zero for 1 s at the end. It does NOT activate the
controller: it starts publishing at once; the robot moves while the controller is active (Y).
Keep joystick_estop running (A = emergency stop).
"""
import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_msgs.msg import TFMessage


KP, KTH, VMAX, WMAX, TOL, TIMEOUT = 1.0, 1.0, 0.05, 0.2, 0.01, 15.0
VMIN = 0.025  # m/s: below this the mecanum wheels stall on the floor (static friction)


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def body_cmd(ex, ey, eth, th):
    """World-frame error -> saturated body-frame twist."""
    c, s = math.cos(th), math.sin(th)
    vx, vy = KP * (c * ex + s * ey), KP * (-s * ex + c * ey)
    n = math.hypot(vx, vy)
    if n > VMAX:
        vx, vy = vx * VMAX / n, vy * VMAX / n
    elif 0.0 < n < VMIN:
        vx, vy = vx * VMIN / n, vy * VMIN / n
    return vx, vy, max(-WMAX, min(WMAX, KTH * eth))



def main():
    _self_check()  # also when started as a ros2 entry point
    dx, dy = ([float(a) for a in sys.argv[1:3]] + [0.10, 0.0][len(sys.argv[1:3]):])
    assert math.hypot(dx, dy) <= 0.5, 'max 0.5 m'
    rclpy.init()
    node = rclpy.create_node('move_relative')
    log = node.get_logger()
    qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.BEST_EFFORT, deadline=Duration(seconds=0.105))
    pub = node.create_publisher(Twist, '/omni_controller/twist_cmd', qos)
    # pose (and wheels) received by a second node spun in its own thread: the test loop only
    # reads the newest value, so the pose stays fresh whatever the loop does (Qualisys at 100 Hz)
    listen = rclpy.create_node(node.get_name() + '_listen')
    listen_exec = SingleThreadedExecutor()
    listen_exec.add_node(listen)
    threading.Thread(target=listen_exec.spin, daemon=True).start()
    latest = {}

    def on_tf(m):
        # direct /tf subscription (the Python tf2 buffer fell ~1 s behind at 100 Hz)
        for t in m.transforms:
            if t.header.frame_id == 'world' and t.child_frame_id == 'base_link' and not any(
                    math.isnan(v) for v in (t.transform.translation.x, t.transform.translation.y)):
                latest['tf'] = t
    listen.create_subscription(TFMessage, '/tf', on_tf, QoSProfile(
        depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT))

    def pose():
        t = latest.get('tf')
        if t is None:
            return None
        age = (node.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
        p = (t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation))
        return None if age > 0.5 or any(math.isnan(v) for v in p) else p

    def publish(vx=0.0, vy=0.0, wz=0.0):
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = vx, vy, wz
        pub.publish(m)

    # wait for a fresh pose, publishing zero so the subscription matches
    t0 = time.monotonic()
    p0 = None
    while p0 is None and time.monotonic() - t0 < 5.0:
        publish()
        rclpy.spin_once(node, timeout_sec=0.05)
        p0 = pose()
    if p0 is None:
        log.error('no fresh world -> base_link pose: is the Qualisys driver streaming?')
        return
    goal = (p0[0] + dx, p0[1] + dy, p0[2])
    log.info(f'start ({p0[0]:+.3f}, {p0[1]:+.3f}, {math.degrees(p0[2]):+.1f} deg) '
             f'-> goal ({goal[0]:+.3f}, {goal[1]:+.3f})')

    node.get_logger().warn('Publishing now: the robot moves only if the controller is active (Y = activate, A = stop)')

    t0, last, prev, reason = time.monotonic(), time.monotonic(), p0, 'timeout'
    while time.monotonic() - t0 < TIMEOUT:
        rclpy.spin_once(node, timeout_sec=0.0)
        p = pose()
        if p is None:
            publish()
            if time.monotonic() - last > 0.5:
                reason = 'pose lost'
                break
        else:
            if math.hypot(p[0] - prev[0], p[1] - prev[1]) > 0.3:
                reason = 'pose jump'
                break
            last, prev = time.monotonic(), p
            ex, ey = goal[0] - p[0], goal[1] - p[1]
            eth = math.atan2(math.sin(goal[2] - p[2]), math.cos(goal[2] - p[2]))
            if math.hypot(ex, ey) < TOL and abs(eth) < 0.05:
                reason = 'reached'
                break
            publish(*body_cmd(ex, ey, eth, p[2]))
        time.sleep(0.05)

    for _ in range(20):
        publish()
        rclpy.spin_once(node, timeout_sec=0.05)  # keep reading the pose while stopping
    p = pose()
    if p:
        log.info(f'{reason}: end ({p[0]:+.3f}, {p[1]:+.3f}, {math.degrees(p[2]):+.1f} deg), '
                 f'moved dx={p[0] - p0[0]:+.3f} dy={p[1] - p0[1]:+.3f}')
    else:
        log.warn(f'{reason}: no pose at the end')
    listen_exec.shutdown()
    listen.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


def _self_check():
    # error straight ahead with heading 0 -> pure +vx, saturated
    vx, vy, wz = body_cmd(0.10, 0.0, 0.0, 0.0)
    assert abs(vx - 0.05) < 1e-9 and abs(vy) < 1e-9 and wz == 0.0
    # same world error with the robot facing +y (90 deg) -> it must move to its right (-vy)
    vx, vy, _ = body_cmd(0.10, 0.0, 0.0, math.pi / 2)
    assert abs(vx) < 1e-9 and abs(vy + 0.05) < 1e-9
    # small error: speed raised to VMIN so the robot does not stall
    vx, vy, _ = body_cmd(0.02, 0.0, 0.0, 0.0)
    assert abs(vx - VMIN) < 1e-9


if __name__ == '__main__':
    main()
