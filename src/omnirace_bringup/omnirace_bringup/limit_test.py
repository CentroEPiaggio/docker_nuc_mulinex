#!/usr/bin/env python3
"""Real-robot test of the omni_controller twist limits, measured two independent ways.

From the current pose (x0, y0, th0) it drives a forward/back trajectory along the world x axis
with a different heading at every waypoint:

    0. x0,     th0              the pose where the robot is now (reached at once)
    1. x0 + L, th0              forward
    2. x0,     th0              back
    3. x0 + L, th0 + 90 deg     forward, turning
    4. x0,     th0 - 90 deg     back, turning the other way
    5. x0 + L, th0              forward, back to the start heading

On purpose it ASKS MORE than the limits (CMD_V = 1.0 m/s, CMD_W = 2.0 rad/s): omni_controller must
clamp each axis of the body twist to twist_max (vx, vy <= 0.5 m/s, wz <= 1.0 rad/s).
The body velocity actually reached is measured with:
  - the Qualisys: pose difference over 0.3 s using the driver stamps, rotated to the body frame;
  - the motor drivers: wheel speeds from /state_broadcaster/joints_state through the mecanum
    forward kinematics of omni_controller (wheel odometry).
Each axis is compared with its limit (PASS if <= MARGIN x limit), for both sources.
A rosbag of /tf, /tf_static, twist_cmd and joint states is recorded in ~/mulinex_ws/bags.

    python3 limit_test.py            # L = 0.5 m (needs ~0.8 m free in +x, ~0.4 m around)
    python3 limit_test.py 0.3        # shorter segments
    python3 limit_test.py 0.5 nobag  # no rosbag

Safety: stops and exits if the pose is lost/NaN for 0.3 s or jumps by > 0.3 m; no area limit
(the operator stops the robot with A). Keep the joystick
e-stop (A) at hand. Publishes with the controller QoS, sends zero for 1 s at the end.
It starts publishing at once and never calls the controller services:
the robot moves only while the controller is active (Y on the joystick).
"""
import math
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from pi3hat_moteus_int_msgs.msg import JointsStates
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_msgs.msg import TFMessage


CMD_V, CMD_W = 1.0, 2.0          # requested above the limits on purpose
LIM = (0.5, 0.5, 1.0)            # omni_controller twist_max_x, twist_max_y, twist_max_z
MARGIN = 1.10                    # pass if measured <= 110 % of the limit
KP, KTH = 4.0, 4.0               # high gains: the command saturates for most of the segment
TOL, TOL_TH, SEG_TIMEOUT = 0.03, 0.08, 10.0
WINDOW = 0.3                     # s, Qualisys speed window (driver stamps)
D90 = math.pi / 2
AXES = ('vx', 'vy', 'wz')
# omnirace.yaml geometry, as used by omni_controller (wheel_ik.cpp)
WR, DX, DY, MA = 0.075, 0.235, 0.188, 135.0
ROT = DX - DY / math.tan(math.radians(MA))
WHEELS = ['LF_WHEEL', 'LH_WHEEL', 'RF_WHEEL', 'RH_WHEEL']
TOPICS = ['/tf', '/tf_static', '/omni_controller/twist_cmd', '/state_broadcaster/joints_state']


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def body_cmd(ex, ey, eth, th):
    """World error -> body twist, saturated at CMD_V / CMD_W (above the controller limits)."""
    c, s = math.cos(th), math.sin(th)
    vx, vy = KP * (c * ex + s * ey), KP * (-s * ex + c * ey)
    n = math.hypot(vx, vy)
    if n > CMD_V:
        vx, vy = vx * CMD_V / n, vy * CMD_V / n
    return vx, vy, max(-CMD_W, min(CMD_W, KTH * eth))


def waypoints(x0, y0, th0, L):
    return [(x0, y0, th0), (x0 + L, y0, th0), (x0, y0, th0), (x0 + L, y0, wrap(th0 + D90)),
            (x0, y0, wrap(th0 - D90)), (x0 + L, y0, th0)]


def wheel_body_velocity(w):
    """Mecanum forward kinematics of omni_controller; the IK sends wheel = -B v (omnirace)."""
    lf, lh, rf, rh = (-x for x in w)
    return (WR / 4 * (-lf - lh + rf + rh), WR / 4 * (lf - lh + rf - rh),
            WR / (4 * ROT) * (lf + lh + rf + rh))


class QualisysSpeed:
    """Body velocity from Qualisys poses, using the driver stamps (not the arrival time)."""
    def __init__(self):
        self.buf = []

    def add(self, stamp, p):
        if self.buf and stamp <= self.buf[-1][0]:
            return None                                  # same pose seen again
        self.buf.append((stamp, p))
        while stamp - self.buf[0][0] > WINDOW:
            self.buf.pop(0)
        (ta, a), (tb, b) = self.buf[0], self.buf[-1]
        if tb - ta < 0.8 * WINDOW:
            return None
        dt, th = tb - ta, a[2] + wrap(b[2] - a[2]) / 2   # heading at mid window
        wx, wy = (b[0] - a[0]) / dt, (b[1] - a[1]) / dt
        c, s = math.cos(th), math.sin(th)
        return c * wx + s * wy, -s * wx + c * wy, wrap(b[2] - a[2]) / dt




def main():
    _self_check()  # also when started as a ros2 entry point
    args = [a for a in sys.argv[1:] if a != 'nobag']
    L = float(args[0]) if args else 0.5
    assert 0.1 <= L <= 0.8, 'L between 0.1 and 0.8 m'
    rclpy.init()
    node = rclpy.create_node('limit_test')
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
    wheel = {'v': None}

    def on_js(m):
        idx = {n: i for i, n in enumerate(m.name)}
        if all(n in idx for n in WHEELS):
            wheel['v'] = wheel_body_velocity([m.velocity[idx[n]] for n in WHEELS])
    listen.create_subscription(JointsStates, '/state_broadcaster/joints_state', on_js, 1)  # newest only

    def pose():
        """(stamp [s], (x, y, th)) or None if missing, NaN or older than 0.5 s."""
        t = latest.get('tf')
        if t is None:
            return None
        st = Time.from_msg(t.header.stamp)
        if (node.get_clock().now() - st).nanoseconds * 1e-9 > 0.5:
            return None
        p = (t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation))
        return None if any(math.isnan(v) for v in p) else (st.nanoseconds * 1e-9, p)

    def publish(vx=0.0, vy=0.0, wz=0.0):
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = vx, vy, wz
        pub.publish(m)

    def stop():
        for _ in range(20):
            publish()
            rclpy.spin_once(node, timeout_sec=0.05)

    t0, sp0 = time.monotonic(), None
    while sp0 is None and time.monotonic() - t0 < 5.0:
        publish()
        rclpy.spin_once(node, timeout_sec=0.05)
        sp0 = pose()
    if sp0 is None:
        log.error('no fresh world -> base_link pose: is the Qualisys driver streaming?')
        return
    p0 = sp0[1]
    wps = waypoints(*p0, L)
    log.info(f'start ({p0[0]:+.3f}, {p0[1]:+.3f}, {math.degrees(p0[2]):+.0f} deg), L = {L} m, '
             f'commands up to {CMD_V} m/s / {CMD_W} rad/s (limits vx,vy {LIM[0]}, wz {LIM[2]})')

    bag = None
    if 'nobag' not in sys.argv[1:]:
        path = os.path.expanduser(f'~/mulinex_ws/bags/limit_test_{time.strftime("%Y%m%d_%H%M%S")}')
        os.makedirs(os.path.dirname(path), exist_ok=True)
        bag = subprocess.Popen(['ros2', 'bag', 'record', '-o', path] + TOPICS,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        log.info(f'recording rosbag {path}')

    node.get_logger().warn('Publishing now: the robot moves only if the controller is active (Y = activate, A = stop)')

    results, prev, last_ok, abort = [], p0, time.monotonic(), None
    try:
        for k, goal in enumerate(wps):
            qs, mq, mw = QualisysSpeed(), [0.0] * 3, [0.0] * 3
            ts, reached = time.monotonic(), False
            while time.monotonic() - ts < SEG_TIMEOUT:
                rclpy.spin_once(node, timeout_sec=0.0)
                now, sp = time.monotonic(), pose()
                if wheel['v'] is not None:
                    mw = [max(m, abs(v)) for m, v in zip(mw, wheel['v'])]
                if sp is None:
                    publish()
                    if now - last_ok > 0.3:
                        abort = 'pose lost'
                        break
                    time.sleep(0.02)
                    continue
                stamp, p = sp
                if math.hypot(p[0] - prev[0], p[1] - prev[1]) > 0.3:
                    abort = 'pose jump'
                    break
                last_ok, prev = now, p
                v = qs.add(stamp, p)
                if v:
                    mq = [max(m, abs(x)) for m, x in zip(mq, v)]
                ex, ey, eth = goal[0] - p[0], goal[1] - p[1], wrap(goal[2] - p[2])
                if math.hypot(ex, ey) < TOL and abs(eth) < TOL_TH:
                    reached = True
                    break
                publish(*body_cmd(ex, ey, eth, p[2]))
                time.sleep(0.02)
            results.append((reached, mq, mw))
            log.info(f'wp {k} ({goal[0]:+.2f}, {goal[1]:+.2f}, {math.degrees(goal[2]):+.0f} deg) '
                     f'{"reached" if reached else "NOT reached"} | max |vx| |vy| |wz|: '
                     f'qualisys {mq[0]:.2f} {mq[1]:.2f} {mq[2]:.2f}  '
                     f'wheels {mw[0]:.2f} {mw[1]:.2f} {mw[2]:.2f}')
            if abort:
                log.error(f'ABORT: {abort}')
                break
    finally:
        stop()
        if bag:
            bag.send_signal(signal.SIGINT)
            bag.wait(timeout=10)

    log.info('==================== RESULT (max per axis, body frame) ====================')
    for src, i in (('qualisys', 1), ('wheels  ', 2)):
        mx = [max((r[i][a] for r in results), default=0.0) for a in range(3)]
        verdict = ['PASS' if m <= lim * MARGIN else 'FAIL' for m, lim in zip(mx, LIM)]
        log.info(f'{src}: ' + '   '.join(f'{n} {m:.3f} (limit {lim}) {v}'
                                          for n, m, lim, v in zip(AXES, mx, LIM, verdict)))
    log.info(f'waypoints reached: {sum(r[0] for r in results)}/{len(wps)}'
             + (f'  (aborted: {abort})' if abort else ''))
    listen_exec.shutdown()
    listen.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


def _self_check():
    w = waypoints(1.0, 2.0, 0.0, 0.5)
    assert w[0] == (1.0, 2.0, 0.0) and w[1][:2] == (1.5, 2.0) and w[2][:2] == (1.0, 2.0)
    assert abs(w[3][2] - D90) < 1e-9
    vx, vy, wz = body_cmd(0.5, 0.0, 0.0, 0.0)           # far ahead: saturated at CMD_V
    assert abs(vx - CMD_V) < 1e-9 and abs(vy) < 1e-9 and wz == 0.0
    vx, vy, _ = body_cmd(0.5, 0.0, 0.0, D90)            # facing +y: move to its right
    assert abs(vx) < 1e-9 and abs(vy + CMD_V) < 1e-9
    # robot facing +y moving along world +x at 0.5 m/s -> body vy = -0.5
    q = QualisysSpeed()
    for i in range(16):
        r = q.add(i * 0.02, (0.01 * i, 0.0, D90))
    assert abs(r[0]) < 1e-9 and abs(r[1] + 0.5) < 1e-9 and abs(r[2]) < 1e-9
    assert q.add(15 * 0.02, (9.0, 9.0, 0.0)) is None    # repeated stamp ignored
    # wheel odometry inverts the controller IK (wheel = -B v)
    for v in [(0.5, 0.0, 0.0), (0.0, 0.5, 0.0), (0.0, 0.0, 1.0), (0.3, -0.2, 0.4)]:
        B = [(-1, 1, ROT), (-1, -1, ROT), (1, 1, ROT), (1, -1, ROT)]
        wh = [-sum(b * x for b, x in zip(row, v)) / WR for row in B]
        assert all(abs(a - b) < 1e-9 for a, b in zip(wheel_body_velocity(wh), v))


if __name__ == '__main__':
    main()
