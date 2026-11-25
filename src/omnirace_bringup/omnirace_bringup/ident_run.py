#!/usr/bin/env python3
"""Open-loop excitation of the Omnicar for model identification, recorded in a rosbag.

The robot receives PREDEFINED body-velocity profiles (no controller in the loop), so the data
show the robot response to steps, chirps and combined motions at several amplitudes.
After every block it goes back to the start pose in closed loop (slowly), so it stays in the
Qualisys area; those parts are labelled "return" and are not excitation data.

Blocks (python3 ident_run.py [steps|chirp|combo|all] [nobag]; default all):
  steps   vx, vy: +a / 0 / -a / 0 for a = 0.1, 0.2, 0.3, 0.5 m/s (<= 0.4 m per step)
          wz:     +a / 0 / -a / 0 for a = 0.25, 0.5, 1.0 rad/s (3 s each)
  chirp   vx, vy: 0.2 m/s, wz: 0.6 rad/s, sine sweeping 0.2 -> 2 Hz in 20 s
  combo   body diagonals (vx = +-vy), circles both ways (vx 0.15, wz +-0.5: radius 0.3 m,
          one full turn), a mixed motion (zero-mean sines on vx, vy, wz, 20 s)

Every segment start is published on /ident/segment (std_msgs/String "<label> <t0> <duration>")
and written to segments.csv next to the bag, so each part of the bag can be cut out.
Bag topics: /tf, /tf_static, /omni_controller/twist_cmd, /state_broadcaster/joints_state,
/omni_controller/distributors_state (battery voltage), /ident/segment.

Safety: no area limit (the operator stops the robot with A); it aborts only if the Qualisys
pose is lost for 0.3 s (the returns are closed loop). Commands stay
within the controller limits (0.5 m/s per axis, 1.0 rad/s). Keep the joystick e-stop (A) at
hand. It starts publishing at once and never calls the controller services:
the robot moves only while the controller is active (Y on the joystick).
Needs about 0.8 m free around the start pose.
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
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


RATE = 50.0                       # Hz, command rate (controller deadline is 0.105 s)
AREA = 0.8                        # m, the profiles stay within this distance (self-check only)
LIM_V, LIM_W = 0.5, 1.0           # omni_controller twist limits
RET_V, RET_W, RET_TOL, RET_TOL_TH, RET_TIMEOUT = 0.2, 0.6, 0.02, 0.05, 20.0
TOPICS = ['/tf', '/tf_static', '/omni_controller/twist_cmd', '/state_broadcaster/joints_state',
          '/omni_controller/distributors_state', '/ident/segment']


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


# ---------------------------------------------------------------- profiles
# A segment is (label, duration [s], f(t) -> (vx, vy, wz)); a block is a list of segments,
# followed by a closed-loop return to the start pose.

def const(vx=0.0, vy=0.0, wz=0.0):
    return lambda t: (vx, vy, wz)


def step_block(axis, amps, t_max, d_max, pause=1.0):
    seg = []
    for a in amps:
        T = min(t_max, d_max / a) if axis != 'wz' else t_max
        v = {'vx': (a, 0, 0), 'vy': (0, a, 0), 'wz': (0, 0, a)}[axis]
        seg += [(f'step_{axis}_+{a}', T, const(*v)), (f'zero', pause, const()),
                (f'step_{axis}_-{a}', T, const(*(-x for x in v))), ('zero', pause, const())]
    return seg


def chirp(axis, amp, f0=0.2, f1=2.0, T=20.0):
    def f(t):
        s = amp * math.sin(2 * math.pi * (f0 * t + (f1 - f0) * t * t / (2 * T)))
        return {'vx': (s, 0, 0), 'vy': (0, s, 0), 'wz': (0, 0, s)}[axis]
    return [(f'chirp_{axis}_{amp}', T, f), ('zero', 1.0, const())]


def program(which):
    blocks = []
    if which in ('steps', 'all'):
        blocks += [step_block('vx', [0.1, 0.2, 0.3, 0.5], 2.0, 0.4),
                   step_block('vy', [0.1, 0.2, 0.3, 0.5], 2.0, 0.4),
                   step_block('wz', [0.25, 0.5, 1.0], 3.0, None)]
    if which in ('chirp', 'all'):
        blocks += [chirp('vx', 0.2), chirp('vy', 0.2), chirp('wz', 0.6)]
    if which in ('combo', 'all'):
        a, vc, w = 0.2, 0.15, 0.5             # circle: radius vc / w = 0.3 m
        blocks += [
            [('diag_+x+y', 1.5, const(a, a)), ('zero', 1.0, const()),
             ('diag_-x-y', 1.5, const(-a, -a)), ('zero', 1.0, const()),
             ('diag_+x-y', 1.5, const(a, -a)), ('zero', 1.0, const()),
             ('diag_-x+y', 1.5, const(-a, a)), ('zero', 1.0, const())],
            [('circle_ccw', 2 * math.pi / w, const(vc, 0, w)), ('zero', 1.0, const()),
             ('circle_cw', 2 * math.pi / w, const(vc, 0, -w)), ('zero', 1.0, const())],
            # zero-mean sines on all axes (whole periods in 20 s), so it does not drift away
            [('mixed', 20.0, lambda t: (0.2 * math.sin(2 * math.pi * 0.2 * t),
                                        0.15 * math.sin(2 * math.pi * 0.3 * t),
                                        0.5 * math.sin(2 * math.pi * 0.15 * t))),
             ('zero', 1.0, const())],
        ]
    return blocks


def clamp_cmd(vx, vy, wz):
    """Limits of the controller, as float (Twist fields reject int: profiles contain 0)."""
    return (float(max(-LIM_V, min(LIM_V, vx))), float(max(-LIM_V, min(LIM_V, vy))),
            float(max(-LIM_W, min(LIM_W, wz))))


# ---------------------------------------------------------------- runtime



def main():
    _self_check()  # also when started as a ros2 entry point
    which = next((a for a in sys.argv[1:] if a in ('steps', 'chirp', 'combo', 'all')), 'all')
    rclpy.init()
    node = rclpy.create_node('ident_run')
    log = node.get_logger()
    qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.BEST_EFFORT, deadline=Duration(seconds=0.105))
    pub = node.create_publisher(Twist, '/omni_controller/twist_cmd', qos)
    seg_pub = node.create_publisher(String, '/ident/segment', 10)
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
    dt = 1.0 / RATE

    def pose():
        t = latest.get('tf')
        if t is None:
            return None
        if (node.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds * 1e-9 > 0.5:
            return None
        p = (t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation))
        return None if any(math.isnan(v) for v in p) else p

    def publish(vx=0.0, vy=0.0, wz=0.0):
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = clamp_cmd(vx, vy, wz)
        pub.publish(m)

    t0, p0 = time.monotonic(), None
    while p0 is None and time.monotonic() - t0 < 5.0:
        publish()
        rclpy.spin_once(node, timeout_sec=0.05)
        p0 = pose()
    if p0 is None:
        log.error('no fresh world -> base_link pose: is the Qualisys driver streaming?')
        return
    blocks = program(which)
    total = sum(d for b in blocks for _, d, _ in b)
    log.info(f'start ({p0[0]:+.3f}, {p0[1]:+.3f}, {math.degrees(p0[2]):+.0f} deg), '
             f'program "{which}": {len(blocks)} blocks, {total:.0f} s of excitation + returns, '
             f'no area limit: A = stop')

    bag, csv = None, None
    if 'nobag' not in sys.argv[1:]:
        path = os.path.expanduser(f'~/mulinex_ws/bags/ident_{which}_{time.strftime("%Y%m%d_%H%M%S")}')
        os.makedirs(os.path.dirname(path), exist_ok=True)
        bag = subprocess.Popen(['ros2', 'bag', 'record', '-o', path] + TOPICS,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        csv = open(path + '_segments.csv', 'w')
        csv.write('label,t_start_ros,duration_s\n')
        log.info(f'recording rosbag {path} (segments in {path}_segments.csv)')

    def mark(label, duration):
        t = node.get_clock().now().nanoseconds * 1e-9
        seg_pub.publish(String(data=f'{label} {t:.3f} {duration:.3f}'))
        if csv:
            csv.write(f'{label},{t:.3f},{duration:.3f}\n')
            csv.flush()

    def run_segment(label, duration, f):
        """Open loop. Returns None or 'pose lost'."""
        mark(label, duration)
        ts, last_ok = time.monotonic(), time.monotonic()
        while time.monotonic() - ts < duration:
            rclpy.spin_once(node, timeout_sec=0.0)
            p = pose()
            if p is None:
                if time.monotonic() - last_ok > 0.3:
                    return 'pose lost'
            else:
                last_ok = time.monotonic()
            publish(*f(time.monotonic() - ts))
            time.sleep(dt)
        return None

    def go_home():
        """Closed loop back to the start pose, slowly. Returns None or 'pose lost'."""
        mark('return', 0.0)
        ts, last_ok = time.monotonic(), time.monotonic()
        while time.monotonic() - ts < RET_TIMEOUT:
            rclpy.spin_once(node, timeout_sec=0.0)
            p = pose()
            if p is None:
                publish()
                if time.monotonic() - last_ok > 0.3:
                    return 'pose lost'
                time.sleep(dt)
                continue
            last_ok = time.monotonic()
            ex, ey, eth = p0[0] - p[0], p0[1] - p[1], wrap(p0[2] - p[2])
            if math.hypot(ex, ey) < RET_TOL and abs(eth) < RET_TOL_TH:
                break
            c, s = math.cos(p[2]), math.sin(p[2])
            vx, vy = 1.5 * (c * ex + s * ey), 1.5 * (-s * ex + c * ey)
            n = math.hypot(vx, vy)
            if n > RET_V:
                vx, vy = vx * RET_V / n, vy * RET_V / n
            elif 0.0 < n < 0.025:                        # static friction on the floor
                vx, vy = vx * 0.025 / n, vy * 0.025 / n
            publish(vx, vy, max(-RET_W, min(RET_W, 1.5 * eth)))
            time.sleep(dt)
        for _ in range(int(RATE * 0.5)):
            publish()
            time.sleep(dt)
        return None

    node.get_logger().warn('Publishing now: the robot moves only if the controller is active (Y = activate, A = stop)')
    abort = None
    try:
        for b, block in enumerate(blocks, 1):
            log.info(f'block {b}/{len(blocks)}: ' + ', '.join(l for l, _, _ in block if l != 'zero'))
            for label, duration, f in block:
                r = run_segment(label, duration, f)
                if r:
                    abort = r
                    break
            if abort:
                break
            r = go_home()
            if r:
                abort = r
                break
    finally:
        for _ in range(int(RATE)):
            publish()
            time.sleep(dt)
        if bag:
            bag.send_signal(signal.SIGINT)
            bag.wait(timeout=10)
        if csv:
            csv.close()
    log.info('ABORTED: ' + abort if abort else 'done: all blocks completed')
    listen_exec.shutdown()
    listen.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


def _self_check():
    for which in ('steps', 'chirp', 'combo', 'all'):
        for block in program(which):
            for label, d, f in block:
                assert d > 0, label
                for k in range(int(d * 50) + 1):
                    vx, vy, wz = f(k / 50.0)
                    assert all(type(v) is float for v in clamp_cmd(vx, vy, wz)), label
                    assert abs(vx) <= LIM_V + 1e-9 and abs(vy) <= LIM_V + 1e-9, label
                    assert abs(wz) <= LIM_W + 1e-9, label
                if label.startswith(('step_vx', 'step_vy', 'diag')):
                    v = f(0.0)
                    assert math.hypot(v[0], v[1]) * d <= 0.45, label   # stays in the area
    # a full circle comes back to the start: v / w radius, 2 pi / w duration
    circ = [s for s in program('combo')[1] if s[0] == 'circle_ccw'][0]
    assert abs(circ[1] * circ[2](0)[2] - 2 * math.pi) < 1e-9
    # every block, integrated as a kinematic omnidirectional base, stays well inside AREA
    for block in program('all'):
        x = y = th = dmax = 0.0
        for label, d, f in block:
            for k in range(int(d * 100)):
                vx, vy, wz = f(k / 100.0)
                c, s = math.cos(th), math.sin(th)
                x, y, th = x + (c * vx - s * vy) / 100, y + (s * vx + c * vy) / 100, th + wz / 100
                dmax = max(dmax, math.hypot(x, y))
        assert dmax <= AREA - 0.15, (block[0][0], dmax)


if __name__ == '__main__':
    main()
