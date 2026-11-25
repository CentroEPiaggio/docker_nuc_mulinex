#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math


D90 = math.pi / 2
D180 = math.pi
D270 = 3 * math.pi / 2
D45 = math.pi / 4
D135 = 3 * math.pi / 4
D225 = 5 * math.pi / 4
D315 = 7 * math.pi / 4
class OmniWaypointFollower(Node):
    def __init__(self):
        super().__init__('omni_waypoint_follower')

        # --- Configuration ---
        # List of waypoints: (x, y, theta) ---> 22 wpts

        # self.waypoints = [
        #     (0.5, 0.0, 0.0), #A
        #     (1.0, 0.0, -D90),#B
        #     (1.0, 1.0, D180), #C
        #     (1.5, 0.5, D180), #D
        #     (2.0, 0.0, D90), #E
        #     (2.0, 2.25, D45), #F
        #     (3.0, 0, -D90), #G
        #     (3.0, 1.0, D135), #H
        #     (4.0, 0.5, 0), #I
        #     (5.0, 0.5, 0), #J
        #     (4.0, 0.5, 0), #K == I
        #     (4.5, 1.5, -D90), #L
        #     (4.5, 2.25, -D90), #M 
        #     (3.5, 1.5, -D135), #N
        #     (3.5, 2.25, -D135), #O
        #     (3.0, 1.0, D135), #P ==H
        #     (4.0, 0.5, 0), # Q == I
        #     (4.5, 1.5, -D90), #R == L
        #     (4.5, 2.25, -D90), #S == M
        #     (2.0, 2.25, D45), #T == F
        #     (1.0, 2.0, D45), #U
        #     (0.5, 2.0, 0), #V
        # ]

        # Race track: start + 32 official waypoints.
        # Track reduced by 15 % toward the start (0, 0), x and y rounded to 0.1 m; headings unchanged.
        self.waypoints = [
            (0.0, 0.0, 0), # C
            (0.9, 0.9, -D90), # B
            (1.1, 0.4, -D45), # C
            (1.5, 0.2, 0), # D
            (1.9, 0.4, D45), # E
            (2.1, 0.9, 1.047), # F
            (2.3, 1.3, D45), # G
            (2.8, 1.5, 0), # H
            (3.2, 1.3, -D45), # I
            (3.4, 0.9, -D90), # J
            (3.2, 0.4, -2.094), # K
            (2.8, 0.2, D180), # L
            (2.3, 0.4, 2.094), # M
            (2.1, 0.9, 2.094), # N
            (1.9, 1.3, D135), # O
            (1.5, 1.5, D180), # P
            (1.1, 1.3, -2.094), # Q
            (0.9, 0.9, -D90), # R
            (0.9, 0.4, D90), # S
            (0.6, 1.7, 1.047), # T
            (1.1, 2.1, 0), # U
            (1.7, 1.7, D45), # V
            (1.7, 2.1, -1.047), # W
            (2.3, 2.1, -3.142), # X
            (3.0, 1.9, 2.618), # Y
            (3.4, 1.5, D135), # Z
            (3.6, 1.1, D90), # A1
            (3.4, 0.6, 0), # C1
            (3.0, 0.6, D180), # C1
            (2.6, 0.9, 2.618), # D1
            (2.1, 1.1, -2.880), # E1
            (1.3, 1.1, D90), # F1
            (0.0, 1.7, 0), # G1
        ]

        # Test course (kept, not official; uncomment to use), same workspace (x 0..3.6, y 0..2.1), about as long as the official
        # reduced track (simulated 47 s vs 46 s). Waypoints far apart (1.5 to 2.3 m after the
        # diagonal run). Waypoint 2 is the one long run that exploits the per-axis limits:
        # 2.8 m along +x with the body at -45 deg, i.e. a diagonal in the body frame
        # (vx = vy = 0.5 -> 0.71 m/s). Headings change gradually (30 deg, at most 45 deg)
        # between waypoints, except three sharp rotations: 90 deg at wp 5, 60 deg at wp 8,
        # 90 deg at wp 13. (x [m], y [m], theta [deg])
        # test_course_deg = [
        #     (0.0, 0.0, 0),       # start
        #     (0.4, 0.4, 0),
        #     (3.2, 0.4, -45),     # long diagonal run (body at -45 deg to the motion)
        #     (2.2, 1.9, -15), (0.4, 1.2, 15),
        #     (2.0, 0.4, 105),     # sharp: 90 deg
        #     (3.3, 1.8, 135), (1.2, 1.9, 165),
        #     (0.4, 0.6, -135),    # sharp: 60 deg
        #     (2.6, 1.2, -105), (1.0, 1.8, -75), (3.2, 1.4, -45), (2.0, 0.5, -15),
        #     (0.6, 1.6, 75),      # sharp: 90 deg
        #     (2.2, 2.0, 45), (0.0, 1.7, 0),
        # ]
        # self.waypoints = [(x, y, math.radians(t)) for x, y, t in test_course_deg]

        # Short test: 0.5 m straight along x, heading unchanged.
        # self.waypoints = [
        #     (0.00, 0.00, 0),  # start
        #     (0.50, 0.00, 0),
        # ]

        # Line test, y = LINE_Y, orientation changes at every waypoint (uncomment to use).
        # 180 -> -90 crosses +-pi; the last one is a rotation in place.
        # LINE_Y = 0.74  # line shifted in y, where the real-robot tests were run (track: 0.00)
        # self.waypoints = [
        #     (0.00, LINE_Y, 0),     # start
        #     (0.50, LINE_Y, 0),
        #     (1.00, LINE_Y, D90),
        #     (1.50, LINE_Y, D180),
        #     (2.00, LINE_Y, -D90),
        #     (2.50, LINE_Y, D45),
        #     (3.00, LINE_Y, -D135),
        #     (3.00, LINE_Y, 0),
        # ]
        self.current_wp_idx = 0

        # Current frame has a position offset of (1.75, 0.75), so i need to substract it from all the wpts
        # self.waypoints = [(x - 1.2, y - 0.75, theta) for (x, y, theta) in self.waypoints]

        # Constraints
        self.max_v_xy = 0.5      # m/s
        # self.max_v_xy = 0.2    # m/s, reduced for the first tests on the real robot
        self.max_v_theta = 1.0   # rad/s

        # Tolerances
        self.xy_tolerance = 0.1
        self.theta_tolerance = 0.15 # rad

        # Control gain (P-controller)
        # A gain of 1.0 means velocity command matches error until clamped by max velocity
        self.Kp_xy = 2.0
        self.Kp_theta = 2.0

        # --- ROS 2 Infrastructure ---
        # TF Listener to get odom -> base_link
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher for velocities
        cmd_vel_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        # Deadline: 105 ms
        cmd_vel_qos.deadline = Duration(seconds=0, nanoseconds=105000000)
        self.cmd_vel_pub = self.create_publisher(Twist, '/omni_controller/twist_cmd', cmd_vel_qos)

        # Timer for control loop (running at 20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('Omni Waypoint Follower started.')


    def get_yaw_from_quaternion(self, q):
        """Converts a geometry_msgs Quaternion to yaw (rotation around Z)."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Normalizes an angle to be within [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def control_loop(self):
        # Check if we finished all waypoints
        if self.current_wp_idx >= len(self.waypoints):
            self.stop_robot()
            self.get_logger().info('All waypoints reached!', once=True)
            return

        # 1. Get current pose from TF
        try:
            t = self.tf_buffer.lookup_transform(
                'world',
                'base_link',
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f'Waiting for TF: {ex}', throttle_duration_sec=2.0)
            return

        robot_x = t.transform.translation.x
        robot_y = t.transform.translation.y
        robot_theta = self.get_yaw_from_quaternion(t.transform.rotation)

        # 2. Get target waypoint
        target_x, target_y, target_theta = self.waypoints[self.current_wp_idx]

        # 3. Calculate Global Errors
        error_x_global = target_x - robot_x
        error_y_global = target_y - robot_y
        error_theta = self.normalize_angle(target_theta - robot_theta)
        
        distance = math.hypot(error_x_global, error_y_global)

        # 4. Check if waypoint is reached
        if distance <= self.xy_tolerance and abs(error_theta) <= self.theta_tolerance:
            self.get_logger().info(f'Waypoint {self.current_wp_idx} reached!')
            self.current_wp_idx += 1
            return

        # 5. Transform Global linear errors into Local (base_link) frame
        error_x_local = error_x_global * math.cos(robot_theta) + error_y_global * math.sin(robot_theta)
        error_y_local = -error_x_global * math.sin(robot_theta) + error_y_global * math.cos(robot_theta)

        # 6. Apply Proportional Control and Clamp to Max Velocities
        cmd_vx = self.Kp_xy * error_x_local
        cmd_vy = self.Kp_xy * error_y_local
        cmd_vtheta = self.Kp_theta * error_theta

        cmd_vx = max(-self.max_v_xy, min(self.max_v_xy, cmd_vx))
        cmd_vy = max(-self.max_v_xy, min(self.max_v_xy, cmd_vy))
        cmd_vtheta = max(-self.max_v_theta, min(self.max_v_theta, cmd_vtheta))

        # 7. Publish Twist message
        msg = Twist()
        msg.linear.x = float(cmd_vx)
        msg.linear.y = float(cmd_vy)
        msg.angular.z = float(cmd_vtheta)
        self.cmd_vel_pub.publish(msg)

    def stop_robot(self):
        """Sends a zero-velocity command to stop the robot safely."""
        msg = Twist()
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OmniWaypointFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()