import json
import math
from enum import Enum

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# ---------------------------------------------------------
# 1. State Definition
# ---------------------------------------------------------
class RaceState(Enum):
    IDLE = "IDLE"
    READY = "READY"
    STARTED = "STARTED"
    FINISHED = "FINISHED"


# ---------------------------------------------------------
# 2. Waypoint Tracker Logic
# ---------------------------------------------------------
class WaypointTracker:
    """Encapsulates all logic related to waypoints, tolerances, and penalties."""
    
    def __init__(self, waypoints, xy_tolerance=0.15, theta_tolerance=0.2):
        self.waypoints = waypoints
        self.total_wp = len(waypoints)
        self.xy_tolerance = xy_tolerance
        self.theta_tolerance = theta_tolerance
        self.reset()

    def reset(self):
        self.current_wp_idx = 0
        self.last_reached_wp_idx = None
        self.waypoint_min_error_m = [float('inf')] * self.total_wp
        self.waypoint_penalties = [0.0] * self.total_wp
        self.total_penalty = 0.0

    def _normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def update_penalty(self, waypoint_idx, x, y):
        if waypoint_idx is None or not (0 <= waypoint_idx < self.total_wp):
            return

        target_x, target_y, _ = self.waypoints[waypoint_idx]
        pos_error = math.hypot(x - target_x, y - target_y)

        # Track the closest the robot ever got to the waypoint
        if pos_error < self.waypoint_min_error_m[waypoint_idx]:
            self.waypoint_min_error_m[waypoint_idx] = pos_error

    def finalize_penalty(self, waypoint_idx):
        if waypoint_idx is None or not (0 <= waypoint_idx < self.total_wp):
            return

        min_error_m = self.waypoint_min_error_m[waypoint_idx]
        if math.isinf(min_error_m):
            return

        min_error_cm = min_error_m * 100.0
        self.waypoint_penalties[waypoint_idx] = max(0.0, min_error_cm - 5.0)
        self.total_penalty = sum(self.waypoint_penalties)

    def update(self, x, y, yaw):
        """Updates waypoint progress based on current pose. Returns True if finished."""
        if self.current_wp_idx >= self.total_wp:
            if self.last_reached_wp_idx is not None:
                self.update_penalty(self.last_reached_wp_idx, x, y)
                self.finalize_penalty(self.last_reached_wp_idx)
            self.total_penalty = sum(self.waypoint_penalties)
            return True # Finished

        target_x, target_y, target_theta = self.waypoints[self.current_wp_idx]
        pos_error = math.hypot(x - target_x, y - target_y)
        theta_error = abs(self._normalize_angle(yaw - target_theta))

        # Update penalty for current and previously reached waypoints
        self.update_penalty(self.current_wp_idx, x, y)
        if self.last_reached_wp_idx is not None and self.last_reached_wp_idx != self.current_wp_idx:
            self.update_penalty(self.last_reached_wp_idx, x, y)

        # Check if waypoint is reached
        if pos_error <= self.xy_tolerance and theta_error <= self.theta_tolerance:
            if self.last_reached_wp_idx is not None:
                self.finalize_penalty(self.last_reached_wp_idx)
            self.last_reached_wp_idx = self.current_wp_idx
            self.current_wp_idx += 1

        self.total_penalty = sum(self.waypoint_penalties)
        return False # Still racing

    def get_current_wp_display(self):
        return min(self.current_wp_idx + 1, self.total_wp)


# ---------------------------------------------------------
# 3. Main ROS 2 Node
# ---------------------------------------------------------
class RaceManagerNode(Node):
    def __init__(self):
        super().__init__('omnirace_telemetry_broadcaster')

        # --- Configuration & Parameters ---
        self.team_name = "Team 1"
        self.target_frame = 'world'
        self.target_frame = 'map'
        self.source_frame = 'base_link'
        self.source_frame = 'Omnirace'
        self.origin_xy_tolerance = 0.15
        self.origin_theta_tolerance = 0.2
        self.ready_delay_sec = 5.0
        self.cmd_vel_epsilon = 0.01

        waypoints = [
            (1.00, 1.00, -math.pi / 2), (1.25, 0.50, -math.pi / 4),
            (1.75, 0.25, 0.0),          (2.25, 0.50, math.pi / 4),
            (2.50, 1.00, math.pi / 3),  (2.75, 1.50, math.pi / 4),
            (3.25, 1.75, 0.0),          (3.75, 1.50, -math.pi / 4),
            (4.00, 1.00, -math.pi / 2), (3.75, 0.50, -2 * math.pi / 3),
            (3.25, 0.25, math.pi),      (2.75, 0.50, 2 * math.pi / 3),
            (2.50, 1.00, 2 * math.pi / 3),(2.25, 1.50, 3 * math.pi / 4),
            (1.75, 1.75, math.pi),      (1.25, 1.50, -2 * math.pi / 3),
            (1.00, 1.00, -math.pi / 2), (1.00, 0.50, math.pi / 2),
            (0.75, 2.00, math.pi / 3),  (1.25, 2.50, 0.0),
            (2.00, 2.00, math.pi / 4),  (2.00, 2.50, -math.pi / 3),
            (2.75, 2.50, -math.pi),     (3.50, 2.25, 5 * math.pi / 6),
            (4.00, 1.75, 3 * math.pi / 4),(4.25, 1.25, math.pi / 2),
            (4.00, 0.75, 0.0),          (3.50, 0.75, math.pi),
            (3.00, 1.00, 5 * math.pi / 6),(2.50, 1.25, -11 * math.pi / 12),
            (1.50, 1.25, math.pi / 2),  (0.00, 2.00, 0.0),
        ]

        self.waypoint_tracker = WaypointTracker(waypoints, xy_tolerance=0.15, theta_tolerance=0.2)

        # --- State Variables ---
        self.state = RaceState.IDLE
        self.start_time = None
        self.finished_elapsed_time = 0.0
        self.origin_hold_start_time = None
        
        self.latest_cmd_vel = {'vx': 0.0, 'vy': 0.0, 'wz': 0.0}
        self.movement_detected = False  # Latch to guarantee we don't miss quick commands

        # --- ROS Interfaces ---
        self.telemetry_pub = self.create_publisher(String, '/omnirace_telemetry', 10)
        self.transform_buffer = Buffer()
        self.tf_listener = TransformListener(self.transform_buffer, self)

        twist_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        twist_qos.deadline = Duration(seconds=0, nanoseconds=105_000_000)

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/omni_controller/twist_cmd', self.cmd_vel_callback, twist_qos)

        # Main loop running at 10Hz
        self.timer = self.create_timer(0.1, self.tick)

    # -----------------------------------------------------
    # ROS Callbacks
    # -----------------------------------------------------
    def cmd_vel_callback(self, msg):
        self.latest_cmd_vel = {
            'vx': float(msg.linear.x),
            'vy': float(msg.linear.y),
            'wz': float(msg.angular.z),
        }
        
        # If any command exceeds epsilon, trigger the movement latch
        if any(abs(v) > self.cmd_vel_epsilon for v in self.latest_cmd_vel.values()):
            self.movement_detected = True

    def tick(self):
        """Main loop: Get pose, update state machine, and publish telemetry."""
        pose = self._get_robot_pose()
        if pose is None:
            return  # Can't update state without TF

        self._process_state_machine(pose)
        self._publish_telemetry()
        
        # Reset movement latch *after* processing state machine
        self.movement_detected = False

    # -----------------------------------------------------
    # State Machine Logic (Centralized)
    # -----------------------------------------------------
    def _process_state_machine(self, pose):
        x, y, yaw = pose
        now = self.get_clock().now()

        if self.state == RaceState.IDLE:
            if self._is_at_origin(x, y, yaw):
                if self.origin_hold_start_time is None:
                    self.origin_hold_start_time = now
                elif (now - self.origin_hold_start_time).nanoseconds / 1e9 >= self.ready_delay_sec:
                    self.state = RaceState.READY
                    self.start_time = None
                    self.finished_elapsed_time = 0.0
                    self.get_logger().info("State -> READY")
            else:
                self.origin_hold_start_time = None

        elif self.state == RaceState.READY:
            if not self._is_at_origin(x, y, yaw):
                # If robot was pushed out of origin without commands, reset to IDLE
                self.state = RaceState.IDLE
                self.origin_hold_start_time = None
                self.get_logger().info("Pushed out of origin. State -> IDLE")
                
            elif self.movement_detected:
                self.state = RaceState.STARTED
                self.start_time = now
                self.finished_elapsed_time = 0.0
                self.waypoint_tracker.reset()
                self.get_logger().info("Movement detected. State -> STARTED")

        elif self.state == RaceState.STARTED:
            is_finished = self.waypoint_tracker.update(x, y, yaw)
            if is_finished and not self.movement_detected:
                if self.start_time is not None:
                    self.finished_elapsed_time = (
                        self.get_clock().now() - self.start_time
                    ).nanoseconds / 1e9
                self.state = RaceState.FINISHED
                self.get_logger().info("Track complete. State -> FINISHED")
            elif not is_finished and not self.movement_detected:
                # If robot stopped moving for 5 seconds, consider it idle (aborted)
                if self.start_time is not None:
                    elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                    if elapsed >= 5.0:
                        self.finished_elapsed_time = elapsed
                        self.state = RaceState.IDLE
                        self.waypoint_tracker.reset()
                        self.start_time = None
                        self.finished_elapsed_time = 0.0
                        self.get_logger().info("No movement for 5 seconds. State -> IDLE")

            # If out of bound: stop
            if abs(x) > 5.0 or abs(y) > 3.5 or x < -0.5:
                self.finished_elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                self.state = RaceState.IDLE
                self.waypoint_tracker.reset()
                self.start_time = None
                self.finished_elapsed_time = 0.0
                self.get_logger().info("Out of bounds. State -> IDLE") 


        elif self.state == RaceState.FINISHED:
            if self.movement_detected:
                self.state = RaceState.IDLE
                self.waypoint_tracker.reset()
                self.start_time = None
                self.finished_elapsed_time = 0.0
                self.get_logger().info("New movement. State -> IDLE")

    # -----------------------------------------------------
    # Helper Methods
    # -----------------------------------------------------
    def _is_at_origin(self, x, y, yaw):
        def normalize_angle(angle):
            return math.atan2(math.sin(angle), math.cos(angle))
            
        return (abs(x) <= self.origin_xy_tolerance and 
                abs(y) <= self.origin_xy_tolerance and 
                abs(normalize_angle(yaw)) <= self.origin_theta_tolerance)

    def _get_robot_pose(self):
        try:
            transform = self.transform_buffer.lookup_transform(
                self.target_frame, self.source_frame, rclpy.time.Time()
            )
            q = transform.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return transform.transform.translation.x, transform.transform.translation.y, yaw
            
        except TransformException as exc:
            self.get_logger().warn(f'Waiting for TF transform: {exc}', throttle_duration_sec=2.0)
            return None

    def _get_cmd_source_name(self):
        try:
            publishers = self.get_publishers_info_by_topic('/omni_controller/twist_cmd')
        except Exception:
            return 'unknown'

        if not publishers:
            return 'unknown'

        names = []
        for publisher in publishers:
            namespace = (publisher.node_namespace or '').rstrip('/')
            node_name = publisher.node_name or ''
            if namespace and node_name:
                names.append(f'{namespace}/{node_name}')
            elif node_name:
                names.append(node_name)

        if not names:
            return 'unknown'

        if len(names) == 1:
            return names[0]

        return ', '.join(sorted(names))

    def _publish_telemetry(self):
        elapsed = 0.0
        if self.state == RaceState.STARTED and self.start_time:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        elif self.state == RaceState.FINISHED:
            elapsed = self.finished_elapsed_time

        current_wp = 0
        if self.state in (RaceState.STARTED, RaceState.FINISHED):
            current_wp = self.waypoint_tracker.get_current_wp_display()

        state_dict = {
            "team_name": self.team_name,
            "cmd_source": self._get_cmd_source_name(),
            "cmd_source_topic": "/omni_controller/twist_cmd",
            "current_wp": current_wp,
            "total_wp": self.waypoint_tracker.total_wp,
            "elapsed_time": round(elapsed, 2),
            "penalties": round(self.waypoint_tracker.total_penalty, 2),
            "state": self.state.value,
            "status": self.state.value,
            "cmd_vel": self.latest_cmd_vel,
        }

        msg = String(data=json.dumps(state_dict))
        self.telemetry_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RaceManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()