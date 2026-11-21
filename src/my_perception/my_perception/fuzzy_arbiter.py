#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np


class FuzzyBehaviorArbiter(Node):
    def __init__(self):
        super().__init__('fuzzy_behavior_arbiter')

        # Subscribers
        self.sub_nav = self.create_subscription(
            Twist, '/cmd_vel_nav', self.nav_callback, 10)
        self.sub_track = self.create_subscription(
            Twist, '/cmd_vel_tracker', self.tracker_callback, 10)
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Publisher to robot base
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # Internal state
        self.cmd_nav = Twist()
        self.cmd_track = Twist()
        self.min_front_dist = 5.0  # default large distance

        # Heartbeat / watchdog timers (in seconds)
        self.last_nav_time = 0.0
        self.last_track_time = 0.0
        self.timeout_threshold = 0.5

        # Control loop timer (30 Hz)
        self.timer = self.create_timer(1.0 / 30.0, self.arbitration_loop)
        self.get_logger().info('Fuzzy Behavior Arbiter initialized at 30Hz.')

    def nav_callback(self, msg: Twist):
        self.cmd_nav = msg
        self.last_nav_time = self.get_clock().now().nanoseconds / 1e9

    def tracker_callback(self, msg: Twist):
        self.cmd_track = msg
        self.last_track_time = self.get_clock().now().nanoseconds / 1e9

    def scan_callback(self, msg: LaserScan):
        # Scan front 120-degree cone (-60 deg to +60 deg)
        ranges = np.array(msg.ranges)
        # Filter valid lidar returns
        valid = (ranges > msg.range_min) & (ranges < msg.range_max) & np.isfinite(ranges)
        
        if np.any(valid):
            # Focus on front sector (index 0 is forward on YDLidar TminiPro)
            n_points = len(ranges)
            front_span = int(n_points * (60.0 / 360.0))
            front_ranges = np.concatenate([ranges[-front_span:], ranges[:front_span]])
            valid_front = valid[np.concatenate([np.arange(n_points - front_span, n_points), np.arange(0, front_span)])]
            
            if np.any(valid_front):
                self.min_front_dist = float(np.min(front_ranges[valid_front]))
            else:
                self.min_front_dist = 5.0
        else:
            self.min_front_dist = 5.0

    def evaluate_fuzzy_memberships(self, d: float):
        """Triangular / Trapezoidal membership functions for obstacle distance."""
        # Critical: 1.0 at d <= 0.35, fades to 0.0 at 0.50
        if d <= 0.35:
            mu_critical = 1.0
        elif 0.35 < d < 0.50:
            mu_critical = (0.50 - d) / (0.50 - 0.35)
        else:
            mu_critical = 0.0

        # Caution: peaks around 0.60
        if d <= 0.40 or d >= 0.90:
            mu_caution = 0.0
        elif 0.40 < d <= 0.60:
            mu_caution = (d - 0.40) / (0.60 - 0.40)
        else:
            mu_caution = (0.90 - d) / (0.90 - 0.60)

        # Safe: 0.0 at d <= 0.70, rises to 1.0 at 1.0
        if d <= 0.70:
            mu_safe = 0.0
        elif 0.70 < d < 1.0:
            mu_safe = (d - 0.70) / (1.0 - 0.70)
        else:
            mu_safe = 1.0

        return mu_critical, mu_caution, mu_safe

    def arbitration_loop(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        track_alive = (current_time - self.last_track_time) < self.timeout_threshold
        nav_alive = (current_time - self.last_nav_time) < self.timeout_threshold

        # If neither node is publishing, keep robot stopped
        if not track_alive and not nav_alive:
            self.pub_cmd_vel.publish(Twist())
            return

        # Single active source fallbacks
        if track_alive and not nav_alive:
            # Only visual tracking active
            w_track, w_nav = 1.0, 0.0
        elif nav_alive and not track_alive:
            # Only Nav2 active
            w_track, w_nav = 0.0, 1.0
        else:
            # Both active: execute TSK Fuzzy Arbitration
            mu_crit, mu_caut, mu_safe = self.evaluate_fuzzy_memberships(self.min_front_dist)
            total_weight = mu_crit + mu_caut + mu_safe + 1e-6

            # TSK Rule consequence weights:
            # Rule 1 (Safe): w_track=0.85, w_nav=0.15
            # Rule 2 (Caution): w_track=0.20, w_nav=0.80
            # Rule 3 (Critical): w_track=0.0, w_nav=0.0 (Brake)
            w_track = (mu_safe * 0.85 + mu_caut * 0.20 + mu_crit * 0.0) / total_weight
            w_nav = (mu_safe * 0.15 + mu_caut * 0.80 + mu_crit * 0.0) / total_weight

        # Compute blended command (Mecanum holonomic: vx, vy, wz)
        cmd_out = Twist()
        cmd_out.linear.x = w_track * self.cmd_track.linear.x + w_nav * self.cmd_nav.linear.x
        cmd_out.linear.y = w_track * self.cmd_track.linear.y + w_nav * self.cmd_nav.linear.y
        cmd_out.angular.z = w_track * self.cmd_track.angular.z + w_nav * self.cmd_nav.angular.z

        # Hard emergency brake if obstacle is within critical deadband
        if self.min_front_dist < 0.25:
            cmd_out.linear.x = min(0.0, cmd_out.linear.x)  # allow only reversing or zero
            cmd_out.linear.y = 0.0

        self.pub_cmd_vel.publish(cmd_out)


def main(args=None):
    rclpy.init(args=args)
    node = FuzzyBehaviorArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd_vel.publish(Twist())  # safe stop
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()