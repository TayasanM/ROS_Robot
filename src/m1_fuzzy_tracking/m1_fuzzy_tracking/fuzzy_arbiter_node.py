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
        self.sub_joy = self.create_subscription(
            Twist, '/cmd_vel_joy', self.joy_callback, 10)
        self.sub_nav = self.create_subscription(
            Twist, '/cmd_vel_nav', self.nav_callback, 10)
        self.sub_track = self.create_subscription(
            Twist, '/cmd_vel_tracker', self.tracker_callback, 10)
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Output to chassis driver
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # State storage
        self.cmd_joy = Twist()
        self.cmd_nav = Twist()
        self.cmd_track = Twist()
        self.min_front_dist = 5.0

        # Heartbeats & thresholds
        self.last_joy_time = 0.0
        self.last_nav_time = 0.0
        self.last_track_time = 0.0
        self.timeout_threshold = 0.4
        self.velocity_deadband = 0.02

        # 30 Hz Arbitration Loop
        self.timer = self.create_timer(1.0 / 30.0, self.arbitration_loop)
        self.get_logger().info('Priority Behavior Arbiter with Fuzzy Safety ready.')

    def joy_callback(self, msg: Twist):
        self.cmd_joy = msg
        self.last_joy_time = self.get_clock().now().nanoseconds / 1e9

    def nav_callback(self, msg: Twist):
        self.cmd_nav = msg
        self.last_nav_time = self.get_clock().now().nanoseconds / 1e9

    def tracker_callback(self, msg: Twist):
        self.cmd_track = msg
        self.last_track_time = self.get_clock().now().nanoseconds / 1e9

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        valid = (ranges > msg.range_min) & (ranges < msg.range_max) & np.isfinite(ranges)
        
        if np.any(valid):
            n_points = len(ranges)
            front_span = int(n_points * (60.0 / 360.0))  # Front 120-degree cone
            front_ranges = np.concatenate([ranges[-front_span:], ranges[:front_span]])
            valid_front = valid[np.concatenate([np.arange(n_points - front_span, n_points), np.arange(0, front_span)])]
            
            if np.any(valid_front):
                self.min_front_dist = float(np.min(front_ranges[valid_front]))
            else:
                self.min_front_dist = 5.0
        else:
            self.min_front_dist = 5.0

    def is_active(self, last_time: float, cmd: Twist, current_time: float) -> bool:
        """Checks if a command topic is actively publishing non-zero commands."""
        if (current_time - last_time) > self.timeout_threshold:
            return False
        return (abs(cmd.linear.x) > self.velocity_deadband or
                abs(cmd.linear.y) > self.velocity_deadband or
                abs(cmd.angular.z) > self.velocity_deadband)

    def compute_fuzzy_safety_factor(self, d: float) -> float:
        """
        Calculates a smooth speed scaling factor [0.0, 1.0] from LiDAR proximity.
        - d <= 0.25m: Factor = 0.0 (Emergency Stop)
        - 0.25m < d < 0.65m: Smooth ramp (Slowdown)
        - d >= 0.65m: Factor = 1.0 (Full Speed)
        """
        if d <= 0.25:
            return 0.0
        elif d >= 0.65:
            return 1.0
        else:
            # Linear membership transition
            return (d - 0.25) / (0.65 - 0.25)

    def arbitration_loop(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        safety_scale = self.compute_fuzzy_safety_factor(self.min_front_dist)

        cmd_out = Twist()

        # =======================================================
        # 1. JOYSTICK OVERRIDE (Highest Priority)
        # =======================================================
        if self.is_active(self.last_joy_time, self.cmd_joy, current_time):
            cmd_out.linear.x = self.cmd_joy.linear.x
            cmd_out.linear.y = self.cmd_joy.linear.y
            cmd_out.angular.z = self.cmd_joy.angular.z
            if self.min_front_dist < 0.25 and cmd_out.linear.x > 0.0:
                cmd_out.linear.x = 0.0  # Prevent forward impact during manual driving
            self.pub_cmd_vel.publish(cmd_out)
            return

        # =======================================================
        # 2. NAV2 NAVIGATION (Available -> Run WITHOUT tracking)
        # =======================================================
        if self.is_active(self.last_nav_time, self.cmd_nav, current_time):
            cmd_out.linear.x = self.cmd_nav.linear.x * safety_scale
            cmd_out.linear.y = self.cmd_nav.linear.y * safety_scale
            cmd_out.angular.z = self.cmd_nav.angular.z
            self.pub_cmd_vel.publish(cmd_out)
            return

        # =======================================================
        # 3. VISUAL TRACKING (Otherwise -> Follow tracking)
        # =======================================================
        if self.is_active(self.last_track_time, self.cmd_track, current_time):
            cmd_out.linear.x = self.cmd_track.linear.x * safety_scale
            cmd_out.linear.y = self.cmd_track.linear.y * safety_scale
            cmd_out.angular.z = self.cmd_track.angular.z
            self.pub_cmd_vel.publish(cmd_out)
            return

        # =======================================================
        # 4. IDLE (No active source -> Stop)
        # =======================================================
        self.pub_cmd_vel.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = FuzzyBehaviorArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd_vel.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()