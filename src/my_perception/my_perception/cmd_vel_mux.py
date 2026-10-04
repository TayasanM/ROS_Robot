#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class CmdVelPriorityMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_priority_mux')

        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel_joy', self.joy_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_tracker', self.tracker_callback, 10)

        # State tracking
        self.last_joy_active_time = 0.0
        self.joy_timeout = 0.35  # Seconds of joystick inactivity before returning to tracker
        self.stick_deadband = 0.05  # Filter out analog stick resting drift

        # Active commands
        self.latest_joy_cmd = Twist()
        self.latest_tracker_cmd = Twist()

        # Timer loop to publish smoothly at 30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self.mux_loop)
        self.get_logger().info("Priority Velocity Multiplexer Active: Joy has Priority 100 over YOLO Tracker.")

    def joy_callback(self, msg):
        self.latest_joy_cmd = msg
        # Check if joystick thumbstick is physically displaced beyond deadband
        is_active = (
            abs(msg.linear.x) > self.stick_deadband or
            abs(msg.linear.y) > self.stick_deadband or
            abs(msg.angular.z) > self.stick_deadband
        )
        if is_active:
            self.last_joy_active_time = time.time()

    def tracker_callback(self, msg):
        self.latest_tracker_cmd = msg

    def mux_loop(self):
        now = time.time()
        out_cmd = Twist()

        # If joystick was moved within the timeout window, give it exclusive control
        if (now - self.last_joy_active_time) < self.joy_timeout:
            out_cmd = self.latest_joy_cmd
        else:
            out_cmd = self.latest_tracker_cmd

        self.cmd_pub.publish(out_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPriorityMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
