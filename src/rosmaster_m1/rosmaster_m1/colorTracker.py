#!/usr/bin/env python3

import time
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


@dataclass
class MouseSelection:
    start: Optional[Tuple[int, int]] = None
    end: Optional[Tuple[int, int]] = None
    active: bool = False


class SimplePid:
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()

    def compute(self, value: float, target: float) -> float:
        now = time.time()
        dt = max(now - self.last_time, 1e-3)
        error = value - target
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        self.last_error = error
        self.last_time = now
        return self.kp * error + self.ki * self.integral + self.kd * derivative


class ColorFollow(Node):
    def __init__(self):
        super().__init__('color_follow')

        self.declare_parameter('rgb_topic', '/ascamera_hp60c/camera_publisher/rgb0/image')
        self.declare_parameter('depth_topic', '/ascamera_hp60c/camera_publisher/depth0/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('joy_state_topic', '/JoyState')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('target_distance_mm', 500.0)
        self.declare_parameter('distance_deadband_mm', 40.0)
        self.declare_parameter('center_deadband_px', 45.0)
        self.declare_parameter('max_linear_speed', 0.8)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('linear_kp', 0.0005)
        self.declare_parameter('linear_ki', 0.0)
        self.declare_parameter('linear_kd', 0.0005)
        self.declare_parameter('angular_kp', 0.018)
        self.declare_parameter('angular_ki', 0.0)
        self.declare_parameter('angular_kd', 0.005)
        self.declare_parameter('h_min', 0)
        self.declare_parameter('s_min', 85)
        self.declare_parameter('v_min', 126)
        self.declare_parameter('h_max', 9)
        self.declare_parameter('s_max', 253)
        self.declare_parameter('v_max', 253)
        self.declare_parameter('min_radius', 10.0)
        self.declare_parameter('show_window', True)
        self.declare_parameter('flip_image', False)

        self.rgb_topic = self.get_parameter('rgb_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.joy_state_topic = self.get_parameter('joy_state_topic').value
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.target_distance_mm = float(self.get_parameter('target_distance_mm').value)
        self.distance_deadband_mm = float(self.get_parameter('distance_deadband_mm').value)
        self.center_deadband_px = float(self.get_parameter('center_deadband_px').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.min_radius = float(self.get_parameter('min_radius').value)
        self.show_window = bool(self.get_parameter('show_window').value)
        self.flip_image = bool(self.get_parameter('flip_image').value)

        self.linear_pid = SimplePid(
            float(self.get_parameter('linear_kp').value),
            float(self.get_parameter('linear_ki').value),
            float(self.get_parameter('linear_kd').value),
        )
        self.angular_pid = SimplePid(
            float(self.get_parameter('angular_kp').value),
            float(self.get_parameter('angular_ki').value),
            float(self.get_parameter('angular_kd').value),
        )

        self.bridge = CvBridge()
        self.mouse = MouseSelection()
        self.hsv_range = self.load_hsv_from_parameters()
        self.tracking_enabled = False
        self.joy_active = False
        self.last_seen_time = time.time()
        self.window_name = 'ROSMaster M1 Color Follow'

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Bool, self.joy_state_topic, self.joy_callback, 10)

        rgb_sub = Subscriber(self, Image, self.rgb_topic)
        depth_sub = Subscriber(self, Image, self.depth_topic)
        self.sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=0.5)
        self.sync.registerCallback(self.frame_callback)

        if self.show_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback(self.window_name, self.mouse_callback)

        self.get_logger().info('Color follow node is ready')

    def load_hsv_from_parameters(self):
        lower = np.array([
            int(self.get_parameter('h_min').value),
            int(self.get_parameter('s_min').value),
            int(self.get_parameter('v_min').value),
        ], dtype=np.uint8)
        upper = np.array([
            int(self.get_parameter('h_max').value),
            int(self.get_parameter('s_max').value),
            int(self.get_parameter('v_max').value),
        ], dtype=np.uint8)
        return lower, upper

    def joy_callback(self, msg: Bool):
        self.joy_active = msg.data
        if self.joy_active:
            self.stop_robot()

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.mouse.start = (x, y)
            self.mouse.end = (x, y)
            self.mouse.active = True
            self.tracking_enabled = False
            self.stop_robot()
        elif event == cv2.EVENT_MOUSEMOVE and self.mouse.active:
            self.mouse.end = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and self.mouse.active:
            self.mouse.end = (x, y)
            self.mouse.active = False
            self.tracking_enabled = True

    def frame_callback(self, rgb_msg: Image, depth_msg: Image):
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as error:
            self.get_logger().warning(f'Image conversion failed: {error}')
            return

        rgb = cv2.resize(rgb, (self.image_width, self.image_height))
        depth = cv2.resize(depth, (self.image_width, self.image_height))

        if self.flip_image:
            rgb = cv2.flip(rgb, 1)
            depth = cv2.flip(depth, 1)

        display = rgb.copy()
        key = cv2.waitKey(1) & 0xFF if self.show_window else 255

        if key == ord('r'):
            self.reset()
        elif key == ord('q'):
            rclpy.shutdown()
            return
        elif key == ord('i'):
            self.hsv_range = self.load_hsv_from_parameters()
            self.tracking_enabled = True
        elif key == 32:
            self.tracking_enabled = True

        if self.mouse.active and self.mouse.start and self.mouse.end:
            self.draw_selection(display)
            self.update_hsv_from_selection(rgb)

        if self.tracking_enabled:
            target = self.find_color_target(rgb)
            if target is not None:
                cx, cy, radius, mask = target
                distance = self.get_depth_mm(depth, cx, cy)
                self.control_robot(cx, distance)
                self.draw_target(display, cx, cy, radius, distance)
                self.last_seen_time = time.time()
                if self.show_window:
                    cv2.imshow(self.window_name + ' Mask', mask)
            else:
                if time.time() - self.last_seen_time > 0.5:
                    self.stop_robot()

        if self.show_window:
            cv2.imshow(self.window_name, display)

    def update_hsv_from_selection(self, image):
        if not self.mouse.start or not self.mouse.end:
            return
        x1, y1 = self.mouse.start
        x2, y2 = self.mouse.end
        x_min, x_max = sorted([x1, x2])
        y_min, y_max = sorted([y1, y2])
        if x_max - x_min < 5 or y_max - y_min < 5:
            return

        roi = image[y_min:y_max, x_min:x_max]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        lower = np.array([
            max(0, int(np.percentile(h, 10)) - 5),
            max(0, int(np.percentile(s, 10)) - 20),
            max(0, int(np.percentile(v, 10)) - 20),
        ], dtype=np.uint8)

        upper = np.array([
            min(179, int(np.percentile(h, 90)) + 5),
            min(255, int(np.percentile(s, 90)) + 20),
            min(255, int(np.percentile(v, 90)) + 20),
        ], dtype=np.uint8)

        self.hsv_range = lower, upper

    def find_color_target(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower, upper = self.hsv_range
        mask = cv2.inRange(hsv, lower, upper)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        if area < 100:
            return None

        (x, y), radius = cv2.minEnclosingCircle(contour)
        if radius < self.min_radius:
            return None

        return int(x), int(y), int(radius), mask

    def get_depth_mm(self, depth, cx: int, cy: int) -> float:
        h, w = depth.shape[:2]
        x0 = max(cx - 2, 0)
        x1 = min(cx + 3, w)
        y0 = max(cy - 2, 0)
        y1 = min(cy + 3, h)
        roi = depth[y0:y1, x0:x1].astype(np.float32)
        valid = roi[np.isfinite(roi)]
        valid = valid[valid > 0]
        if valid.size == 0:
            return 0.0
        value = float(np.median(valid))
        if value < 20.0:
            value *= 1000.0
        return value

    def control_robot(self, cx: int, distance_mm: float):
        if self.joy_active:
            return

        twist = Twist()
        image_center = self.image_width / 2.0

        if distance_mm > 0.0:
            linear = self.linear_pid.compute(distance_mm, self.target_distance_mm)
            if abs(distance_mm - self.target_distance_mm) < self.distance_deadband_mm:
                linear = 0.0
            twist.linear.x = self.limit(linear, -self.max_linear_speed, self.max_linear_speed)

        angular = self.angular_pid.compute(float(cx), image_center)
        if abs(cx - image_center) < self.center_deadband_px:
            angular = 0.0
        twist.angular.z = self.limit(-angular, -self.max_angular_speed, self.max_angular_speed)

        if distance_mm <= 0.0:
            twist.linear.x = 0.0

        self.cmd_pub.publish(twist)

    def draw_selection(self, image):
        x1, y1 = self.mouse.start
        x2, y2 = self.mouse.end
        cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 0), 2)

    def draw_target(self, image, cx, cy, radius, distance):
        cv2.circle(image, (cx, cy), radius, (0, 255, 0), 2)
        cv2.circle(image, (cx, cy), 4, (0, 0, 255), -1)
        text = f'x:{cx} y:{cy} dist:{int(distance)}mm'
        cv2.putText(image, text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    def reset(self):
        self.tracking_enabled = False
        self.mouse = MouseSelection()
        self.linear_pid.reset()
        self.angular_pid.reset()
        self.stop_robot()

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def limit(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def destroy_node(self):
        self.stop_robot()
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ColorFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
