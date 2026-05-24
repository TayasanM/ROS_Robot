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
class RoiState:
    start: Optional[Tuple[int, int]] = None
    box: Optional[Tuple[int, int, int, int]] = None
    drawing: bool = False


class SimplePid:
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()

    def set_gains(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd

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


class KcfFollow(Node):
    def __init__(self):
        super().__init__('kcf_follow')

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
        self.declare_parameter('show_window', True)
        self.declare_parameter('flip_image', False)
        self.declare_parameter('auto_start_tracking', False)
        self.declare_parameter('x1', 0)
        self.declare_parameter('y1', 0)
        self.declare_parameter('x2', 0)
        self.declare_parameter('y2', 0)

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
        self.show_window = bool(self.get_parameter('show_window').value)
        self.flip_image = bool(self.get_parameter('flip_image').value)
        self.auto_start_tracking = bool(self.get_parameter('auto_start_tracking').value)

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
        self.roi = RoiState()
        self.tracker = None
        self.tracking = False
        self.joy_active = False
        self.latest_frame = None
        self.last_distance = 0.0
        self.window_name = 'ROSMaster M1 KCF Follow'

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Bool, self.joy_state_topic, self.joy_callback, 10)

        rgb_sub = Subscriber(self, Image, self.rgb_topic)
        depth_sub = Subscriber(self, Image, self.depth_topic)
        self.sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=0.5)
        self.sync.registerCallback(self.frame_callback)

        if self.show_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback(self.window_name, self.mouse_callback)

        self.get_logger().info('KCF follow node is ready')

    def joy_callback(self, msg: Bool):
        self.joy_active = msg.data
        if self.joy_active:
            self.stop_robot()

    def make_tracker(self):
        if hasattr(cv2, 'legacy') and hasattr(cv2.legacy, 'TrackerKCF_create'):
            return cv2.legacy.TrackerKCF_create()
        if hasattr(cv2, 'TrackerKCF_create'):
            return cv2.TrackerKCF_create()
        raise RuntimeError('OpenCV KCF tracker is not available. Install opencv-contrib-python or ros-humble-vision-opencv.')

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.roi.start = (x, y)
            self.roi.box = None
            self.roi.drawing = True
            self.tracking = False
            self.stop_robot()

        elif event == cv2.EVENT_MOUSEMOVE and self.roi.drawing and self.roi.start:
            self.roi.box = self.normalize_box(self.roi.start[0], self.roi.start[1], x, y)

        elif event == cv2.EVENT_LBUTTONUP and self.roi.drawing and self.roi.start:
            self.roi.drawing = False
            self.roi.box = self.normalize_box(self.roi.start[0], self.roi.start[1], x, y)
            self.start_tracker_from_box(self.roi.box)

    def normalize_box(self, x1: int, y1: int, x2: int, y2: int):
        x = min(x1, x2)
        y = min(y1, y2)
        w = abs(x2 - x1)
        h = abs(y2 - y1)
        return x, y, w, h

    def start_tracker_from_box(self, box):
        if self.latest_frame is None or box is None:
            return
        x, y, w, h = box
        if w < 10 or h < 10:
            return
        self.tracker = self.make_tracker()
        self.tracker.init(self.latest_frame, tuple(box))
        self.tracking = True
        self.linear_pid.reset()
        self.angular_pid.reset()

    def try_auto_start(self):
        if not self.auto_start_tracking or self.tracking or self.latest_frame is None:
            return
        x1 = int(self.get_parameter('x1').value)
        y1 = int(self.get_parameter('y1').value)
        x2 = int(self.get_parameter('x2').value)
        y2 = int(self.get_parameter('y2').value)
        box = self.normalize_box(x1, y1, x2, y2)
        self.start_tracker_from_box(box)

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

        self.latest_frame = rgb.copy()
        self.try_auto_start()

        display = rgb.copy()
        key = cv2.waitKey(1) & 0xFF if self.show_window else 255

        if key == ord('r'):
            self.reset_tracker()
        elif key == ord('q'):
            rclpy.shutdown()
            return

        if self.tracking and self.tracker is not None:
            ok, box = self.tracker.update(rgb)
            if ok:
                x, y, w, h = [int(v) for v in box]
                cx = x + w // 2
                cy = y + h // 2
                distance = self.get_depth_mm(depth, cx, cy)
                self.control_robot(cx, distance)
                self.draw_tracking(display, x, y, w, h, cx, cy, distance)
            else:
                self.reset_tracker()

        if self.roi.drawing and self.roi.box is not None:
            x, y, w, h = self.roi.box
            cv2.rectangle(display, (x, y), (x + w, y + h), (255, 0, 0), 2)

        if self.show_window:
            cv2.imshow(self.window_name, display)

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
            self.last_distance = distance_mm

        angular = self.angular_pid.compute(float(cx), image_center)
        if abs(cx - image_center) < self.center_deadband_px:
            angular = 0.0
        twist.angular.z = self.limit(-angular, -self.max_angular_speed, self.max_angular_speed)

        if distance_mm <= 0.0:
            twist.linear.x = 0.0

        self.cmd_pub.publish(twist)

    def draw_tracking(self, image, x, y, w, h, cx, cy, distance):
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(image, (cx, cy), 4, (0, 0, 255), -1)
        text = f'x:{cx} y:{cy} dist:{int(distance)}mm'
        cv2.putText(image, text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 2)

    def reset_tracker(self):
        self.tracker = None
        self.tracking = False
        self.roi = RoiState()
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
    node = KcfFollow()
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
