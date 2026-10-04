#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

import message_filters
import numpy as np
import json
import os
import time
from ultralytics import YOLO

import tf2_ros
import tf2_geometry_msgs

class YoloMapGroundingNode(Node):
    def __init__(self):
        super().__init__('yolo_map_grounding')
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")
        
        self.output_json_path = os.path.expanduser("~/map_landmarks.json")
        self.detected_landmarks = {}

        # Camera Intrinsics (Defaults, updated dynamically via CameraInfo)
        self.fx = 500.0
        self.fy = 500.0
        self.cx = 320.0
        self.cy = 240.0
        self.target_frame = "map"

        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers & Buffers
        self.marker_pub = self.create_publisher(MarkerArray, '/detected_objects_markers', 10)
        self.latest_rgb_msg = None
        self.latest_depth_msg = None
        self.latest_display_frame = None
        self.last_inference_time = 0.0

        # Subscriptions
        self.create_subscription(
            CameraInfo,
            '/ascamera_hp60c/camera_publisher/rgb0/camera_info',
            self.camera_info_callback,
            10
        )

        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/ascamera_hp60c/camera_publisher/rgb0/image', qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/ascamera_hp60c/camera_publisher/depth0/image_raw', qos_profile=qos_profile_sensor_data)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=2, slop=0.1)
        self.ts.registerCallback(self.sync_callback)

        self.get_logger().info("YOLO Grounding Node initialized on Jetson Orin.")

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.cx = msg.k[2]
        self.fy = msg.k[4]
        self.cy = msg.k[5]

    def sync_callback(self, rgb_msg, depth_msg):
        self.latest_rgb_msg = rgb_msg
        self.latest_depth_msg = depth_msg

    def process_frame(self):
        now = time.time()
        if (now - self.last_inference_time) < 0.12 or self.latest_rgb_msg is None:
            return
        self.last_inference_time = now

        rgb_msg = self.latest_rgb_msg
        depth_msg = self.latest_depth_msg

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            depth_image = np.nan_to_num(depth_image, nan=0.0)

            results = self.model(rgb_image, conf=0.45, verbose=False)
            boxes = results[0].boxes
            marker_array = MarkerArray()
            camera_frame_id = depth_msg.header.frame_id if depth_msg.header.frame_id else "ascamera_hp60c_camera_frame"

            has_map_tf = self.tf_buffer.can_transform(
                self.target_frame, 
                camera_frame_id, 
                Time()
            )

            for box in boxes:
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]
                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())

                u, v = int((x1 + x2) / 2), int((y1 + y2) / 2)
                v_min, v_max = max(0, v - 2), min(depth_image.shape[0], v + 3)
                u_min, u_max = max(0, u - 2), min(depth_image.shape[1], u + 3)
                depth_window = depth_image[v_min:v_max, u_min:u_max]
                valid = depth_window[depth_window > 0]

                if len(valid) == 0:
                    continue

                raw_z = np.median(valid)
                z_m = raw_z / 1000.0 if depth_image.dtype == np.uint16 else float(raw_z)
                if z_m < 0.3 or z_m > 4.5:
                    continue

                x_cam = (u - self.cx) * z_m / self.fx
                y_cam = (v - self.cy) * z_m / self.fy
                z_cam = z_m

                label_text = f"{cls_name} ({z_m:.2f}m)"

                if has_map_tf:
                    try:
                        pt = PointStamped()
                        pt.header.stamp = depth_msg.header.stamp
                        pt.header.frame_id = camera_frame_id
                        pt.point.x, pt.point.y, pt.point.z = x_cam, y_cam, z_cam

                        pt_map = self.tf_buffer.transform(pt, self.target_frame)
                        mx, my, mz = round(pt_map.point.x, 2), round(pt_map.point.y, 2), round(pt_map.point.z, 2)
                        
                        self.register_landmark(cls_name, mx, my, mz)
                        marker_array.markers.append(self.create_marker(cls_name, mx, my, mz, len(marker_array.markers)))
                        label_text = f"{cls_name} [Map: {mx},{my}]"
                    except Exception:
                        pass

                cv.rectangle(rgb_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv.putText(rgb_image, label_text, (x1, max(20, y1 - 6)),
                           cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            if len(marker_array.markers) > 0:
                self.marker_pub.publish(marker_array)

            self.latest_display_frame = rgb_image

        except Exception as e:
            self.get_logger().error(f"Process error: {e}")

    def register_landmark(self, label, x, y, z):
        for obj_id, data in self.detected_landmarks.items():
            if data["label"] == label and np.hypot(data["x"] - x, data["y"] - y) < 0.4:
                return
        lid = f"{label}_{len(self.detected_landmarks) + 1}"
        self.detected_landmarks[lid] = {"label": label, "x": x, "y": y, "z": z}
        self.get_logger().info(f"Registered {lid} at Map [{x}, {y}, {z}]")
        with open(self.output_json_path, 'w') as f:
            json.dump(self.detected_landmarks, f, indent=2)

    def create_marker(self, label, x, y, z, mid):
        m = Marker()
        m.header.frame_id = self.target_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = "yolo_landmarks", mid
        m.type, m.action = Marker.SPHERE, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.color.a, m.color.r, m.color.g, m.color.b = 0.8, 1.0, 0.3, 0.0
        m.lifetime = rclpy.duration.Duration(seconds=2.0).to_msg()
        return m

def main(args=None):
    rclpy.init(args=args)
    node = YoloMapGroundingNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)
            node.process_frame()

            if node.latest_display_frame is not None:
                cv.imshow("YOLO 3D Grounding", node.latest_display_frame)

            key = cv.waitKey(1)
            if key == 27 or key == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
