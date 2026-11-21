#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rcl_interfaces.msg import SetParametersResult
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Int16

import message_filters
import numpy as np
import time
from ultralytics import YOLO

import tf2_ros
import tf2_geometry_msgs

class DynamicPID:
    def __init__(self, kp, ki, kd, max_out=1.0):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.max_out = float(max_out)
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = time.time()

    def compute(self, target, current):
        now = time.time()
        dt = max(0.01, now - self.last_time)
        error = target - current

        self.integral += error * dt
        self.integral = float(np.clip(self.integral, -self.max_out, self.max_out))

        derivative = (error - self.prev_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        self.prev_error = error
        self.last_time = now
        return float(np.clip(output, -self.max_out, self.max_out))

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class YoloTrackVoicePIDNode(Node):
    def __init__(self):
        super().__init__('yolo_track_voice_pid')
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")

        # Dynamic Parameters
        self.declare_parameter('linear_Kp', 0.85)
        self.declare_parameter('linear_Ki', 0.0)
        self.declare_parameter('linear_Kd', 0.05)
        self.declare_parameter('angular_Kp', 1.30)
        self.declare_parameter('angular_Ki', 0.0)
        self.declare_parameter('angular_Kd', 0.05)
        self.declare_parameter('minDistance', 0.8)
        self.declare_parameter('max_linear_speed', 0.45)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_tracker')

        lin_kp = self.get_parameter('linear_Kp').value
        lin_ki = self.get_parameter('linear_Ki').value
        lin_kd = self.get_parameter('linear_Kd').value
        max_lin = self.get_parameter('max_linear_speed').value

        ang_kp = self.get_parameter('angular_Kp').value
        ang_ki = self.get_parameter('angular_Ki').value
        ang_kd = self.get_parameter('angular_Kd').value
        max_ang = self.get_parameter('max_angular_speed').value

        self.target_dist = float(self.get_parameter('minDistance').value)
        self.linear_pid = DynamicPID(lin_kp, lin_ki, lin_kd, max_lin)
        self.angular_pid = DynamicPID(ang_kp, ang_ki, ang_kd, max_ang)

        self.add_on_set_parameters_callback(self.parameter_update_callback)

        # Boot into IDLE state until speech command arrives
        self.tracked_target_id = -1
        self.last_tracked_id = -1
        self.resume_requested = False
        self.target_name_lock = {}
        self.image_center_x = 320.0

        self.fx = 500.0
        self.fy = 500.0
        self.cx = 320.0
        self.cy = 240.0
        self.target_frame = "map"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/detected_objects_markers', 10)

        self.create_subscription(Int16, '/tracker_id', self.tracker_id_callback, 10)
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

        self.latest_rgb = None
        self.latest_depth = None
        self.latest_display = None
        self.last_infer = 0.0

        self.get_logger().info(f"Voice Tracking Mode Active (Waiting for voice command). Topic: {cmd_topic}")

    def parameter_update_callback(self, params):
        for param in params:
            if param.name == 'linear_Kp':
                self.linear_pid.kp = float(param.value)
            elif param.name == 'linear_Ki':
                self.linear_pid.ki = float(param.value)
            elif param.name == 'linear_Kd':
                self.linear_pid.kd = float(param.value)
            elif param.name == 'angular_Kp':
                self.angular_pid.kp = float(param.value)
            elif param.name == 'angular_Ki':
                self.angular_pid.ki = float(param.value)
            elif param.name == 'angular_Kd':
                self.angular_pid.kd = float(param.value)
            elif param.name == 'minDistance':
                self.target_dist = float(param.value)
            elif param.name == 'max_linear_speed':
                self.linear_pid.max_out = float(param.value)
            elif param.name == 'max_angular_speed':
                self.angular_pid.max_out = float(param.value)
        return SetParametersResult(successful=True)

    def tracker_id_callback(self, msg):
        req_id = msg.data
        self.angular_pid.reset()
        self.linear_pid.reset()

        if req_id == -1:
            self.tracked_target_id = -1
            self.resume_requested = False
            self.cmd_pub.publish(Twist())
            self.get_logger().info("STOPPED (IDLE). Motors halted.")
        elif req_id == -2:
            self.resume_requested = True
            self.get_logger().info("RESUME / AUTO-LOCK received.")
        else:
            self.tracked_target_id = req_id
            self.last_tracked_id = req_id
            self.resume_requested = False
            self.get_logger().info(f"Switched to Target ID: {self.tracked_target_id}")

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.cx = msg.k[2]
        self.fy = msg.k[4]
        self.cy = msg.k[5]
        self.image_center_x = self.cx

    def sync_callback(self, rgb_msg, depth_msg):
        self.latest_rgb = rgb_msg
        self.latest_depth = depth_msg

    def process_frame(self):
        now = time.time()
        if (now - self.last_infer) < 0.04 or self.latest_rgb is None:
            return
        self.last_infer = now

        rgb_msg = self.latest_rgb
        depth_msg = self.latest_depth

        try:
            rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            depth_img = np.nan_to_num(depth_img, nan=0.0)

            results = self.model.track(
                rgb_img,
                persist=True,
                tracker="bytetrack.yaml",
                conf=0.45,
                verbose=False
            )

            marker_array = MarkerArray()
            twist = Twist()
            target_found = False

            cam_frame = depth_msg.header.frame_id if depth_msg.header.frame_id else "ascamera_hp60c_camera_frame"
            has_map_tf = self.tf_buffer.can_transform(self.target_frame, cam_frame, Time())

            if results[0].boxes is not None and results[0].boxes.id is not None:
                boxes = results[0].boxes.xyxy.cpu().numpy()
                track_ids = results[0].boxes.id.int().cpu().numpy()
                clss = results[0].boxes.cls.int().cpu().numpy()

                # Smart Resume logic
                if self.resume_requested and len(track_ids) > 0:
                    self.resume_requested = False
                    if self.last_tracked_id in track_ids:
                        self.tracked_target_id = self.last_tracked_id
                    else:
                        centers_x = [(b[0] + b[2]) / 2.0 for b in boxes]
                        closest_idx = int(np.argmin(np.abs(np.array(centers_x) - self.image_center_x)))
                        self.tracked_target_id = int(track_ids[closest_idx])
                        self.last_tracked_id = self.tracked_target_id

                for box, track_id, cls_id in zip(boxes, track_ids, clss):
                    x1, y1, x2, y2 = map(int, box)
                    tid = int(track_id)

                    if tid not in self.target_name_lock:
                        self.target_name_lock[tid] = self.model.names[int(cls_id)]
                    label_name = self.target_name_lock[tid]

                    u = int((x1 + x2) / 2)
                    v = int((y1 + y2) / 2)

                    # Inner 40% ROI depth extraction
                    box_w = x2 - x1
                    box_h = y2 - y1
                    roi_x1 = max(0, min(depth_img.shape[1] - 1, int(x1 + 0.3 * box_w)))
                    roi_x2 = max(roi_x1 + 1, min(depth_img.shape[1], int(x2 - 0.3 * box_w)))
                    roi_y1 = max(0, min(depth_img.shape[0] - 1, int(y1 + 0.3 * box_h)))
                    roi_y2 = max(roi_y1 + 1, min(depth_img.shape[0], int(y2 - 0.3 * box_h)))

                    inner_region = depth_img[roi_y1:roi_y2, roi_x1:roi_x2]
                    if depth_img.dtype == np.uint16:
                        valid = inner_region[(inner_region > 200) & (inner_region < 6000)]
                    else:
                        valid = inner_region[(inner_region > 0.2) & (inner_region < 6.0)]

                    z_m = 0.0
                    if len(valid) > 5:
                        raw_z = np.percentile(valid, 20)
                        z_m = float(raw_z / 1000.0) if depth_img.dtype == np.uint16 else float(raw_z)

                    is_active = (self.tracked_target_id != -1 and tid == self.tracked_target_id)
                    box_color = (255, 120, 0) if is_active else (0, 255, 0)
                    cv.rectangle(rgb_img, (x1, y1), (x2, y2), box_color, 2)
                    status_str = " [TRACKING]" if is_active else ""
                    cv.putText(rgb_img, f"ID:{tid} {label_name} ({z_m:.2f}m){status_str}",
                               (x1, max(20, y1 - 8)), cv.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)

                    # 3D TF Projection
                    if 0.3 < z_m < 5.0 and has_map_tf:
                        x_c = (u - self.cx) * z_m / self.fx
                        y_c = (v - self.cy) * z_m / self.fy
                        z_c = z_m

                        try:
                            pt = PointStamped()
                            pt.header.stamp = depth_msg.header.stamp
                            pt.header.frame_id = cam_frame
                            pt.point.x, pt.point.y, pt.point.z = x_c, y_c, z_c
                            pt_map = self.tf_buffer.transform(pt, self.target_frame)
                            self.add_rviz_markers(marker_array, tid, label_name,
                                                  pt_map.point.x, pt_map.point.y, pt_map.point.z, is_active)
                        except Exception:
                            pass

                    # Closed-loop PID
                    if is_active and 0.30 < z_m < 4.0:
                        target_found = True
                        self.last_tracked_id = tid

                        norm_error_x = (self.image_center_x - u) / self.image_center_x
                        if abs(norm_error_x) > 0.05:
                            ang_out = self.angular_pid.compute(0.0, -norm_error_x)
                            if 0.0 < abs(ang_out) < 0.18:
                                ang_out = float(np.sign(ang_out) * 0.18)
                            twist.angular.z = ang_out
                        else:
                            twist.angular.z = 0.0

                        error_dist = z_m - self.target_dist
                        if abs(error_dist) > 0.06:
                            lin_out = self.linear_pid.compute(z_m, self.target_dist)
                            if 0.0 < abs(lin_out) < 0.12:
                                lin_out = float(np.sign(lin_out) * 0.12)
                            twist.linear.x = lin_out
                        else:
                            twist.linear.x = 0.0

            if len(marker_array.markers) > 0:
                self.marker_pub.publish(marker_array)

            if target_found:
                self.cmd_pub.publish(twist)
            else:
                self.cmd_pub.publish(Twist())

            if self.tracked_target_id == -1:
                state_label = "State: IDLE (Waiting Voice)"
            elif target_found:
                state_label = f"Voice Target: {self.tracked_target_id}"
            else:
                state_label = f"Searching for ID: {self.tracked_target_id}"

            osd_text = f"{state_label} | CmdX:{twist.linear.x:.2f} | CmdZ:{twist.angular.z:.2f}"
            cv.putText(rgb_img, osd_text, (10, 25), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            self.latest_display = rgb_img

        except Exception as e:
            self.get_logger().error(f"Execution error: {e}")

    def add_rviz_markers(self, marker_array, tid, name, x, y, z, is_target):
        m = Marker()
        m.header.frame_id = self.target_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = "yolo_shapes", tid * 2
        m.type, m.action = Marker.CYLINDER, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.scale.x = m.scale.y = 0.25
        m.scale.z = 0.4
        m.color.r, m.color.g, m.color.b, m.color.a = (0.0, 0.4, 1.0, 0.85) if is_target else (0.0, 1.0, 0.2, 0.65)
        m.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
        marker_array.markers.append(m)

        t = Marker()
        t.header.frame_id = self.target_frame
        t.header.stamp = self.get_clock().now().to_msg()
        t.ns, t.id = "yolo_labels", (tid * 2) + 1
        t.type, t.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        t.pose.position.x, t.pose.position.y, t.pose.position.z = x, y, z + 0.35
        t.scale.z = 0.15
        t.text = f"[ID:{tid}] {name}" + (" (TRACKING)" if is_target else "")
        t.color.r, t.color.g, t.color.b, t.color.a = 1.0, 1.0, 1.0, 1.0
        t.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
        marker_array.markers.append(t)

def main(args=None):
    rclpy.init(args=args)
    node = YoloTrackVoicePIDNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)
            node.process_frame()
            if node.latest_display is not None:
                cv.imshow("YOLO Tracking (Voice Mode)", node.latest_display)
            key = cv.waitKey(1)
            if key == 27 or key == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        cv.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
