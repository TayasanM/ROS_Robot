#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import time
import yaml
import cv2 as cv
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from nav2_msgs.action import NavigateThroughPoses
import tf2_ros


class YoloMissionInspectorNode(Node):
    def __init__(self):
        super().__init__('yolo_tracker_node')
        self.bridge = CvBridge()
        self.model = None  # Loaded only when robot reaches the room

        self.declare_parameter('arrival_tolerance', 0.25)
        self.declare_parameter('wait_before_scan_sec', 60.0)
        self.declare_parameter('search_spin_speed', 0.25)
        self.declare_parameter('search_duration_sec', 26.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('default_target_object', 'bottle')
        self.declare_parameter('breadcrumb_spacing', 0.45)

        self.arrival_tol = float(self.get_parameter('arrival_tolerance').value)
        self.wait_duration = float(self.get_parameter('wait_before_scan_sec').value)
        self.search_spin = float(self.get_parameter('search_spin_speed').value)
        self.search_duration = float(self.get_parameter('search_duration_sec').value)
        self.default_target = str(self.get_parameter('default_target_object').value)
        self.breadcrumb_spacing = float(self.get_parameter('breadcrumb_spacing').value)

        self.places = self.load_map_mapping()

        # States: IDLE -> NAV_TO_ZONE -> WAITING -> SEARCHING -> NAV_HOME -> IDLE
        self.state = 'IDLE'
        self.target_zone = None
        self.target_object = self.default_target
        self.return_pose = None
        self.recorded_route = []  # Breadcrumbs recorded on the way to the room
        self.nav_start_time = 0.0
        self.wait_start_time = 0.0
        self.search_start_time = 0.0
        self.seen_objects = set()
        self.target_found = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.tts_pub = self.create_publisher(String, '/speech_tts', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/detected_objects_markers', 10)
        self.nav_through_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        self.create_subscription(String, '/voice_mission_cmd', self.mission_cmd_cb, 10)
        self.create_subscription(
            Image, '/ascamera_hp60c/camera_publisher/rgb0/image',
            self.rgb_cb, qos_profile=qos_profile_sensor_data
        )

        self.latest_rgb = None
        self.latest_display = None  # Holds live scan OR frozen detection frame
        self.window_open = False
        self.last_infer = 0.0

        self.create_timer(1.0, self.publish_zone_markers)
        self.get_logger().info(f"DWA + YOLO Mission Node Ready. Places: {list(self.places.keys())}")

    def load_map_mapping(self):
        candidates = [
            os.path.join(get_package_share_directory('yahboomcar_fuzzy_nav'), 'params', 'map_mapping.yaml'),
            os.path.expanduser('~/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_fuzzy_nav/params/map_mapping.yaml')
        ]
        for path in candidates:
            if os.path.exists(path):
                with open(path, 'r') as f:
                    return yaml.safe_load(f) or {}
        self.get_logger().error("map_mapping.yaml not found!")
        return {}

    def get_current_pose_dict(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
            return {
                'position': {'x': t.transform.translation.x, 'y': t.transform.translation.y, 'z': 0.0},
                'orientation': {
                    'x': t.transform.rotation.x, 'y': t.transform.rotation.y,
                    'z': t.transform.rotation.z, 'w': t.transform.rotation.w
                }
            }
        except Exception:
            return self.places.get('charging_position', {
                'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
            })

    def dict_to_pose_stamped(self, pose_dict):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(pose_dict['position']['x'])
        pose.pose.position.y = float(pose_dict['position']['y'])
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = float(pose_dict['orientation'].get('x', 0.0))
        pose.pose.orientation.y = float(pose_dict['orientation'].get('y', 0.0))
        pose.pose.orientation.z = float(pose_dict['orientation'].get('z', 0.0))
        pose.pose.orientation.w = float(pose_dict['orientation'].get('w', 1.0))
        return pose

    def send_goal_from_dict(self, pose_dict):
        self.goal_pub.publish(self.dict_to_pose_stamped(pose_dict))
        self.nav_start_time = time.time()

    def record_route_breadcrumb(self):
        """Records (x, y) waypoints while driving to the room so we can retrace them."""
        curr = self.get_current_pose_dict()
        cx, cy = float(curr['position']['x']), float(curr['position']['y'])
        if not self.recorded_route:
            self.recorded_route.append(curr)
            return
        last = self.recorded_route[-1]
        lx, ly = float(last['position']['x']), float(last['position']['y'])
        if math.hypot(cx - lx, cy - ly) >= self.breadcrumb_spacing:
            self.recorded_route.append(curr)

    def send_reversed_route_home(self):
        """Sends the reversed outbound breadcrumbs to Nav2 NavigateThroughPoses."""
        reversed_poses = [self.dict_to_pose_stamped(p) for p in reversed(self.recorded_route)]
        # Ensure the exact starting pose is the final destination
        reversed_poses.append(self.dict_to_pose_stamped(self.return_pose))

        if self.nav_through_client.wait_for_server(timeout_sec=2.0):
            goal_msg = NavigateThroughPoses.Goal()
            goal_msg.poses = reversed_poses
            self.nav_through_client.send_goal_async(goal_msg)
            self.nav_start_time = time.time()
            self.get_logger().info(
                f"[MISSION] Retracing exact same route home ({len(reversed_poses)} waypoints)."
            )
        else:
            self.get_logger().warn("NavigateThroughPoses not available, falling back to direct /goal_pose.")
            self.send_goal_from_dict(self.return_pose)

    def publish_zone_markers(self):
        ma = MarkerArray()
        for idx, (zname, pdata) in enumerate(self.places.items()):
            px, py = float(pdata['position']['x']), float(pdata['position']['y'])
            is_active = (self.target_zone == zname)

            disc = Marker()
            disc.header.frame_id = 'map'
            disc.header.stamp = self.get_clock().now().to_msg()
            disc.ns, disc.id = 'map_zones', idx * 2
            disc.type, disc.action = Marker.CYLINDER, Marker.ADD
            disc.pose.position.x, disc.pose.position.y, disc.pose.position.z = px, py, 0.02
            disc.scale.x = disc.scale.y = 0.40 if is_active else 0.30
            disc.scale.z = 0.04
            disc.color.r, disc.color.g, disc.color.b, disc.color.a = (
                (1.0, 0.65, 0.0, 0.85) if is_active else (0.1, 0.7, 1.0, 0.75)
            )
            ma.markers.append(disc)

            txt = Marker()
            txt.header.frame_id = 'map'
            txt.header.stamp = self.get_clock().now().to_msg()
            txt.ns, txt.id = 'map_zone_labels', idx * 2 + 1
            txt.type, txt.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = px, py, 0.30
            txt.scale.z = 0.20
            txt.text = f"[{zname.upper()}]"
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            ma.markers.append(txt)

        if ma.markers:
            self.marker_pub.publish(ma)

    def mission_cmd_cb(self, msg: String):
        cmd = msg.data.strip()

        if cmd == 'STOP':
            self.state = 'IDLE'
            self.target_zone = None
            self.cmd_pub.publish(Twist())
            return

        if cmd == 'RETURN_HOME':
            self.return_pose = self.places.get('charging_position', self.get_current_pose_dict())
            self.target_zone = 'charging_position'
            self.state = 'NAV_HOME'
            self.send_goal_from_dict(self.return_pose)
            return

        if cmd.startswith('MISSION:'):
            _, zone_name, obj_class = cmd.split(':')
            if zone_name not in self.places:
                self.tts_pub.publish(String(data=f"Location {zone_name} is not saved in map mapping."))
                return

            self.return_pose = self.get_current_pose_dict()
            self.recorded_route = [self.return_pose]
            self.target_zone = zone_name
            self.target_object = self.default_target if obj_class == 'any' else obj_class
            self.seen_objects.clear()
            self.target_found = False
            self.state = 'NAV_TO_ZONE'
            self.send_goal_from_dict(self.places[zone_name])
            self.get_logger().info(f"[MISSION] DWA Navigating to {zone_name} to find '{self.target_object}'")

    def rgb_cb(self, msg: Image):
        self.latest_rgb = msg

    def check_arrival(self, target_pose_dict):
        if (time.time() - self.nav_start_time) < 3.0:
            return False
        curr = self.get_current_pose_dict()
        dx = float(target_pose_dict['position']['x']) - float(curr['position']['x'])
        dy = float(target_pose_dict['position']['y']) - float(curr['position']['y'])
        return math.hypot(dx, dy) <= self.arrival_tol

    def start_return_home(self):
        # Stop spinning and stop YOLO inference, but DO NOT close the OpenCV window!
        self.cmd_pub.publish(Twist())
        self.state = 'NAV_HOME'
        self.send_reversed_route_home()

    def speak_final_report(self):
        if self.target_found:
            msg = f"Mission complete. I found the {self.target_object} in {self.target_zone}."
        else:
            found_list = ", ".join(sorted(self.seen_objects)) if self.seen_objects else "nothing"
            msg = (
                f"Mission complete. I did not find the {self.target_object} in {self.target_zone}. "
                f"Instead I saw {found_list}."
            )
        self.tts_pub.publish(String(data=msg))
        self.get_logger().info(f"[REPORT] {msg}")

    def process_step(self):
        # 1. Record route breadcrumbs while driving to the room
        if self.state == 'NAV_TO_ZONE' and self.target_zone in self.places:
            self.record_route_breadcrumb()
            if self.check_arrival(self.places[self.target_zone]):
                self.state = 'WAITING'
                self.wait_start_time = time.time()
                self.tts_pub.publish(
                    String(data=f"Arrived at {self.target_zone}. Waiting one minute before starting detection.")
                )
                self.get_logger().info(f"[MISSION] Reached {self.target_zone}. Waiting {self.wait_duration}s...")
            return

        # 2. Wait 1 Minute at Exact Point -> Open YOLO & Start Slow Scan
        if self.state == 'WAITING':
            if (time.time() - self.wait_start_time) >= self.wait_duration:
                if self.model is None:
                    self.get_logger().info("Opening YOLO object detection model...")
                    self.model = YOLO("yolov8n.pt")
                self.state = 'SEARCHING'
                self.search_start_time = time.time()
                self.tts_pub.publish(String(data=f"Starting scan for {self.target_object}."))
                self.get_logger().info(f"[MISSION] Wait complete. Scanning slowly for '{self.target_object}'.")
            return

        # 3. Check Arrival Back Home -> Speak Report (YOLO stays stopped, window stays open)
        if self.state == 'NAV_HOME' and self.return_pose is not None:
            if self.check_arrival(self.return_pose):
                self.state = 'IDLE'
                if self.target_zone and self.target_zone != 'charging_position':
                    self.speak_final_report()
                else:
                    self.tts_pub.publish(String(data="Arrived at charging position."))
                self.target_zone = None
            return

        # 4. Run YOLO ONLY in SEARCHING state (stops immediately once bottle is found)
        if self.state != 'SEARCHING':
            return

        now = time.time()
        if (now - self.last_infer) < 0.05 or self.latest_rgb is None:
            return
        self.last_infer = now

        try:
            rgb_img = self.bridge.imgmsg_to_cv2(self.latest_rgb, desired_encoding='bgr8')
            results = self.model(rgb_img, conf=0.45, verbose=False)

            if results[0].boxes is not None:
                for box in results[0].boxes:
                    cls_name = self.model.names[int(box.cls[0])]
                    x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                    color = (0, 165, 255) if cls_name == self.target_object else (0, 255, 0)
                    cv.rectangle(rgb_img, (x1, y1), (x2, y2), color, 2)
                    cv.putText(rgb_img, cls_name, (x1, max(20, y1 - 8)),
                               cv.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                    self.seen_objects.add(cls_name)
                    if cls_name == self.target_object:
                        self.target_found = True

            elapsed_scan = time.time() - self.search_start_time

            if self.target_found:
                # Freeze this exact frame in the window and stop running YOLO
                cv.rectangle(rgb_img, (0, 0), (rgb_img.shape[1], 45), (0, 140, 0), -1)
                cv.putText(
                    rgb_img, f"{self.target_object.upper()} DETECTED! YOLO STOPPED - RETURNING HOME",
                    (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2
                )
                self.latest_display = rgb_img
                self.start_return_home()
            elif elapsed_scan >= self.search_duration:
                cv.putText(
                    rgb_img, f"{self.target_object.upper()} NOT FOUND - RETURNING HOME",
                    (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2
                )
                self.latest_display = rgb_img
                self.start_return_home()
            else:
                spin = Twist()
                spin.angular.z = self.search_spin
                self.cmd_pub.publish(spin)
                osd = f"SCANNING ({elapsed_scan:.1f}s) | Target:{self.target_object}"
                cv.putText(rgb_img, osd, (10, 25), cv.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
                self.latest_display = rgb_img

        except Exception as e:
            self.get_logger().error(f"YOLO error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloMissionInspectorNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)
            node.process_step()
            # Keep displaying the live or frozen detection frame without closing the window
            if node.latest_display is not None:
                cv.imshow("YOLO Bottle Detector", node.latest_display)
                node.window_open = True
            if node.window_open:
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
