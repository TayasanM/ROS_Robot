#!/usr/bin/env python3
"""
Unified Fuzzy-DWA Costmap Navigator & Visual Tracking Arbiter
-------------------------------------------------------------
1. During Navigation (Go to Room / Kitchen / Home):
   Modulates Nav2 DWA velocity (/cmd_vel_dwa) using Takagi-Sugeno Fuzzy Logic
   over /map, /global_costmap/costmap, /local_costmap/costmap, and /local_plan.
2. During Object Search & YOLO Tracking:
   Routes /cmd_vel_tracker to /cmd_vel while still enforcing Fuzzy Local Costmap
   speed scaling and Mecanum lateral obstacle repulsion.
3. Logs all trajectories, costmap risks, and velocities to CSV.
"""

import os
import csv
import math
from datetime import datetime
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, OccupancyGrid
from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)


def trapmf(x: float, a: float, b: float, c: float, d: float) -> float:
    if x <= a or x >= d:
        return 0.0
    elif b <= x <= c:
        return 1.0
    elif a < x < b:
        return (x - a) / (b - a + 1e-6)
    else:
        return (d - x) / (d - c + 1e-6)


def trimf(x: float, a: float, b: float, c: float) -> float:
    return max(0.0, min((x - a) / (b - a + 1e-6), (c - x) / (c - b + 1e-6)))


class FuzzyDWACostmapTracker(Node):
    def __init__(self):
        super().__init__('fuzzy_tracker')

        # Declare and read YAML parameters
        self.declare_parameter('max_vx', 0.26)
        self.declare_parameter('max_vy', 0.20)
        self.declare_parameter('max_wz', 0.80)
        self.declare_parameter('risk_low_end', 55.0)
        self.declare_parameter('risk_med_peak', 60.0)
        self.declare_parameter('risk_high_start', 65.0)
        self.declare_parameter('scale_low_risk', 1.0)
        self.declare_parameter('scale_med_risk', 0.55)
        self.declare_parameter('scale_high_risk', 0.15)
        self.declare_parameter('side_cost_start', 35.0)
        self.declare_parameter('side_cost_full', 75.0)

        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_wz = float(self.get_parameter('max_wz').value)
        self.risk_low_end = float(self.get_parameter('risk_low_end').value)
        self.risk_med_peak = float(self.get_parameter('risk_med_peak').value)
        self.risk_high_start = float(self.get_parameter('risk_high_start').value)
        self.scale_low_risk = float(self.get_parameter('scale_low_risk').value)
        self.scale_med_risk = float(self.get_parameter('scale_med_risk').value)
        self.scale_high_risk = float(self.get_parameter('scale_high_risk').value)
        self.side_cost_start = float(self.get_parameter('side_cost_start').value)
        self.side_cost_full = float(self.get_parameter('side_cost_full').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Costmaps & Plans
        self.static_map = None
        self.global_costmap = None
        self.local_costmap = None
        self.global_plan = []
        self.local_plan = []

        # Velocity Inputs: DWA Navigation vs. YOLO Tracker
        self.dwa_cmd = Twist()
        self.tracker_cmd = Twist()
        self.last_dwa_time = 0.0
        self.last_tracker_time = 0.0
        self.timeout_threshold = 0.45
        self.velocity_deadband = 0.015
        self.step_count = 0

        # Setup CSV Logging
        log_dir = os.path.expanduser('~/yahboomcar_ros2_ws/fuzzy_logs')
        os.makedirs(log_dir, exist_ok=True)
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(log_dir, f'fuzzy_mission_{timestamp_str}.csv')

        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time_sec', 'active_source', 'rx', 'ry', 'ryaw',
            'pred_path_cost', 'front_cost', 'left_cost', 'right_cost', 'global_cost',
            'in_vx', 'in_vy', 'in_wz',
            'fuzzy_vx', 'fuzzy_vy', 'fuzzy_wz'
        ])
        self.start_time = self.get_clock().now().nanoseconds * 1e-9

        map_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, map_qos)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.global_cm_cb, 10)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.local_cm_cb, 10)
        self.create_subscription(Path, '/plan', self.global_plan_cb, 10)
        self.create_subscription(Path, '/local_plan', self.local_plan_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_dwa', self.dwa_cmd_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_tracker', self.tracker_cmd_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f'Fuzzy-DWA Costmap Mission Navigator Ready! CSV: {self.csv_path}')

    def map_cb(self, msg: OccupancyGrid):
        self.static_map = msg

    def global_cm_cb(self, msg: OccupancyGrid):
        self.global_costmap = msg

    def local_cm_cb(self, msg: OccupancyGrid):
        self.local_costmap = msg

    def global_plan_cb(self, msg: Path):
        self.global_plan = msg.poses

    def local_plan_cb(self, msg: Path):
        self.local_plan = msg.poses

    def dwa_cmd_cb(self, msg: Twist):
        self.dwa_cmd = msg
        self.last_dwa_time = self.get_clock().now().nanoseconds * 1e-9

    def tracker_cmd_cb(self, msg: Twist):
        self.tracker_cmd = msg
        self.last_tracker_time = self.get_clock().now().nanoseconds * 1e-9

    def is_cmd_active(self, last_time: float, cmd: Twist, now_sec: float) -> bool:
        if (now_sec - last_time) > self.timeout_threshold:
            return False
        return (abs(cmd.linear.x) > self.velocity_deadband or
                abs(cmd.linear.y) > self.velocity_deadband or
                abs(cmd.angular.z) > self.velocity_deadband)

    def get_pose_in_frame(self, target_frame: str = 'map'):
        try:
            t = self.tf_buffer.lookup_transform(target_frame, 'base_footprint', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            return x, y, yaw
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def sample_grid_cost(self, grid: OccupancyGrid, wx: float, wy: float) -> float:
        if grid is None:
            return 0.0
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        gx = int((wx - ox) / res)
        gy = int((wy - oy) / res)

        if 0 <= gx < grid.info.width and 0 <= gy < grid.info.height:
            val = grid.data[gy * grid.info.width + gx]
            return float(max(0, val))
        return 0.0

    def evaluate_costmap_sectors(self):
        if self.local_costmap is None:
            return 0.0, 0.0, 0.0, 0.0

        local_frame = self.local_costmap.header.frame_id
        pose = self.get_pose_in_frame(local_frame)
        if pose is None:
            return 0.0, 0.0, 0.0, 0.0
        rx, ry, ryaw = pose

        front_costs, left_costs, right_costs = [0.0], [0.0], [0.0]
        for dist in [0.18, 0.26, 0.35, 0.45]:
            for ang in [-0.25, 0.0, 0.25]:
                wx = rx + dist * math.cos(ryaw + ang)
                wy = ry + dist * math.sin(ryaw + ang)
                front_costs.append(self.sample_grid_cost(self.local_costmap, wx, wy))

            for ang in [0.55, 0.90, 1.25]:
                wx = rx + dist * math.cos(ryaw + ang)
                wy = ry + dist * math.sin(ryaw + ang)
                left_costs.append(self.sample_grid_cost(self.local_costmap, wx, wy))

            for ang in [-0.55, -0.90, -1.25]:
                wx = rx + dist * math.cos(ryaw + ang)
                wy = ry + dist * math.sin(ryaw + ang)
                right_costs.append(self.sample_grid_cost(self.local_costmap, wx, wy))

        pred_costs = [0.0]
        for p in self.local_plan:
            pred_costs.append(
                self.sample_grid_cost(self.local_costmap, p.pose.position.x, p.pose.position.y)
            )

        return max(pred_costs), max(front_costs), max(left_costs), max(right_costs)

    def control_loop(self):
        now_abs = self.get_clock().now().nanoseconds * 1e-9

        # Select active source: YOLO Search/Tracking or Fuzzy-DWA Navigation
        tracker_active = self.is_cmd_active(self.last_tracker_time, self.tracker_cmd, now_abs)
        dwa_active = self.is_cmd_active(self.last_dwa_time, self.dwa_cmd, now_abs)

        if tracker_active:
            active_source = "YOLO_TRACKER"
            raw_cmd = self.tracker_cmd
        elif dwa_active:
            active_source = "FUZZY_DWA_NAV"
            raw_cmd = self.dwa_cmd
        else:
            return

        map_pose = self.get_pose_in_frame('map') or self.get_pose_in_frame('odom')
        if map_pose is None:
            return
        rx, ry, ryaw = map_pose

        pred_cost, front_cost, left_cost, right_cost = self.evaluate_costmap_sectors()
        global_cost = self.sample_grid_cost(self.global_costmap, rx, ry)
        static_map_cost = self.sample_grid_cost(self.static_map, rx, ry)

        # During DWA Navigation, include predicted trajectory cost; during YOLO tracking, check front/global cost
        if active_source == "FUZZY_DWA_NAV":
            obstacle_risk = max(pred_cost, front_cost, global_cost, static_map_cost)
        else:
            obstacle_risk = max(front_cost, global_cost)

        # 1. Fuzzify Obstacle Risk
        risk_low  = trapmf(obstacle_risk, 0.0, 0.0, 25.0, self.risk_low_end)
        risk_med  = trimf(obstacle_risk, 30.0, self.risk_med_peak, 85.0)
        risk_high = trapmf(obstacle_risk, self.risk_high_start, 88.0, 100.0, 100.0)

        # 2. Takagi-Sugeno Forward Speed Scaling
        denom = risk_low + risk_med + risk_high + 1e-6
        speed_scale = (
            risk_low  * self.scale_low_risk +
            risk_med  * self.scale_med_risk +
            risk_high * self.scale_high_risk
        ) / denom

        # Scale forward velocity (allow backing up if YOLO PID steps back from close target)
        if raw_cmd.linear.x > 0.0:
            fuzzy_vx = float(np.clip(raw_cmd.linear.x * speed_scale, 0.0, self.max_vx))
        else:
            fuzzy_vx = float(np.clip(raw_cmd.linear.x, -self.max_vx, 0.0))

        # 3. Fuzzy Mecanum Lateral Obstacle Avoidance from Local Costmap
        l_high = trapmf(left_cost, self.side_cost_start, self.side_cost_full, 100.0, 100.0)
        r_high = trapmf(right_cost, self.side_cost_start, self.side_cost_full, 100.0, 100.0)
        vy_repulse = (r_high - l_high) * self.max_vy

        if active_source == "FUZZY_DWA_NAV" and risk_high > 0.3:
            escape_dir = 1.0 if left_cost < right_cost else -1.0
            vy_repulse += risk_high * escape_dir * (0.8 * self.max_vy)

        fuzzy_vy = float(np.clip(raw_cmd.linear.y + vy_repulse, -self.max_vy, self.max_vy))
        fuzzy_wz = float(np.clip(raw_cmd.angular.z, -self.max_wz, self.max_wz))

        out_cmd = Twist()
        out_cmd.linear.x = fuzzy_vx
        out_cmd.linear.y = fuzzy_vy
        out_cmd.angular.z = fuzzy_wz
        self.cmd_pub.publish(out_cmd)

        # Log to CSV
        rel_sec = now_abs - self.start_time
        self.csv_writer.writerow([
            f'{rel_sec:.3f}', active_source, f'{rx:.4f}', f'{ry:.4f}', f'{ryaw:.4f}',
            f'{pred_cost:.1f}', f'{front_cost:.1f}', f'{left_cost:.1f}',
            f'{right_cost:.1f}', f'{global_cost:.1f}',
            f'{raw_cmd.linear.x:.4f}', f'{raw_cmd.linear.y:.4f}', f'{raw_cmd.angular.z:.4f}',
            f'{fuzzy_vx:.4f}', f'{fuzzy_vy:.4f}', f'{fuzzy_wz:.4f}'
        ])
        self.csv_file.flush()

    def destroy_node(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FuzzyDWACostmapTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
