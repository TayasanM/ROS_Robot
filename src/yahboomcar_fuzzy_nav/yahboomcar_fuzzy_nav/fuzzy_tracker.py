#!/usr/bin/env python3
"""
Standalone Fuzzy-DWA Costmap Navigator for Yahboom ROSMASTER M1 (Mecanum)
-------------------------------------------------------------------------
This node reads:
  1. Static map (/map)
  2. Global costmap (/global_costmap/costmap)
  3. Local costmap (/local_costmap/costmap)
  4. DWA's predicted local trajectory (/local_plan) and global path (/plan)
  5. DWA's raw velocity command (/cmd_vel_dwa)

It uses a Takagi-Sugeno Fuzzy Inference System (FIS) configured via ROS 2 YAML
parameters (fuzzy_nav_params.yaml) to:
  - Smoothly scale DWA's forward velocity (vx) when approaching high-cost cells.
  - Inject Mecanum lateral strafing velocity (vy) away from inflated obstacle zones.
  - Log all pose, costmap, DWA, and Fuzzy control signals to a timestamped CSV file.
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


# =====================================================================
# Fuzzy Membership Helper Functions
# =====================================================================
def trapmf(x: float, a: float, b: float, c: float, d: float) -> float:
    """
    Trapezoidal membership function.
    Returns degree of membership in [0.0, 1.0] for input x given vertices (a, b, c, d).
    """
    if x <= a or x >= d:
        return 0.0
    elif b <= x <= c:
        return 1.0
    elif a < x < b:
        return (x - a) / (b - a + 1e-6)
    else:
        return (d - x) / (d - c + 1e-6)


def trimf(x: float, a: float, b: float, c: float) -> float:
    """
    Triangular membership function.
    Returns degree of membership in [0.0, 1.0] for input x given vertices (a, b, c).
    """
    return max(0.0, min((x - a) / (b - a + 1e-6), (c - x) / (c - b + 1e-6)))


# =====================================================================
# Main Fuzzy-DWA Costmap Tracker Node
# =====================================================================
class FuzzyDWACostmapTracker(Node):
    def __init__(self):
        super().__init__('fuzzy_tracker')

        # -------------------------------------------------------------
        # 1. Declare and Load Parameters from fuzzy_nav_params.yaml
        # -------------------------------------------------------------
        # Velocity limits for the Yahboom M1 Mecanum chassis
        self.declare_parameter('max_vx', 0.26)          # Max forward/backward speed (m/s)
        self.declare_parameter('max_vy', 0.20)          # Max Mecanum sideways strafe speed (m/s)
        self.declare_parameter('max_wz', 0.80)          # Max angular rotation speed (rad/s)

        # Fuzzy Costmap Risk Thresholds (OccupancyGrid values range from 0 to 100)
        self.declare_parameter('risk_low_end', 55.0)    # Upper bound of Low Risk trapezoid
        self.declare_parameter('risk_med_peak', 60.0)   # Peak of Medium Risk triangle
        self.declare_parameter('risk_high_start', 65.0) # Lower bound of High Risk trapezoid

        # Takagi-Sugeno Consequent Singleton Outputs for Forward Speed Scaling
        self.declare_parameter('scale_low_risk', 1.0)   # 100% of DWA speed in free space
        self.declare_parameter('scale_med_risk', 0.55)  # 55% of DWA speed in medium inflation
        self.declare_parameter('scale_high_risk', 0.15) # 15% of DWA speed near obstacles

        # Lateral Mecanum Repulsion Thresholds (Left/Right flank costs)
        self.declare_parameter('side_cost_start', 35.0) # Cost where sideways push begins
        self.declare_parameter('side_cost_full', 75.0)  # Cost where sideways push reaches max

        # Retrieve parameter values into class attributes
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

        self.get_logger().info(
            f'Loaded YAML Params -> max_vx: {self.max_vx}, max_vy: {self.max_vy}, '
            f'risk_thresholds: [{self.risk_low_end}, {self.risk_med_peak}, {self.risk_high_start}]'
        )

        # -------------------------------------------------------------
        # 2. Initialize TF2 Buffer and State Variables
        # -------------------------------------------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.static_map = None
        self.global_costmap = None
        self.local_costmap = None
        self.global_plan = []
        self.local_plan = []
        self.dwa_cmd = Twist()
        self.last_dwa_time = self.get_clock().now()
        self.step_count = 0

        # -------------------------------------------------------------
        # 3. Setup Automatic CSV Data Logging
        # -------------------------------------------------------------
        log_dir = os.path.expanduser('~/yahboomcar_ros2_ws/fuzzy_logs')
        os.makedirs(log_dir, exist_ok=True)
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(log_dir, f'fuzzy_dwa_costmap_{timestamp_str}.csv')

        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # Write CSV Header
        self.csv_writer.writerow([
            'time_sec', 'rx', 'ry', 'ryaw',
            'pred_path_cost', 'front_cost', 'left_cost', 'right_cost', 'global_cost',
            'dwa_vx', 'dwa_vy', 'dwa_wz',
            'fuzzy_vx', 'fuzzy_vy', 'fuzzy_wz'
        ])
        self.start_time = self.get_clock().now().nanoseconds * 1e-9

        # -------------------------------------------------------------
        # 4. Configure ROS 2 Subscribers and Publisher
        # -------------------------------------------------------------
        # Transient Local QoS is required to receive the static /map from map_server
        map_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(OccupancyGrid, '/map', self.map_cb, map_qos)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.global_cm_cb, 10)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.local_cm_cb, 10)
        self.create_subscription(Path, '/plan', self.global_plan_cb, 10)
        self.create_subscription(Path, '/local_plan', self.local_plan_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_dwa', self.dwa_cmd_cb, 10)

        # Final modulated velocity publisher to the robot's base driver
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Run the fuzzy control loop at 20 Hz (every 0.05 seconds)
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f'Fuzzy-DWA Costmap Navigator Ready! Logging CSV to: {self.csv_path}')

    # =================================================================
    # Subscriber Callbacks
    # =================================================================
    def map_cb(self, msg: OccupancyGrid):
        """Stores the static occupancy grid map (/map)."""
        self.static_map = msg

    def global_cm_cb(self, msg: OccupancyGrid):
        """Stores the global costmap (/global_costmap/costmap)."""
        self.global_costmap = msg

    def local_cm_cb(self, msg: OccupancyGrid):
        """Stores the rolling local costmap (/local_costmap/costmap)."""
        self.local_costmap = msg

    def global_plan_cb(self, msg: Path):
        """Stores the global path (/plan) generated by Nav2."""
        self.global_plan = msg.poses

    def local_plan_cb(self, msg: Path):
        """Stores DWA's predicted local trajectory (/local_plan)."""
        self.local_plan = msg.poses

    def dwa_cmd_cb(self, msg: Twist):
        """Captures DWA's velocity output (/cmd_vel_dwa) and updates activity timestamp."""
        self.dwa_cmd = msg
        self.last_dwa_time = self.get_clock().now()

    # =================================================================
    # Coordinate & Costmap Helper Methods
    # =================================================================
    def get_pose_in_frame(self, target_frame: str = 'map'):
        """
        Looks up the robot's current pose (x, y, yaw) of base_footprint in target_frame.
        Returns None if the TF transform is not yet available.
        """
        try:
            t = self.tf_buffer.lookup_transform(target_frame, 'base_footprint', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            # Convert quaternion to Euler yaw angle (radians)
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            return x, y, yaw
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def sample_grid_cost(self, grid: OccupancyGrid, wx: float, wy: float) -> float:
        """
        Converts a world coordinate (wx, wy) into grid cell indices (gx, gy)
        and returns the cell's occupancy cost in [0.0, 100.0].
        """
        if grid is None:
            return 0.0

        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y

        gx = int((wx - ox) / res)
        gy = int((wy - oy) / res)

        # Ensure the sampled point lies inside the costmap boundaries
        if 0 <= gx < grid.info.width and 0 <= gy < grid.info.height:
            val = grid.data[gy * grid.info.width + gx]
            # Unknown cells (-1) are treated as 0 cost so they don't block motion
            return float(max(0, val))
        return 0.0

    def evaluate_costmap_sectors(self):
        """
        Samples the local_costmap in 3 radial sectors around the robot (Front, Left, Right)
        and along DWA's predicted trajectory (/local_plan).
        Returns: (pred_path_cost, front_cost, left_cost, right_cost)
        """
        if self.local_costmap is None:
            return 0.0, 0.0, 0.0, 0.0

        # Local costmap typically uses the 'odom' frame
        local_frame = self.local_costmap.header.frame_id
        pose = self.get_pose_in_frame(local_frame)
        if pose is None:
            return 0.0, 0.0, 0.0, 0.0
        rx, ry, ryaw = pose

        front_costs, left_costs, right_costs = [0.0], [0.0], [0.0]

        # Probe radial distances from 0.18m to 0.45m around the robot chassis
        for dist in [0.18, 0.26, 0.35, 0.45]:
            # Front cone: -14 deg to +14 deg
            for ang in [-0.25, 0.0, 0.25]:
                wx = rx + dist * math.cos(ryaw + ang)
                wy = ry + dist * math.sin(ryaw + ang)
                front_costs.append(self.sample_grid_cost(self.local_costmap, wx, wy))

            # Left flank: +31 deg to +71 deg (for Mecanum right-strafe repulsion)
            for ang in [0.55, 0.90, 1.25]:
                wx = rx + dist * math.cos(ryaw + ang)
                wy = ry + dist * math.sin(ryaw + ang)
                left_costs.append(self.sample_grid_cost(self.local_costmap, wx, wy))

            # Right flank: -31 deg to -71 deg (for Mecanum left-strafe repulsion)
            for ang in [-0.55, -0.90, -1.25]:
                wx = rx + dist * math.cos(ryaw + ang)
                wy = ry + dist * math.sin(ryaw + ang)
                right_costs.append(self.sample_grid_cost(self.local_costmap, wx, wy))

        # Sample cost along every pose of DWA's predicted trajectory (/local_plan)
        pred_costs = [0.0]
        for p in self.local_plan:
            pred_costs.append(
                self.sample_grid_cost(self.local_costmap, p.pose.position.x, p.pose.position.y)
            )

        return max(pred_costs), max(front_costs), max(left_costs), max(right_costs)

    # =================================================================
    # Main 20 Hz Fuzzy Inference & Control Loop
    # =================================================================
    def control_loop(self):
        # Only publish commands while DWA is actively sending velocity commands (< 0.5s old)
        dt_dwa = (self.get_clock().now() - self.last_dwa_time).nanoseconds * 1e-9
        if dt_dwa > 0.5:
            return

        # Get robot pose in 'map' frame (fallback to 'odom' if map TF is momentarily busy)
        map_pose = self.get_pose_in_frame('map') or self.get_pose_in_frame('odom')
        if map_pose is None:
            return
        rx, ry, ryaw = map_pose

        # 1. Extract Costmap Sector Risks
        pred_cost, front_cost, left_cost, right_cost = self.evaluate_costmap_sectors()
        global_cost = self.sample_grid_cost(self.global_costmap, rx, ry)
        static_map_cost = self.sample_grid_cost(self.static_map, rx, ry)

        # Combine predicted trajectory cost, front cost, and global/static map awareness
        obstacle_risk = max(pred_cost, front_cost, global_cost, static_map_cost)

        # 2. Fuzzify Forward Obstacle Risk into Low, Medium, and High membership degrees
        risk_low  = trapmf(obstacle_risk, 0.0, 0.0, 25.0, self.risk_low_end)
        risk_med  = trimf(obstacle_risk, 30.0, self.risk_med_peak, 85.0)
        risk_high = trapmf(obstacle_risk, self.risk_high_start, 88.0, 100.0, 100.0)

        # 3. Takagi-Sugeno Defuzzification for Forward Velocity Scaling
        denom = risk_low + risk_med + risk_high + 1e-6
        speed_scale = (
            risk_low  * self.scale_low_risk +
            risk_med  * self.scale_med_risk +
            risk_high * self.scale_high_risk
        ) / denom

        fuzzy_vx = float(np.clip(self.dwa_cmd.linear.x * speed_scale, -self.max_vx, self.max_vx))

        # 4. Fuzzy Mecanum Lateral Strafe (vy) from Left/Right Costmap Gradients
        l_high = trapmf(left_cost, self.side_cost_start, self.side_cost_full, 100.0, 100.0)
        r_high = trapmf(right_cost, self.side_cost_start, self.side_cost_full, 100.0, 100.0)

        # If left cost is high, push right (-vy); if right cost is high, push left (+vy)
        vy_repulse = (r_high - l_high) * self.max_vy

        # If the predicted path ahead enters a high-risk zone, strafe toward the clearer side
        if risk_high > 0.3:
            escape_dir = 1.0 if left_cost < right_cost else -1.0
            vy_repulse += risk_high * escape_dir * (0.8 * self.max_vy)

        fuzzy_vy = float(np.clip(self.dwa_cmd.linear.y + vy_repulse, -self.max_vy, self.max_vy))
        fuzzy_wz = float(np.clip(self.dwa_cmd.angular.z, -self.max_wz, self.max_wz))

        # 5. Publish Modulated Velocity Command to /cmd_vel
        out_cmd = Twist()
        out_cmd.linear.x = fuzzy_vx
        out_cmd.linear.y = fuzzy_vy
        out_cmd.angular.z = fuzzy_wz
        self.cmd_pub.publish(out_cmd)

        # 6. Periodic Terminal Diagnostics (once per second)
        self.step_count += 1
        if self.step_count % 20 == 0:
            self.get_logger().info(
                f'Costs[Pred:{pred_cost:.0f}, F:{front_cost:.0f}, L:{left_cost:.0f}, R:{right_cost:.0f}] | '
                f'DWA(vx:{self.dwa_cmd.linear.x:.2f}, vy:{self.dwa_cmd.linear.y:.2f}) -> '
                f'Fuzzy(vx:{fuzzy_vx:.2f}, vy:{fuzzy_vy:.2f}, wz:{fuzzy_wz:.2f})'
            )

        # 7. Record Data Row to CSV File
        now_sec = self.get_clock().now().nanoseconds * 1e-9 - self.start_time
        self.csv_writer.writerow([
            f'{now_sec:.3f}', f'{rx:.4f}', f'{ry:.4f}', f'{ryaw:.4f}',
            f'{pred_cost:.1f}', f'{front_cost:.1f}', f'{left_cost:.1f}',
            f'{right_cost:.1f}', f'{global_cost:.1f}',
            f'{self.dwa_cmd.linear.x:.4f}', f'{self.dwa_cmd.linear.y:.4f}', f'{self.dwa_cmd.angular.z:.4f}',
            f'{fuzzy_vx:.4f}', f'{fuzzy_vy:.4f}', f'{fuzzy_wz:.4f}'
        ])
        self.csv_file.flush()

    def destroy_node(self):
        """Ensures the CSV file is safely closed when the node shuts down."""
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

