#!/usr/bin/env python3
"""
mcl_node.py

ROS 2 Node that wraps the Particle Filter and integrates it with the Robile.
It uses a dummy map (similar to the one in controller.py) for testing in the absence
of a full SLAM map.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import LaserScan
import tf2_ros

def euler_from_quaternion(quaternion):
    x, y, z, w = quaternion
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = math.asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy
    ]

from localisation.particle_filter import ParticleFilter


class MCLNode(Node):

    # ── Map Settings (Matching controller.py for testing) ──────────────
    GRID_RESOLUTION = 0.1
    GRID_ORIGIN_X = -5.0
    GRID_ORIGIN_Y = -5.0
    GRID_SIZE = 60  # 6m x 6m

    OBSTACLE_X_MIN = -3.0
    OBSTACLE_X_MAX = -1.0
    OBSTACLE_Y_MIN = -3.0
    OBSTACLE_Y_MAX = -1.0

    def __init__(self):
        super().__init__('mcl_node')
        
        # Build the dummy map
        self.grid_map = self._generate_square_obstacle_grid()
        
        # Initialize Particle Filter
        self.pf = ParticleFilter(
            num_particles=500,
            grid_map=self.grid_map,
            resolution=self.GRID_RESOLUTION,
            origin_x=self.GRID_ORIGIN_X,
            origin_y=self.GRID_ORIGIN_Y,
            initial_pose=(0.0, 0.0, 0.0), # Assuming robot starts at (0, 0, 0)
            initial_noise=(0.2, 0.2, 0.1)
        )
        
        # TF listener for Odometry (odom -> base_link)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # TF broadcaster (map -> odom)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Pose publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/mcl_pose', 10)
        
        # LaserScan subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        # State variables
        self.last_odom_pose = None
        self.latest_scan = None
        
        # Timer for MCL loop (10 Hz)
        self.timer = self.create_timer(0.1, self.mcl_loop)
        
        self.get_logger().info("MCL Node started. Waiting for TF and Scan...")

    def _generate_square_obstacle_grid(self) -> np.ndarray:
        grid = np.zeros((self.GRID_SIZE, self.GRID_SIZE))
        
        col_min = int(round((self.OBSTACLE_X_MIN - self.GRID_ORIGIN_X) / self.GRID_RESOLUTION))
        col_max = int(round((self.OBSTACLE_X_MAX - self.GRID_ORIGIN_X) / self.GRID_RESOLUTION))
        row_min = int(round((self.OBSTACLE_Y_MIN - self.GRID_ORIGIN_Y) / self.GRID_RESOLUTION))
        row_max = int(round((self.OBSTACLE_Y_MAX - self.GRID_ORIGIN_Y) / self.GRID_RESOLUTION))
        
        grid[min(row_min, row_max):max(row_min, row_max) + 1,
             min(col_min, col_max):max(col_min, col_max) + 1] = 1.0
             
        return grid

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def get_odom_pose(self):
        try:
            now = rclpy.time.Time()
            if not self.tf_buffer.can_transform('odom', 'base_link', now):
                return None
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', now)
            
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            
            q = trans.transform.rotation
            _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
            
            return (x, y, theta)
        except Exception as e:
            return None

    def mcl_loop(self):
        # 1. Get current odometry
        current_odom = self.get_odom_pose()
        if current_odom is None:
            return
            
        # 2. Prediction Step (Motion Update)
        if self.last_odom_pose is not None:
            dx = current_odom[0] - self.last_odom_pose[0]
            dy = current_odom[1] - self.last_odom_pose[1]
            dtheta = current_odom[2] - self.last_odom_pose[2]
            
            # Normalize angle difference
            dtheta = (dtheta + math.pi) % (2 * math.pi) - math.pi
            
            # Only update if moved significantly
            if abs(dx) > 0.01 or abs(dy) > 0.01 or abs(dtheta) > 0.01:
                # Transform to local robot frame (vx, vy) for the predict model if needed,
                # but our predict model assumes dx, dy are in world frame? 
                # Wait, our ParticleFilter.predict() applies local (forward/lateral) motion.
                # Let's compute local translation:
                # current_odom[2] is the world heading. 
                # Actually, easier to use odometry differences directly as local commands
                c = math.cos(-self.last_odom_pose[2])
                s = math.sin(-self.last_odom_pose[2])
                local_dx = dx * c - dy * s
                local_dy = dx * s + dy * c
                
                self.pf.predict(local_dx, local_dy, dtheta, noise_std=(0.05, 0.05, 0.05))
                
        self.last_odom_pose = current_odom

        # 3. Update Step (Sensor Update) & Resample
        if self.latest_scan is not None:
            scan = self.latest_scan
            ranges = list(scan.ranges)
            
            # Calculate angles for each ray
            angles = [scan.angle_min + i * scan.angle_increment for i in range(len(ranges))]
            
            self.pf.update(ranges, angles, max_range=scan.range_max if scan.range_max > 0 else 10.0)
            self.pf.resample()
            
            # Consume the scan so we don't update multiple times on the same data
            self.latest_scan = None
            
        # 4. Publish Pose and TF
        est_x, est_y, est_theta = self.pf.get_estimated_pose()
        
        # Publish Pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        pose_msg.pose.position.x = est_x
        pose_msg.pose.position.y = est_y
        
        q = quaternion_from_euler(0, 0, est_theta)
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        
        self.pose_pub.publish(pose_msg)
        
        # Calculate map -> odom TF
        # Map to base_link is (est_x, est_y, est_theta)
        # Odom to base_link is (current_odom)
        # We need Map to Odom = (Map -> Base) * (Odom -> Base)^-1
        
        # Simple 2D TF math:
        c_odom = math.cos(current_odom[2])
        s_odom = math.sin(current_odom[2])
        
        map_odom_theta = est_theta - current_odom[2]
        map_odom_theta = (map_odom_theta + math.pi) % (2 * math.pi) - math.pi
        
        c_mo = math.cos(map_odom_theta)
        s_mo = math.sin(map_odom_theta)
        
        map_odom_x = est_x - (current_odom[0] * c_mo - current_odom[1] * s_mo)
        map_odom_y = est_y - (current_odom[0] * s_mo + current_odom[1] * c_mo)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        
        t.transform.translation.x = map_odom_x
        t.transform.translation.y = map_odom_y
        t.transform.translation.z = 0.0
        
        q_mo = quaternion_from_euler(0, 0, map_odom_theta)
        t.transform.rotation.x = q_mo[0]
        t.transform.rotation.y = q_mo[1]
        t.transform.rotation.z = q_mo[2]
        t.transform.rotation.w = q_mo[3]
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = MCLNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
