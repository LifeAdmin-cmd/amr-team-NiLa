#!/usr/bin/env python3
"""
mapping_node.py

ROS 2 Node that wraps the OccupancyGridMapper and integrates it with the
Robile. Draws the map live with matplotlib so you can watch it build up
while the robot moves. Publishes to /map as an OccupancyGrid too.
"""

import math

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan

from mapping.occupancy_grid_mapper import OccupancyGridMapper

THRESH_FREE = 0.7
THRESH_OCC = 0.4

DIST_THRESH = 0.1    # meters, min robot movement before re-updating
ANGLE_THRESH = 0.05  # radians


class MappingNode(Node):

    MAP_SIZE_M = 20.0
    RESOLUTION = 0.1

    def __init__(self):
        super().__init__('mapping_node')

        self.mapper = OccupancyGridMapper(size_m=self.MAP_SIZE_M, resolution=self.RESOLUTION)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False

        self.last_update_pose = None  # (x, y, yaw)
        self.latest_scan = None

        self.path_x = []  # traveled path, for drawing on the map
        self.path_y = []
        self.planned_path_x = []  # target path from the controller, for drawing
        self.planned_path_y = []

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path_sub = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)

        # Mapping + publishing loop
        self.timer = self.create_timer(0.2, self.mapping_loop)

        # Live plot
        self._setup_plot()

        self.get_logger().info('Mapping node started. Waiting for /odom and /scan...')

    # ------------------------------------------------------------------ #
    def _setup_plot(self):
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.ax.set_title('Occupancy Grid (live)')
        plt.show(block=False)

    def _refresh_plot(self):
        grid = self.mapper.get_probability_grid()
        extent = [
            self.mapper.origin_x,
            self.mapper.origin_x + self.mapper.size_cells * self.mapper.resolution,
            self.mapper.origin_y,
            self.mapper.origin_y + self.mapper.size_cells * self.mapper.resolution,
        ]

        self.ax.clear()
        self.ax.imshow(grid, cmap='Greys', vmin=0.0, vmax=1.0, origin='lower', extent=extent)
        self.ax.plot(self.path_x, self.path_y, '-', color='deepskyblue', linewidth=1.5)
        self.ax.plot(self.planned_path_x, self.planned_path_y, '-o', color='orange', linewidth=1.5, markersize=4)
        self.ax.plot(self.robot_x, self.robot_y, 'rs', markersize=8)
        self.ax.set_title('Occupancy Grid (live)')
        self.ax.set_xlim(extent[0], extent[1])
        self.ax.set_ylim(extent[2], extent[3])
        self.ax.set_aspect('equal')

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    # ------------------------------------------------------------------ #
    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_received = True

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def path_callback(self, msg: Path):
        self.planned_path_x = [pose.pose.position.x for pose in msg.poses]
        self.planned_path_y = [pose.pose.position.y for pose in msg.poses]

    # ------------------------------------------------------------------ #
    def _moved_enough(self) -> bool:
        if self.last_update_pose is None:
            return True
        lx, ly, lyaw = self.last_update_pose
        d_dist = math.hypot(self.robot_x - lx, self.robot_y - ly)
        d_angle = abs(self.robot_yaw - lyaw)
        d_angle = min(d_angle, 2 * math.pi - d_angle)
        return d_dist >= DIST_THRESH or d_angle >= ANGLE_THRESH

    def mapping_loop(self):
        if not self.odom_received or self.latest_scan is None:
            return

        if self._moved_enough():
            scan = self.latest_scan
            self.mapper.update(
                self.robot_x, self.robot_y, self.robot_yaw,
                scan.ranges, scan.angle_min, scan.angle_increment,
                range_min=scan.range_min, range_max=scan.range_max,
            )
            self.last_update_pose = (self.robot_x, self.robot_y, self.robot_yaw)
            self.path_x.append(self.robot_x)
            self.path_y.append(self.robot_y)

        self._publish_map()
        self._refresh_plot()

    def _publish_map(self):
        grid = self.mapper.get_probability_grid()

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.info.resolution = self.mapper.resolution
        msg.info.width = self.mapper.size_cells
        msg.info.height = self.mapper.size_cells
        msg.info.origin.position.x = self.mapper.origin_x
        msg.info.origin.position.y = self.mapper.origin_y
        msg.info.origin.orientation.w = 1.0

        flat = np.full(grid.shape, -1, dtype=np.int8)
        flat[grid > THRESH_FREE] = 0
        flat[grid < THRESH_OCC] = 100
        msg.data = flat.flatten().tolist()

        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
