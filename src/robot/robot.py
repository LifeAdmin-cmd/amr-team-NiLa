#!/usr/bin/env python3
"""
robot/robot.py

Robot: the hardware/topic abstraction layer. Wraps /scan, /cmd_vel, and TF
behind a simple interface (get_lidar, set_velocity, get_goal_in_base_frame,
stop) so the controller never has to touch ROS message types directly.

Usage
-----
    class Controller(Node):
        def __init__(self):
            super().__init__('controller')
            self.robot = Robot(self)
            ...
        def loop(self):
            lidar = self.robot.get_lidar()
            self.robot.set_velocity(0.2, 0.0)
"""

import math
from typing import Optional, Tuple

import rclpy.time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PointStamped

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transform support)

from path_and_motion_planning.potential_field_planner import LidarScan


class Robot:
    """Thin wrapper around a robot's sensing/actuation/frames.

    Takes an existing rclpy Node to create its publisher/subscription/TF
    listener under (no separate node/executor needed).
    """

    def __init__(
        self,
        node: Node,
        cmd_vel_topic: str = "/cmd_vel",
        scan_topic: str = "/scan",
        base_frame: str = "base_link",
        odom_frame: str = "odom",
    ):
        self.node = node
        self.base_frame = base_frame
        self.odom_frame = odom_frame

        self._latest_scan: Optional[LaserScan] = None

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._cmd_pub = node.create_publisher(Twist, cmd_vel_topic, 10)
        self._scan_sub = node.create_subscription(
            LaserScan, scan_topic, self._scan_callback, sensor_qos
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)

    # ------------------------------------------------------------------ #
    # Sensing
    # ------------------------------------------------------------------ #
    def _scan_callback(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    def get_lidar(self) -> Optional[LidarScan]:
        """Latest lidar reading as a LidarScan, or None if nothing received yet."""
        if self._latest_scan is None:
            return None
        msg = self._latest_scan
        return LidarScan(
            ranges=msg.ranges,
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            range_max=msg.range_max,
        )

    # ------------------------------------------------------------------ #
    # Frames
    # ------------------------------------------------------------------ #
    def get_goal_in_base_frame(self, x: float, y: float) -> Optional[Tuple[float, float]]:
        """Transform a goal point from the odom frame into base_link.
        Returns None if the transform isn't available yet."""
        try:
            if not self._tf_buffer.can_transform(
                self.base_frame, self.odom_frame, rclpy.time.Time()
            ):
                return None

            point = PointStamped()
            point.header.frame_id = self.odom_frame
            point.point.x = x
            point.point.y = y
            transformed = self._tf_buffer.transform(point, self.base_frame)
            return transformed.point.x, transformed.point.y
        except Exception as e:
            self.node.get_logger().warn(f"TF error: {e}")
            return None

    # ------------------------------------------------------------------ #
    # Actuation
    # ------------------------------------------------------------------ #
    def set_velocity(self, linear_x: float, angular_z: float) -> None:
        """Publish a velocity command."""
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self._cmd_pub.publish(cmd)

    def stop(self) -> None:
        """Publish zero velocity."""
        self._cmd_pub.publish(Twist())
