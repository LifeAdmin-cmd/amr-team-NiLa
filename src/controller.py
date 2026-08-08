#!/usr/bin/env python3
"""
controller.py

The controller: owns /scan, /cmd_vel, TF, and the control loop.

The goal is defined here. Each scan, the controller calls
PotentialFieldPlanner with just (goal, lidar_data, robot_pos) and gets
back raw movement values -- nothing more. Converting those values into an
actual Twist, clamping to speed limits, publishing, and stopping at the
goal all happen here in the controller.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PointStamped

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transform support)

from path_and_motion_planning.potential_field_planner import PotentialFieldPlanner, LidarScan


class Controller(Node):

    # ── Goal, in the odom frame ──────────────────────────────────────────
    GOAL_X = 0.0
    GOAL_Y = 3.0
    GOAL_THETA = -1.0  # final heading, optional use

    # ── Frames ───────────────────────────────────────────────────────────
    ODOM_FRAME = 'odom'
    BASE_FRAME = 'base_link'

    # ── Motion limits ────────────────────────────────────────────────────
    MAX_LINEAR = 0.4    # m/s cap
    MAX_ANGULAR = 0.8   # rad/s cap

    def __init__(self):
        super().__init__('controller')

        self.odom_goal_x = self.GOAL_X
        self.odom_goal_y = self.GOAL_Y
        self.odom_goal_theta = self.GOAL_THETA

        # The planner is only ever called for VALUES: goal in, lidar in,
        # (vx, vy, dist_to_goal, goal_reached) out. No ROS knowledge inside it.
        self.planner = PotentialFieldPlanner(
            ka=0.4,
            kr=0.3,
            rho0=1.5,
            goal_tolerance=0.3,
            min_obstacle_range=0.15,
        )

        self.goal_reached = False

        # ── TF ───────────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Pub / Sub ────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, sensor_qos
        )

        self.get_logger().info(
            f'Controller started. Goal (odom frame): '
            f'({self.odom_goal_x:.2f}, {self.odom_goal_y:.2f})'
        )

    # ------------------------------------------------------------------ #
    def scan_callback(self, msg: LaserScan):
        if self.goal_reached:
            return

        # 1) Transform goal from odom -> base_link (planner works in the
        #    robot's own frame).
        try:
            if not self.tf_buffer.can_transform(
                self.BASE_FRAME, self.ODOM_FRAME, rclpy.time.Time()
            ):
                self.get_logger().warn(
                    f'Waiting for TF {self.ODOM_FRAME} -> {self.BASE_FRAME}...',
                    throttle_duration_sec=2.0,
                )
                return

            goal_odom = PointStamped()
            goal_odom.header.frame_id = self.ODOM_FRAME
            goal_odom.point.x = self.odom_goal_x
            goal_odom.point.y = self.odom_goal_y
            goal_bl = self.tf_buffer.transform(goal_odom, self.BASE_FRAME)
        except Exception as e:
            self.get_logger().warn(f'TF error: {e}')
            return

        self.planner.set_goal(goal_bl.point.x, goal_bl.point.y)

        # 2) Build the lidar reading the planner expects.
        lidar_data = LidarScan(
            ranges=msg.ranges,
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            range_max=msg.range_max,
        )

        # 3) Call the planner for VALUES ONLY -- goal + lidar in,
        #    (vx, vy, dist_to_goal, goal_reached) out.
        vx, vy, dist_to_goal, goal_reached = self.planner.potential_field_planner_tick(
            lidar_data, robot_pos=(0.0, 0.0)
        )

        # 4) Everything about actually moving the robot happens here.
        if goal_reached:
            self.goal_reached = True
            self.cmd_pub.publish(Twist())  # zero velocity
            self.get_logger().info('Goal reached! Stopping.')
            return

        cmd = self._velocity_to_cmd(vx, vy)
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"dist={dist_to_goal:.2f}m | "
            f"v=({vx:.2f},{vy:.2f}) | "
            f"cmd=({cmd.linear.x:.2f},{cmd.angular.z:.2f})",
            throttle_duration_sec=1.0,
        )

    # ------------------------------------------------------------------ #
    def _velocity_to_cmd(self, vx: float, vy: float) -> Twist:
        """Convert the planner's raw (vx, vy) into a clamped unicycle Twist."""
        linear_x = vx
        angular_z = math.atan2(vy, max(abs(vx), 1e-3))

        linear_x = max(-self.MAX_LINEAR, min(self.MAX_LINEAR, linear_x))
        angular_z = max(-self.MAX_ANGULAR, min(self.MAX_ANGULAR, angular_z))

        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        return cmd

    # ------------------------------------------------------------------ #
    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
