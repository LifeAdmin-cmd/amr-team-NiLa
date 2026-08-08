#!/usr/bin/env python3
"""
controller.py

Orchestration only: owns the goal, runs the control loop, and calls
Robot for sensing/actuation and PotentialFieldPlanner for values.

    Robot                 -> get_lidar(), set_velocity(), get_goal_in_base_frame()
    PotentialFieldPlanner  -> goal + lidar in, (vx, vy, dist, reached) out
    Controller             -> ties the two together, decides speed limits
"""

import math

import rclpy
from rclpy.node import Node

from robot.robot import Robot
from path_and_motion_planning.potential_field_planner import PotentialFieldPlanner


class Controller(Node):

    # ── Goal, in the odom frame ──────────────────────────────────────────
    GOAL_X = 4.0
    GOAL_Y = 10.0
    GOAL_THETA = -1.0  # final heading, optional use

    # ── Motion limits ────────────────────────────────────────────────────
    MAX_LINEAR = 0.4    # m/s cap
    MAX_ANGULAR = 0.8   # rad/s cap

    CONTROL_PERIOD = 0.1  # s, 10 Hz control loop

    def __init__(self):
        super().__init__('controller')

        self.robot = Robot(self)
        self.planner = PotentialFieldPlanner(
            ka=0.4,
            kr=0.3,
            rho0=1.5,
            goal_tolerance=0.3,
            min_obstacle_range=0.15,
        )

        self.goal_reached = False
        self.timer = self.create_timer(self.CONTROL_PERIOD, self.control_loop)

        self.get_logger().info(
            f'Controller started. Goal (odom frame): ({self.GOAL_X:.2f}, {self.GOAL_Y:.2f})'
        )

    # ------------------------------------------------------------------ #
    def control_loop(self):
        if self.goal_reached:
            return

        lidar_data = self.robot.get_lidar()
        if lidar_data is None:
            self.get_logger().warn('Waiting for /scan...', throttle_duration_sec=2.0)
            return

        goal_bl = self.robot.get_goal_in_base_frame(self.GOAL_X, self.GOAL_Y)
        if goal_bl is None:
            self.get_logger().warn('Waiting for TF (odom -> base_link)...', throttle_duration_sec=2.0)
            return

        self.planner.set_goal(*goal_bl)

        vx, vy, dist_to_goal, goal_reached = self.planner.potential_field_planner_tick(
            lidar_data, robot_pos=(0.0, 0.0)
        )

        if goal_reached:
            self.goal_reached = True
            self.robot.stop()
            self.get_logger().info('Goal reached! Stopping.')
            return

        linear_x, angular_z = self._velocity_to_cmd(vx, vy)
        self.robot.set_velocity(linear_x, angular_z)

        self.get_logger().info(
            f"dist={dist_to_goal:.2f}m | v=({vx:.2f},{vy:.2f}) | "
            f"cmd=({linear_x:.2f},{angular_z:.2f})",
            throttle_duration_sec=1.0,
        )

    # ------------------------------------------------------------------ #
    def _velocity_to_cmd(self, vx: float, vy: float) -> tuple:
        """Convert the planner's raw (vx, vy) into a clamped unicycle command."""
        linear_x = vx
        angular_z = math.atan2(vy, max(abs(vx), 1e-3))

        linear_x = max(-self.MAX_LINEAR, min(self.MAX_LINEAR, linear_x))
        angular_z = max(-self.MAX_ANGULAR, min(self.MAX_ANGULAR, angular_z))
        return linear_x, angular_z


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.robot.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
