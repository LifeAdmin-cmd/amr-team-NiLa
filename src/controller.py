#!/usr/bin/env python3
"""
controller.py

Orchestration only: everything below is planned in a LOCAL frame where
(0, 0) is wherever the robot happens to be when the controller starts.
That origin is captured once (from odom, via TF), then every waypoint is
shifted by it before being handed to Robot -- so the map/obstacle/goal are
always "relative to the robot's starting pose", not a fixed odom position.

    Robot                 -> get_robot_pose_in_odom(), get_lidar(), set_velocity()
    FloodFillPlanner       -> local map + start/goal in, waypoint list out
    PotentialFieldPlanner  -> current waypoint + lidar in, (vx, vy, dist, reached) out
    Controller             -> captures origin, ties it all together
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node

from robot.robot import Robot
from path_and_motion_planning.potential_field_planner import PotentialFieldPlanner
from path_and_motion_planning.flood_fill_planner import FloodFillPlanner


class Controller(Node):

    # ── Local grid -> local (robot-relative) frame conversion ────────────
    GRID_RESOLUTION = 0.1   # meters per cell
    GRID_ORIGIN_X = -5.0    # local x of grid cell (row=0, col=0)
    GRID_ORIGIN_Y = -5.0    # local y of grid cell (row=0, col=0)
    GRID_SIZE = 60          # cells per side -> 6m x 6m, covers local [-5, 1]

    # ── The square obstacle, relative to the robot's start (0, 0) ────────
    OBSTACLE_X_MIN = -3.0
    OBSTACLE_X_MAX = -1.0
    OBSTACLE_Y_MIN = -3.0
    OBSTACLE_Y_MAX = -1.0

    # ── Start / goal, relative to the robot's start (0, 0) ───────────────
    START_LOCAL = (0.0, 0.0)     # always the robot's own starting pose
    GOAL_LOCAL = (-4.0, -4.0)    # straight line to here cuts through the obstacle

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

        self.origin = None          # (x, y) in odom -- captured on first tick
        self.waypoints = None       # built once origin is known
        self.waypoint_idx = 0
        self.goal_reached = False

        self.timer = self.create_timer(self.CONTROL_PERIOD, self.control_loop)

        self.get_logger().info('Controller started. Waiting to capture starting pose...')

    # ------------------------------------------------------------------ #
    def _build_waypoints(self):
        """Flood fill the local map once, in the robot-relative frame."""
        grid = self._generate_square_obstacle_grid()
        start_cell = self._local_to_cell(*self.START_LOCAL)
        goal_cell = self._local_to_cell(*self.GOAL_LOCAL)

        flood_fill = FloodFillPlanner(grid, obstacle_threshold=0.5, inflation_radius_cells=3)
        _, waypoint_cells = flood_fill.get_waypoints(start_cell, goal_cell)

        if waypoint_cells is None:
            raise RuntimeError('No path around the obstacle from start to goal.')

        return [self._cell_to_local(c) for c in waypoint_cells]

    def _generate_square_obstacle_grid(self) -> np.ndarray:
        """Free everywhere except one square obstacle block."""
        grid = np.zeros((self.GRID_SIZE, self.GRID_SIZE))

        r_min, c_min = self._local_to_cell(self.OBSTACLE_X_MIN, self.OBSTACLE_Y_MIN)
        r_max, c_max = self._local_to_cell(self.OBSTACLE_X_MAX, self.OBSTACLE_Y_MAX)
        grid[min(r_min, r_max):max(r_min, r_max) + 1,
             min(c_min, c_max):max(c_min, c_max) + 1] = 1.0

        return grid

    # ------------------------------------------------------------------ #
    def _cell_to_local(self, cell) -> tuple:
        """Grid (row, col) -> (x, y) in the robot-relative local frame."""
        row, col = cell
        x = self.GRID_ORIGIN_X + col * self.GRID_RESOLUTION
        y = self.GRID_ORIGIN_Y + row * self.GRID_RESOLUTION
        return x, y

    def _local_to_cell(self, x: float, y: float) -> tuple:
        """(x, y) in the robot-relative local frame -> grid (row, col)."""
        col = int(round((x - self.GRID_ORIGIN_X) / self.GRID_RESOLUTION))
        row = int(round((y - self.GRID_ORIGIN_Y) / self.GRID_RESOLUTION))
        return row, col

    # ------------------------------------------------------------------ #
    def control_loop(self):
        if self.goal_reached:
            return

        # Capture wherever the robot is right now as the local origin,
        # the first time TF becomes available. Everything after this is
        # planned relative to that captured point, not raw odom.
        if self.origin is None:
            origin = self.robot.get_robot_pose_in_odom()
            if origin is None:
                self.get_logger().warn('Waiting for starting pose (TF)...', throttle_duration_sec=2.0)
                return
            self.origin = origin
            self.waypoints = self._build_waypoints()
            self.get_logger().info(
                f'Starting pose captured at odom ({origin[0]:.2f}, {origin[1]:.2f}) -> local (0, 0). '
                f'{len(self.waypoints)} waypoints planned.'
            )

        lidar_data = self.robot.get_lidar()
        if lidar_data is None:
            self.get_logger().warn('Waiting for /scan...', throttle_duration_sec=2.0)
            return

        local_x, local_y = self.waypoints[self.waypoint_idx]
        target_x = self.origin[0] + local_x
        target_y = self.origin[1] + local_y

        goal_bl = self.robot.get_goal_in_base_frame(target_x, target_y)
        if goal_bl is None:
            self.get_logger().warn('Waiting for TF (odom -> base_link)...', throttle_duration_sec=2.0)
            return

        self.planner.set_goal(*goal_bl)

        vx, vy, dist_to_goal, waypoint_reached = self.planner.potential_field_planner_tick(
            lidar_data, robot_pos=(0.0, 0.0)
        )

        if waypoint_reached:
            self.waypoint_idx += 1
            if self.waypoint_idx >= len(self.waypoints):
                self.goal_reached = True
                self.robot.stop()
                self.get_logger().info('Goal reached! All waypoints complete.')
            else:
                self.get_logger().info(
                    f'Waypoint {self.waypoint_idx}/{len(self.waypoints)} reached, '
                    f'moving to next.'
                )
            return

        linear_x, angular_z = self._velocity_to_cmd(vx, vy)
        self.robot.set_velocity(linear_x, angular_z)

        self.get_logger().info(
            f"wp {self.waypoint_idx}/{len(self.waypoints)} | dist={dist_to_goal:.2f}m | "
            f"v=({vx:.2f},{vy:.2f}) | cmd=({linear_x:.2f},{angular_z:.2f})",
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
