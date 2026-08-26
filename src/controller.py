#!/usr/bin/env python3
"""
controller.py

Orchestration only: everything below is planned in a LOCAL frame where
(0, 0) is wherever the robot happens to be when the controller starts.
That origin is captured once (from odom, via TF), then every waypoint is
shifted by it before being handed to Robot -- so the map/goal are always
"relative to the robot's starting pose", not a fixed odom position.

Exploration loop: build an occupancy map from lidar as we drive, ask
FrontierExplorer for the closest unexplored frontier, flood-fill a path to
it, drive there with the potential field planner, then repeat. Stops once
no frontier is left (or none are reachable).

Bounded map: anything with x or y outside [BOUND_MIN, BOUND_MAX] (meters,
centered on the robot's start) is always treated as an obstacle, so
exploration can't wander off into an unbounded area -- see
OccupancyGridMapper(bound_min=..., bound_max=...).

    Robot                 -> get_robot_pose_in_odom(), get_lidar(), set_velocity()
    OccupancyGridMapper    -> lidar + pose in, occupancy grid out
    FrontierExplorer       -> grid + reachability in, next frontier cell out
    FloodFillPlanner       -> grid + start/goal in, waypoint list out
    PotentialFieldPlanner  -> current waypoint + lidar in, (vx, vy, dist, reached) out
    Controller             -> captures origin, ties it all together

Note: this node does NOT draw the map -- run mapping/mapping_node.py
alongside it for the live view, so only one process owns the plot.
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from robot.robot import Robot
from path_and_motion_planning.potential_field_planner import PotentialFieldPlanner
from path_and_motion_planning.flood_fill_planner import FloodFillPlanner
from mapping.occupancy_grid_mapper import OccupancyGridMapper
from exploration.frontier_explorer import FrontierExplorer


class Controller(Node):

    # ── Map config ────────────────────────────────────────────────────────
    MAP_RESOLUTION = 0.1    # meters per cell
    MAX_MAP_SIZE = 20.0     # meters, side length of the allocated grid
    BOUND_MIN = -10.0       # "Bounded map": x/y below this is always an obstacle
    BOUND_MAX = 10.0        # x/y above this is always an obstacle
    INFLATION_RADIUS_CELLS = 3

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
        self.mapper = OccupancyGridMapper(
            size_m=self.MAX_MAP_SIZE,
            resolution=self.MAP_RESOLUTION,
            bound_min=self.BOUND_MIN,
            bound_max=self.BOUND_MAX,
        )
        self.explorer = FrontierExplorer()
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        self.origin = None            # (x, y) in odom -- captured on first tick
        self.waypoints = None         # current path to the active frontier
        self.waypoint_idx = 0
        self.exploration_done = False

        self.timer = self.create_timer(self.CONTROL_PERIOD, self.control_loop)

        self.get_logger().info('Controller started. Waiting to capture starting pose...')

    # ------------------------------------------------------------------ #
    def _cell_to_local(self, cell) -> tuple:
        """Grid (row, col) -> (x, y) in the robot-relative local frame."""
        row, col = cell
        x = self.mapper.origin_x + col * self.mapper.resolution
        y = self.mapper.origin_y + row * self.mapper.resolution
        return x, y

    def _local_to_cell(self, x: float, y: float) -> tuple:
        """(x, y) in the robot-relative local frame -> grid (row, col)."""
        row, col = self.mapper._world_to_cell(x, y)
        return row, col

    # ------------------------------------------------------------------ #
    def _publish_planned_path(self):
        """Publish the current waypoint list (odom frame) so mapping_node can draw it."""
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        for wp_x, wp_y in self.waypoints:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = self.origin[0] + wp_x
            pose.pose.position.y = self.origin[1] + wp_y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.path_pub.publish(msg)

    # ------------------------------------------------------------------ #
    def _plan_next_frontier(self, robot_cell):
        """Find the closest reachable frontier and flood-fill a path to it.
        Returns True if a new path was planned or should be retried next tick,
        False only once exploration is genuinely complete."""
        grid = self.mapper.get_probability_grid()
        flood_fill = FloodFillPlanner(
            grid,
            obstacle_threshold=0.5,
            inflation_radius_cells=self.INFLATION_RADIUS_CELLS,
            inflate_threshold=0.9,  # only inflate around confirmed obstacles, not unknown cells
        )
        dist_from_robot = flood_fill.flood_fill(robot_cell)

        target_cell = self.explorer.select_target(grid, dist_from_robot)
        if target_cell is None:
            # Either no frontiers exist at all (done), or the only ones left
            # are too close to be worth driving to right now (not done --
            # retry next tick once the map has moved on a bit).
            return not self.explorer.is_exploration_complete(grid)

        full_path = flood_fill.plan(robot_cell, target_cell)
        if full_path is None:
            # Reachability said yes but planning disagrees (e.g. inflation) --
            # blacklist it like any other unreachable frontier and try again next tick.
            self.explorer.blacklist.add(target_cell)
            return True

        waypoint_cells = flood_fill.simplify_path(full_path)
        self.waypoints = [self._cell_to_local(c) for c in waypoint_cells]
        self.waypoint_idx = 0
        self._publish_planned_path()

        self.get_logger().info(
            f'New frontier target at cell {target_cell}, {len(self.waypoints)} waypoints planned.'
        )
        return True

    # ------------------------------------------------------------------ #
    def control_loop(self):
        if self.exploration_done:
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
            self.get_logger().info(
                f'Starting pose captured at odom ({origin[0]:.2f}, {origin[1]:.2f}) -> local (0, 0).'
            )

        lidar_data = self.robot.get_lidar()
        if lidar_data is None:
            self.get_logger().warn('Waiting for /scan...', throttle_duration_sec=2.0)
            return

        current_pose = self.robot.get_robot_pose_in_odom()
        if current_pose is None:
            self.get_logger().warn('Waiting for TF (odom -> base_link)...', throttle_duration_sec=2.0)
            return

        pose_x, pose_y, pose_yaw = current_pose
        local_x = pose_x - self.origin[0]
        local_y = pose_y - self.origin[1]

        self.mapper.update(
            local_x, local_y, pose_yaw,
            lidar_data.ranges, lidar_data.angle_min, lidar_data.angle_increment,
            range_max=lidar_data.range_max or 10.0,
        )

        robot_cell = self._local_to_cell(local_x, local_y)

        if not self.waypoints:
            if not self._plan_next_frontier(robot_cell):
                self.exploration_done = True
                self.robot.stop()
                self.get_logger().info('Exploration complete -- no reachable frontiers left.')
            return

        wp_x, wp_y = self.waypoints[self.waypoint_idx]
        target_x = self.origin[0] + wp_x
        target_y = self.origin[1] + wp_y

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
                self.waypoints = None  # reached the frontier -- replan next tick
                self.get_logger().info('Frontier reached, replanning...')
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
