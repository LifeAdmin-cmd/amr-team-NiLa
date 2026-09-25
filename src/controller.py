#!/usr/bin/env python3
"""
controller.py

Orchestration only: everything below is planned in a LOCAL frame where
(0, 0) is wherever the robot happens to be when the controller starts.
That origin is captured once (from odom, via TF), then every waypoint is
shifted by it before being handed to Robot -- so the map/goal are always
"relative to the robot's starting pose", not a fixed odom position.

Exploration loop: build an occupancy map from lidar as we drive, ask
FrontierExplorer for the closest unexplored frontier that doesn't require
a sharp blind-spot turn, flood-fill a path to it, drive there with the
potential field planner (which itself avoids commanding turns into the
robot's rear blind spot by driving backward instead), then repeat. Stops
once no frontier is left (or none are reachable).

Per-waypoint motion: whenever a new waypoint becomes active (a fresh path
is planned, or we advance to the next waypoint in the current path), the
robot first rotates in place to face that waypoint, then switches to
normal potential-field-driven translation once roughly aligned. See
waypoint_state / ROTATE_ANGLE_TOLERANCE / K_ANGULAR_ROTATE below.

Bounded map: anything with x or y outside [BOUND_MIN, BOUND_MAX] (meters,
centered on the robot's start) is always treated as an obstacle, so
exploration can't wander off into an unbounded area -- see
OccupancyGridMapper(bound_min=..., bound_max=...).

    Robot                 -> get_robot_pose_in_odom(), get_lidar(), set_velocity()
    OccupancyGridMapper    -> lidar + pose in, occupancy grid out
    FrontierExplorer       -> grid + reachability + robot pose/heading in, next frontier cell out
    FloodFillPlanner       -> grid + start/goal in, waypoint list out
    PotentialFieldPlanner  -> current waypoint + lidar in, (vx, vy, dist, reached, reverse) out
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
    PATH_REPLAN_EVERY_N_TICKS = 5   # ~2 Hz -- recompute the route to the current target
    REPLAN_FAILURE_GRACE = 10        # consecutive failed replans before giving up on a target
    WAYPOINT_TOLERANCE = 0.5        # meters -- intermediate waypoints, tighter than goal_tolerance
                                      # so passing through one actually requires driving there

    # ── Motion limits ────────────────────────────────────────────────────
    MAX_LINEAR = 1.0    # m/s cap
    MAX_ANGULAR = 1.0 # rad/s cap

    # ── Rotate-to-face-waypoint ──────────────────────────────────────────
    ROTATE_ANGLE_TOLERANCE = math.radians(5)  # "close enough" to stop pure rotation
    K_ANGULAR_ROTATE = 1.0                     # gain used only while rotating in place

    CONTROL_PERIOD = 0.025  # s, 10 Hz control loop

    def __init__(self):
        super().__init__('controller')

        self.robot = Robot(self)
        self.planner = PotentialFieldPlanner(
            ka=0.3,
            kr=0.5,
            rho0=0.5,
            goal_tolerance=0.5,
            min_obstacle_range=0.3,
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
        self.waypoint_state = 'ROTATE'  # 'ROTATE' (facing new waypoint) or 'MOVE'
        self.current_target_cell = None
        self.exploration_done = False
        self._replan_tick = 0
        self._replan_fail_streak = 0   # consecutive failed replans to current_target_cell

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
    def _make_flood_fill(self, grid):
        return FloodFillPlanner(
            grid,
            obstacle_threshold=0.5,
            inflation_radius_cells=self.INFLATION_RADIUS_CELLS,
            inflate_threshold=0.9,  # only inflate around confirmed obstacles, not unknown cells
        )

    def _replan_to_current_target(self, robot_cell) -> bool:
        """Recompute the route to the current target against the latest map.
        Returns False only once REPLAN_FAILURE_GRACE consecutive replans
        have failed -- a single failed plan (e.g. one noisy inflated
        obstacle straddling the path for a tick) is not enough to give up
        on an otherwise-good target."""
        if self.current_target_cell is None:
            return True

        grid = self.mapper.get_probability_grid()
        flood_fill = self._make_flood_fill(grid)
        full_path = flood_fill.plan(robot_cell, self.current_target_cell)

        if full_path is None:
            self._replan_fail_streak += 1
            self.get_logger().info(
                f'Replan to {self.current_target_cell} failed '
                f'({self._replan_fail_streak}/{self.REPLAN_FAILURE_GRACE})...',
                throttle_duration_sec=1.0,
            )
            if self._replan_fail_streak >= self.REPLAN_FAILURE_GRACE:
                return False  # confirmed unreachable -- caller picks a new one
            return True  # still within grace period, keep current target/path for now

        self._replan_fail_streak = 0
        self.explorer.report_reachable(self.current_target_cell)
        waypoint_cells = flood_fill.simplify_path(full_path)
        self.waypoints = [self._cell_to_local(c) for c in waypoint_cells]
        # NOTE: this is a periodic *replan of the route to the same target*,
        # not a new waypoint -- do not reset waypoint_idx/waypoint_state here.
        # The occupancy grid keeps changing every tick (even while only
        # rotating, since new lidar sweeps land differently as heading
        # changes), so flood-fill can return a slightly different waypoint
        # list on every single replan. Forcing waypoint_idx back to 0 and
        # waypoint_state back to 'ROTATE' every ~0.5s would restart alignment
        # before it ever finishes, and the robot would never get to drive.
        # Clamp the index in case the new (possibly shorter) path has fewer
        # waypoints than before.
        self.waypoint_idx = min(self.waypoint_idx, len(self.waypoints) - 1)
        self._publish_planned_path()
        return True

    # ------------------------------------------------------------------ #
    def _plan_next_frontier(self, robot_cell, robot_heading):
        """Find the closest reachable frontier that doesn't require a sharp
        blind-spot turn, and flood-fill a path to it. Returns True if a new
        path was planned or should be retried next tick, False only once
        exploration is genuinely complete."""
        grid = self.mapper.get_probability_grid()
        flood_fill = self._make_flood_fill(grid)
        dist_from_robot = flood_fill.flood_fill(robot_cell)

        target_cell = self.explorer.select_target(
            grid, dist_from_robot, robot_cell, robot_heading
        )
        if target_cell is None:
            # Either no frontiers exist at all (done), or the only ones left
            # are too close to be worth driving to right now (not done --
            # retry next tick once the map has moved on a bit).
            return not self.explorer.is_exploration_complete(grid)

        full_path = flood_fill.plan(robot_cell, target_cell)
        if full_path is None:
            # Reachability said yes but planning disagrees (e.g. inflation).
            # Route through the same grace period as an established target,
            # rather than blacklisting on the very first failed attempt.
            self._replan_fail_streak += 1
            if self._replan_fail_streak >= self.REPLAN_FAILURE_GRACE:
                self.explorer.report_unreachable(target_cell)
                self._replan_fail_streak = 0
            return True

        self._replan_fail_streak = 0
        self.explorer.report_reachable(target_cell)
        waypoint_cells = flood_fill.simplify_path(full_path)
        self.waypoints = [self._cell_to_local(c) for c in waypoint_cells]
        self.waypoint_idx = 0
        self.waypoint_state = 'ROTATE'  # face the first waypoint of the new path before moving
        self.current_target_cell = target_cell
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

        if self.waypoints:
            self._replan_tick += 1
            if self._replan_tick % self.PATH_REPLAN_EVERY_N_TICKS == 0:
                if not self._replan_to_current_target(robot_cell):
                    self.get_logger().info(
                        f'Target unreachable after {self.REPLAN_FAILURE_GRACE} '
                        'consecutive failed replans, picking a new one...'
                    )
                    self.explorer.report_unreachable(self.current_target_cell)
                    self._replan_fail_streak = 0
                    self.waypoints = None
                    self.current_target_cell = None

        if not self.waypoints:
            if not self._plan_next_frontier(robot_cell, pose_yaw):
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

        # ── Rotate-in-place to face the current waypoint before driving ──
        # goal_bl is already expressed in the robot's base frame, so
        # atan2(y, x) directly gives the bearing to the waypoint -- no
        # separate yaw tracking needed. We use this bearing rather than the
        # potential field's force vector because the force also includes
        # obstacle repulsion, which can point away from the waypoint itself;
        # for "always rotate to face the next target first" we want the
        # waypoint's true bearing, not the blended field direction.
        if self.waypoint_state == 'ROTATE':
            bearing = math.atan2(goal_bl[1], goal_bl[0])
            if abs(bearing) <= self.ROTATE_ANGLE_TOLERANCE:
                self.waypoint_state = 'MOVE'  # aligned -- fall through to normal driving below
            else:
                angular_z = self.K_ANGULAR_ROTATE * bearing
                angular_z = max(-self.MAX_ANGULAR, min(self.MAX_ANGULAR, angular_z))
                self.robot.set_velocity(0.0, angular_z)
                self.get_logger().info(
                    f'Rotating to face waypoint {self.waypoint_idx}: '
                    f'bearing={math.degrees(bearing):.1f} deg',
                    throttle_duration_sec=1.0,
                )
                return

        self.planner.set_goal(*goal_bl)

        vx, vy, dist_to_goal, waypoint_reached, reverse = self.planner.potential_field_planner_tick(
            lidar_data, robot_pos=(0.0, 0.0)
        )

        # The planner's own waypoint_reached uses goal_tolerance (0.5m), which is
        # coarser than the spacing between consecutive flood-fill waypoints -- so
        # intermediate waypoints could look "reached" without actually driving
        # there. Use a tighter tolerance for every waypoint except the last one
        # in the list (the actual frontier target), which keeps the planner's
        # own goal_tolerance-based judgement.
        is_final_waypoint = (self.waypoint_idx == len(self.waypoints) - 1)
        if not is_final_waypoint:
            waypoint_reached = dist_to_goal <= self.WAYPOINT_TOLERANCE

        if waypoint_reached:
            self.waypoint_idx += 1
            self.waypoint_state = 'ROTATE'  # face the next waypoint before moving toward it
            if self.waypoint_idx >= len(self.waypoints):
                self.waypoints = None  # reached the frontier -- replan next tick
                self.current_target_cell = None
                self.get_logger().info('Frontier reached, replanning...')
            return

        linear_x, angular_z = self._velocity_to_cmd(vx, vy, reverse)
        self.robot.set_velocity(linear_x, angular_z)

        self.get_logger().info(
            f"wp {self.waypoint_idx}/{len(self.waypoints)} | dist={dist_to_goal:.2f}m | "
            f"v=({vx:.2f},{vy:.2f}) | cmd=({linear_x:.2f},{angular_z:.2f}) | "
            f"{'REV' if reverse else 'fwd'}",
            throttle_duration_sec=1.0,
        )

    # ------------------------------------------------------------------ #
    def _velocity_to_cmd(self, vx: float, vy: float, reverse: bool = False) -> tuple:
        """Convert the planner's raw (vx, vy) into a clamped unicycle command.

        When ``reverse`` is set, the planner has already mirrored (vx, vy)
        into the robot's forward-facing (sensed) cone -- so the correct
        response here is to keep steering toward that same small angle
        (heading doesn't need to change much) but drive with *negative*
        linear speed, backing the robot along the direction its front is
        still pointed. This is what avoids spinning the robot into terrain
        the lidar never saw.
        """
        speed = math.hypot(vx, vy)
        angular_z = math.atan2(vy, max(abs(vx), 1e-3))

        linear_x = -speed if reverse else speed

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
        # rclpy's default SIGINT handler can invalidate the context before
        # this runs, which makes a plain publish() throw
        # "publisher's context is invalid" on Ctrl+C. Guard it so shutdown
        # doesn't crash -- best-effort stop, not a hard requirement.
        if rclpy.ok():
            try:
                node.robot.stop()
            except Exception as exc:
                node.get_logger().warn(f'Could not publish final stop command: {exc}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
