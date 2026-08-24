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
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import PoseArray, Pose
from path_and_motion_planning.particle_filter import ParticleFilter

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
    GOAL_LOCAL = (1.0, 0.0)

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
        
        self.map_grid = None
        self.map_resolution = 0.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        
        # QoS-Profil für die Karte (Transient Local) definieren
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Subscriber mit dem neuen QoS-Profil erstellen
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )

        self.particles_pub = self.create_publisher(PoseArray, '/particles', 10)
        self.pf = None
        self.last_odom = None
        
        self.origin = None          
        self.waypoints = None       
        self.waypoint_idx = 0
        self.goal_reached = False

        self.timer = self.create_timer(self.CONTROL_PERIOD, self.control_loop)
        self.get_logger().info('Controller started. Waiting for pose and map...')

    # ------------------------------------------------------------------ #
    def _build_waypoints(self):
        """Flood fill mit der echten Karte."""
        start_map_x = 0.0
        start_map_y = 0.0
        
        goal_map_x = self.GOAL_LOCAL[0]
        goal_map_y = self.GOAL_LOCAL[1]

        start_cell = self._local_to_cell(start_map_x, start_map_y)
        goal_cell = self._local_to_cell(goal_map_x, goal_map_y)

        n_rows, n_cols = self.map_grid.shape

        if not (0 <= start_cell[0] < n_rows and 0 <= start_cell[1] < n_cols):
            self.get_logger().warn(
                f"Start cell {start_cell} out of map bounds ({n_rows}, {n_cols}). Waiting for map...",
                throttle_duration_sec=2.0
            )
            return None

        if not (0 <= goal_cell[0] < n_rows and 0 <= goal_cell[1] < n_cols):
            self.get_logger().warn(
                f"Goal cell {goal_cell} out of map bounds ({n_rows}, {n_cols}). Goal not yet mapped.",
                throttle_duration_sec=2.0
            )
            return None

        # obstacle_threshold=0.65 lässt freie (0.0) und unbekannte Bereiche (0.5) passierbar
        flood_fill = FloodFillPlanner(self.map_grid, obstacle_threshold=0.65, inflation_radius_cells=2)
        _, waypoint_cells = flood_fill.get_waypoints(start_cell, goal_cell)

        if waypoint_cells is None:
            self.get_logger().warn("No path found yet (goal unreachable or obstructed).", throttle_duration_sec=2.0)
            return None

        local_waypoints = []
        for c in waypoint_cells:
            mx, my = self._cell_to_local(c)
            local_waypoints.append((mx - start_map_x, my - start_map_y))

        return local_waypoints

    # def _generate_square_obstacle_grid(self) -> np.ndarray:
    #     """Free everywhere except one square obstacle block."""
    #     grid = np.zeros((self.GRID_SIZE, self.GRID_SIZE))

    #     r_min, c_min = self._local_to_cell(self.OBSTACLE_X_MIN, self.OBSTACLE_Y_MIN)
    #     r_max, c_max = self._local_to_cell(self.OBSTACLE_X_MAX, self.OBSTACLE_Y_MAX)
    #     grid[min(r_min, r_max):max(r_min, r_max) + 1,
    #          min(c_min, c_max):max(c_min, c_max) + 1] = 1.0

    #     return grid

    # ------------------------------------------------------------------ #
    def _cell_to_local(self, cell: tuple) -> tuple:
        row, col = cell
        x = self.map_origin_x + (col + 0.5) * self.map_resolution
        y = self.map_origin_y + (row + 0.5) * self.map_resolution
        return x, y

    def _local_to_cell(self, x: float, y: float) -> tuple:
        col = int(math.floor((x - self.map_origin_x) / self.map_resolution))
        row = int(math.floor((y - self.map_origin_y) / self.map_resolution))
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
            if origin is not None:
                self.origin = origin
                self.get_logger().info(f'Starting pose captured at odom ({origin[0]:.2f}, {origin[1]:.2f})')
        
        if self.origin is None or self.map_grid is None or self.map_resolution <= 0.0:
            self.get_logger().warn('Waiting for starting pose (TF) and /map...', throttle_duration_sec=2.0)
            return

        current_odom = self.robot.get_robot_pose_in_odom()
        if current_odom is not None and self.last_odom is not None:
            # Aktualisiere die Partikel basierend auf der Bewegung
            self.pf.sample_motion_model_odometry(self.last_odom, current_odom)
            self.last_odom = current_odom
            self._publish_particles()

        if self.waypoints is None:
            self.waypoints = self._build_waypoints()
            if self.waypoints is None:
                # Warten auf größeres Karten-Update von SLAM
                return
            self.get_logger().info(f'{len(self.waypoints)} waypoints planned based on real map.')

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
    
    # ------------------------------------------------------------------ #
    def map_callback(self, msg: OccupancyGrid):
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

        width = msg.info.width
        height = msg.info.height

        data = np.array(msg.data, dtype=np.float32)
        data = np.where(data == -1, 50.0, data)
        data = data / 100.0

        # In ROS OccupancyGrid: data ist Zeile für Zeile gespeichert (height = rows, width = cols)
        self.map_grid = data.reshape((height, width))
        self.get_logger().info(f'Map received: {width}x{height} cells, resolution {self.map_resolution}m')

    
    def _publish_particles(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        
        for p in self.pf.particles:
            pose = Pose()
            pose.position.x = float(p[0])
            pose.position.y = float(p[1])
            # Umrechnung von Gierwinkel (Yaw) in Quaternion für RViz
            pose.orientation.z = math.sin(p[2] / 2.0)
            pose.orientation.w = math.cos(p[2] / 2.0)
            msg.poses.append(pose)
            
        self.particles_pub.publish(msg)


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
