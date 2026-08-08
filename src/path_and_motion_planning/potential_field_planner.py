#!/usr/bin/env python3
"""
potential_field_planner.py

Pure-computation Artificial Potential Field (APF) planner. No ROS
dependencies -- takes a goal + lidar reading, returns a velocity vector.

Usage
-----
    pf = PotentialFieldPlanner(ka=0.4, kr=0.3, rho0=1.5)
    pf.set_goal(4.0, 10.0)  # in base_link frame

    lidar_data = LidarScan(msg.ranges, msg.angle_min, msg.angle_increment)
    vx, vy, dist_to_goal, goal_reached = pf.potential_field_planner_tick(lidar_data)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Sequence, Tuple

Point = Tuple[float, float]


@dataclass
class LidarScan:
    """A single lidar reading -- just the fields the planner needs from a
    ``sensor_msgs/msg/LaserScan``."""

    ranges: Sequence[float]
    angle_min: float
    angle_increment: float
    range_max: Optional[float] = None


@dataclass
class PotentialFieldPlanner:
    """Artificial Potential Field local planner.

    Expects the goal and obstacles in the robot's own frame (robot at the
    origin) -- transform into base_link before calling.

    Parameters
    ----------
    ka : attractive gain -- pull strength toward the goal.
    kr : repulsive gain -- push strength away from obstacles.
    rho0 : obstacle influence radius [m]; farther obstacles are ignored.
    goal_tolerance : distance [m] to count the goal as reached.
    min_obstacle_range : ignore hits closer than this [m] (sensor noise).
    attractive_shape : "conic" (constant pull) or "quadratic" (pull scales
        with distance -- smoother near closely-spaced waypoints).
    """

    ka: float = 0.4
    kr: float = 0.3
    rho0: float = 1.5
    goal_tolerance: float = 0.3
    min_obstacle_range: float = 0.15
    attractive_shape: str = "conic"  # "conic" | "quadratic"

    goal_x: float = field(default=0.0, init=False)
    goal_y: float = field(default=0.0, init=False)
    goal_theta: Optional[float] = field(default=None, init=False)
    _goal_set: bool = field(default=False, init=False)

    # ------------------------------------------------------------------ #
    # Goal management
    # ------------------------------------------------------------------ #
    def set_goal(self, x: float, y: float, theta: Optional[float] = None) -> None:
        """Set/overwrite the current target. ``theta`` = final heading, optional."""
        self.goal_x = x
        self.goal_y = y
        self.goal_theta = theta
        self._goal_set = True

    def has_goal(self) -> bool:
        return self._goal_set

    # ------------------------------------------------------------------ #
    # Core potential field math
    # ------------------------------------------------------------------ #
    def compute_attractive_force(self, robot_pos: Point) -> Point:
        """Velocity pulling the robot toward the goal."""
        rx, ry = robot_pos
        dx = self.goal_x - rx
        dy = self.goal_y - ry
        dist = math.hypot(dx, dy)

        if dist < 1e-9:
            return 0.0, 0.0

        if self.attractive_shape == "quadratic":
            fx = self.ka * dx  # scales with distance -> smooth near goal
            fy = self.ka * dy
        else:
            fx = self.ka * (dx / dist)  # constant-magnitude pull
            fy = self.ka * (dy / dist)

        return fx, fy

    def compute_repulsive_force(
        self, robot_pos: Point, obstacles: Iterable[Point]
    ) -> Point:
        """Velocity pushing the robot away from nearby obstacles."""
        rx, ry = robot_pos
        frx, fry = 0.0, 0.0

        for ox, oy in obstacles:
            dx = ox - rx
            dy = oy - ry
            r = math.hypot(dx, dy)

            if r < self.min_obstacle_range or r > self.rho0 or r < 1e-9:
                continue  # out of range or degenerate

            factor = self.kr * (1.0 / r - 1.0 / self.rho0) / (r ** 2)
            frx += factor * (-dx / r)  # push away from obstacle
            fry += factor * (-dy / r)

        return frx, fry

    def compute_velocity(
        self, robot_pos: Point, obstacles: Iterable[Point]
    ) -> Tuple[float, float]:
        """Total velocity = attractive + repulsive."""
        vax, vay = self.compute_attractive_force(robot_pos)
        vrx, vry = self.compute_repulsive_force(robot_pos, obstacles)
        return vax + vrx, vay + vry

    # ------------------------------------------------------------------ #
    # High-level entry point -- the only thing the controller calls
    # ------------------------------------------------------------------ #
    def potential_field_planner_tick(
        self, lidar_data: LidarScan, robot_pos: Point = (0.0, 0.0)
    ) -> Tuple[float, float, float, bool]:
        """One planning step: lidar + robot pos in, raw (vx, vy) velocity
        out. No clamping, no Twist, no kinematics -- that's the controller's
        job.

        Returns (vx, vy, dist_to_goal, goal_reached).
        """
        dist_to_goal = math.hypot(self.goal_x - robot_pos[0], self.goal_y - robot_pos[1])

        if dist_to_goal < self.goal_tolerance:
            return 0.0, 0.0, dist_to_goal, True

        obstacles = self.scan_to_points(
            lidar_data.ranges,
            lidar_data.angle_min,
            lidar_data.angle_increment,
            max_range=lidar_data.range_max,
        )

        vx, vy = self.compute_velocity(robot_pos, obstacles)
        return vx, vy, dist_to_goal, False

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #
    def scan_to_points(
        self,
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        max_range: Optional[float] = None,
    ) -> List[Point]:
        """Convert raw scan ranges into (x, y) obstacle points, filtering
        out invalid/out-of-range hits."""
        rho_limit = self.rho0 if max_range is None else max_range
        points: List[Point] = []

        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r < self.min_obstacle_range or r > rho_limit:
                continue
            angle = angle_min + i * angle_increment
            points.append((r * math.cos(angle), r * math.sin(angle)))

        return points

    def is_goal_reached(self, robot_pos: Point) -> bool:
        dist = math.hypot(self.goal_x - robot_pos[0], self.goal_y - robot_pos[1])
        return dist < self.goal_tolerance

