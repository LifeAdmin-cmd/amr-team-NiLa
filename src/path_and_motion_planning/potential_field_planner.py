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
    vx, vy, dist_to_goal, goal_reached, reverse = pf.potential_field_planner_tick(lidar_data)
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
    enable_reverse : if True, targets in the robot's rear blind spot are
        driven to backward instead of by spinning to face them.
    reverse_angle_threshold : rear cone half-angle [rad] -- match to your
        lidar's actual angular FOV. Anything beyond this angle from
        straight ahead is unsensed, blind territory.
    reverse_hysteresis_margin : extra angle [rad] required to flip the
        forward/reverse mode once set, so a bearing sitting right at the
        threshold (plus TF/goal noise) doesn't flap the robot between
        driving forward and backward every tick.
    """

    ka: float = 0.4
    kr: float = 0.3
    rho0: float = 1.5
    goal_tolerance: float = 0.3
    min_obstacle_range: float = 0.15
    enable_reverse: bool = True
    reverse_angle_threshold: float = math.pi / 2
    reverse_hysteresis_margin: float = math.radians(10)

    goal_x: float = field(default=0.0, init=False)
    goal_y: float = field(default=0.0, init=False)
    goal_theta: Optional[float] = field(default=None, init=False)
    _goal_set: bool = field(default=False, init=False)
    _reverse_mode: bool = field(default=False, init=False)

    # ------------------------------------------------------------------ #
    # Goal management
    # ------------------------------------------------------------------ #
    def set_goal(self, x: float, y: float, theta: Optional[float] = None) -> None:
        """Set/overwrite the current target. ``theta`` = final heading, optional."""
        self.goal_x = x
        self.goal_y = y
        self.goal_theta = theta
        self._goal_set = True
        self._reverse_mode = False  # new goal -- decide drive mode fresh

    def has_goal(self) -> bool:
        return self._goal_set

    # ------------------------------------------------------------------ #
    # Core potential field math
    # ------------------------------------------------------------------ #
    def compute_attractive_force(self, robot_pos: Point) -> Point:
        """Velocity pulling the robot toward the goal (normalized direction x gain)."""
        rx, ry = robot_pos
        dx = self.goal_x - rx
        dy = self.goal_y - ry
        dist = math.hypot(dx, dy)

        if dist < 1e-9:
            return 0.0, 0.0

        fx = self.ka * (dx / dist)
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

    def _decide_reverse(self, goal_vx: float, goal_vy: float) -> bool:
        """Decide forward vs. reverse from the GOAL bearing alone (not the
        noisy combined attractive+repulsive vector) -- repulsion should only
        ever nudge the path sideways, never single-handedly flip the drive
        mode. Sticky with a hysteresis margin so a bearing sitting near
        the threshold doesn't flap the decision tick to tick.
        """
        if not self.enable_reverse:
            self._reverse_mode = False
            return False

        if abs(goal_vx) < 1e-9 and abs(goal_vy) < 1e-9:
            return self._reverse_mode  # goal reached / degenerate -- keep prior mode

        angle = math.atan2(goal_vy, goal_vx)
        abs_angle = abs(angle)

        if self._reverse_mode:
            # Currently reversing -- only go back to forward once well
            # inside the cone (threshold minus margin).
            if abs_angle < self.reverse_angle_threshold - self.reverse_hysteresis_margin:
                self._reverse_mode = False
        else:
            # Currently forward -- only start reversing once well past
            # the cone (threshold plus margin).
            if abs_angle > self.reverse_angle_threshold + self.reverse_hysteresis_margin:
                self._reverse_mode = True

        return self._reverse_mode

    def compute_velocity(
        self, robot_pos: Point, obstacles: Iterable[Point]
    ) -> Tuple[float, float, bool]:
        """Total velocity = attractive + repulsive, resolved for reverse-safe
        driving. Returns (vx, vy, reverse)."""
        vax, vay = self.compute_attractive_force(robot_pos)
        vrx, vry = self.compute_repulsive_force(robot_pos, obstacles)

        total_vx, total_vy = vax + vrx, vay + vry
        reverse = self._decide_reverse(vax, vay)

        if reverse:
            return -total_vx, -total_vy, True
        return total_vx, total_vy, False

    # ------------------------------------------------------------------ #
    # High-level entry point -- the only thing the controller calls
    # ------------------------------------------------------------------ #
    def potential_field_planner_tick(
        self, lidar_data: LidarScan, robot_pos: Point = (0.0, 0.0)
    ) -> Tuple[float, float, float, bool, bool]:
        """One planning step: lidar + robot pos in, raw (vx, vy) velocity
        out. No clamping, no Twist, no kinematics -- that's the controller's
        job.

        Returns (vx, vy, dist_to_goal, goal_reached, reverse).
        ``reverse`` is True when the target direction is in the robot's
        unsensed rear cone; in that case the controller should command a
        *negative* linear speed along (vx, vy) rather than turning to face
        it -- (vx, vy) already gives the small steering correction needed.
        """
        dist_to_goal = math.hypot(self.goal_x - robot_pos[0], self.goal_y - robot_pos[1])

        if dist_to_goal < self.goal_tolerance:
            return 0.0, 0.0, dist_to_goal, True, False

        obstacles = self.scan_to_points(
            lidar_data.ranges,
            lidar_data.angle_min,
            lidar_data.angle_increment,
            max_range=lidar_data.range_max,
        )

        vx, vy, reverse = self.compute_velocity(robot_pos, obstacles)
        return vx, vy, dist_to_goal, False, reverse

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
