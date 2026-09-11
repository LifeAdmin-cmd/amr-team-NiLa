#!/usr/bin/env python3
"""
frontier_explorer.py

A pure-computation Frontier Search implementation. Frontiers are free cells
next to at least one unknown cell -- the boundary between explored and
unexplored space. No ROS dependencies, making it easy to test and validate.

Works directly on the occupancy grid convention from OccupancyGridMapper:
0.0 = free, 1.0 = occupied, 0.5 = unknown.

Usage
-----
    explorer = FrontierExplorer()

    dist_from_robot = flood_fill_planner.flood_fill(robot_cell)
    target = explorer.select_target(grid, dist_from_robot, robot_cell, robot_heading)

    if target is None:
        # no reachable frontiers left -- exploration complete
        ...
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

Cell = Tuple[int, int]  # (row, col)
HEADING_OFFSET = (math.pi/2)


class FrontierExplorer:
    """Frontier-based exploration: always head for the closest reachable
    unexplored boundary that doesn't require spinning into the robot's
    blind spot. Frontiers that turn out to be unreachable are blacklisted
    (after repeated confirmation) so they aren't retried -- effectively
    marked as explored."""

    def __init__(
        self,
        free_threshold: float = 0.3,
        occ_threshold: float = 0.7,
        min_frontier_distance_cells: int = 5,
        switch_margin_cells: float = 3.0,
        unreachable_confirm_count: int = 3,
        max_heading_change: float = math.pi / 4,
    ):
        """
        :param free_threshold: grid value below which a cell counts as free.
        :param occ_threshold: grid value above which a cell counts as occupied.
                               Anything in between is unknown.
        :param min_frontier_distance_cells: skip candidates closer than this
            (in cells) to the robot -- too close to actually drive to before
            the potential field planner already reports "reached".
        :param switch_margin_cells: only abandon the current target for a new
            one if the new one is at least this many cells closer. Prevents
            noisy flood-fill distances from causing target flip-flop.
        :param unreachable_confirm_count: number of consecutive "unreachable"
            observations required before a cell is blacklisted. Guards
            against a single noisy reading (flood-fill -1, or an external
            failed-plan report) wrongly killing a valid frontier.
        :param max_heading_change: largest bearing change [rad] from the
            robot's current heading a target is allowed to require. Keeps
            the robot from being sent to a frontier it would have to spin
            around to face -- it has no sensor coverage back there. Default
            pi/4 (45 deg). Candidates outside this cone are only used as a
            last resort if nothing inside the cone is reachable, so
            exploration never stalls completely.
        """
        self.free_threshold = free_threshold
        self.occ_threshold = occ_threshold
        self.min_frontier_distance_cells = min_frontier_distance_cells
        self.switch_margin_cells = switch_margin_cells
        self.unreachable_confirm_count = unreachable_confirm_count
        self.max_heading_change = max_heading_change

        self.blacklist: Set[Cell] = set()
        self._unreachable_strikes: Dict[Cell, int] = {}
        self._missing_frontier_strikes: Dict[Cell, int] = {}
        self.current_target: Optional[Cell] = None

    def _is_unknown(self, value: float) -> bool:
        return self.free_threshold <= value <= self.occ_threshold

    def find_frontiers(self, grid: np.ndarray) -> List[Cell]:
        """Free cells with at least one unknown neighbor, excluding blacklisted ones."""
        n_rows, n_cols = grid.shape
        neighbors_4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        frontiers = []

        free_rows, free_cols = np.where(grid < self.free_threshold)
        for r, c in zip(free_rows, free_cols):
            cell = (int(r), int(c))
            if cell in self.blacklist:
                continue
            for dr, dc in neighbors_4:
                nr, nc = r + dr, c + dc
                if 0 <= nr < n_rows and 0 <= nc < n_cols and self._is_unknown(grid[nr, nc]):
                    frontiers.append(cell)
                    break

        return frontiers

    # ------------------------------------------------------------------ #
    # Debounced reachability reporting -- used both internally (flood-fill
    # distance == -1) and externally (controller reporting a failed
    # flood_fill.plan() against this target).
    # ------------------------------------------------------------------ #
    def report_unreachable(self, cell: Cell) -> bool:
        """Record one unreachable observation for ``cell``. Returns True if
        this pushed it over the confirmation threshold and it is now
        blacklisted, False if it's just noted and still considered valid."""
        strikes = self._unreachable_strikes.get(cell, 0) + 1
        self._unreachable_strikes[cell] = strikes
        if strikes >= self.unreachable_confirm_count:
            self.blacklist.add(cell)
            self._unreachable_strikes.pop(cell, None)
            if self.current_target == cell:
                self.current_target = None
            return True
        return False

    def report_reachable(self, cell: Cell) -> None:
        """Clear any accumulated unreachable strikes for ``cell`` -- call
        this when a plan to it just succeeded, so a single earlier hiccup
        doesn't linger and combine with a later one."""
        self._unreachable_strikes.pop(cell, None)

    # ------------------------------------------------------------------ #
    # Heading helpers
    # ------------------------------------------------------------------ #
    def _bearing_to(self, robot_cell: Cell, target_cell: Cell) -> float:
        """Bearing from robot to target in the grid plane.

        Assumes col increases along the same axis as robot_heading's zero
        direction and row increases as heading rotates toward +pi/2. If your
        grid-to-world transform differs, adjust this mapping to match --
        the constraint only works if this bearing is expressed in the same
        frame as robot_heading.
        """
        dr = target_cell[0] - robot_cell[0]
        dc = target_cell[1] - robot_cell[1]
        return math.atan2(dr, dc)

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """Smallest signed difference a - b, wrapped to [-pi, pi]."""
        return (a - b + math.pi) % (2 * math.pi) - math.pi

    def _required_turn(self, robot_cell: Cell, robot_heading: float, target_cell: Cell) -> float:
        """Absolute heading change [rad] needed to face target_cell."""
        bearing = self._bearing_to(robot_cell, target_cell)
        return abs(self._angle_diff(bearing, robot_heading))

    # ------------------------------------------------------------------ #
    # Target selection
    # ------------------------------------------------------------------ #
    def select_target(
        self,
        grid: np.ndarray,
        dist_from_robot: np.ndarray,
        robot_cell: Cell,
        robot_heading: float,
    ) -> Optional[Cell]:
        """Closest reachable frontier cell that doesn't require a turn
        sharper than ``max_heading_change``, or None if exploration is
        complete.

        :param grid: the occupancy probability grid.
        :param dist_from_robot: flood-fill distance grid rooted at the robot's
            current cell (e.g. FloodFillPlanner.flood_fill(robot_cell)).
            -1 means unreachable.
        :param robot_cell: robot's current (row, col), used to compute
            bearing to each candidate frontier.
        :param robot_heading: robot's current heading [rad], same frame as
            ``_bearing_to``.
        """
        frontiers = self.find_frontiers(grid)
        frontier_set = set(frontiers)
        #robot heading offset this is importnat for robil4!!!
        robot_heading += HEADING_OFFSET
        
        in_cone: List[Tuple[float, Cell]] = []
        out_of_cone: List[Tuple[float, Cell]] = []

        for cell in frontiers:
            d = dist_from_robot[cell]
            if d == -1:
                self.report_unreachable(cell)
                continue

            self.report_reachable(cell)
            if d < self.min_frontier_distance_cells:
                continue  # too close to be worth driving to

            turn = self._required_turn(robot_cell, robot_heading, cell)
            if turn <= self.max_heading_change:
                in_cone.append((d, cell))
            else:
                out_of_cone.append((d, cell))

        # Prefer candidates that don't require a sharp turn. Only fall back
        # to a wider-turn candidate if nothing within the cone is reachable,
        # so exploration can still make progress instead of stalling.
        reachable = in_cone if in_cone else out_of_cone

        # Grace period on frontier-membership: a cell dropping out of the
        # frontier set for one tick (occupancy-probability flicker near the
        # free/unknown threshold) shouldn't immediately force reselection.
        # Only actually give up on it after several consecutive misses.
        current_still_valid = False
        if self.current_target is not None:
            if self.current_target in frontier_set:
                self._missing_frontier_strikes.pop(self.current_target, None)
                current_still_valid = True
            elif self.current_target not in self.blacklist:
                strikes = self._missing_frontier_strikes.get(self.current_target, 0) + 1
                self._missing_frontier_strikes[self.current_target] = strikes
                if strikes < self.unreachable_confirm_count:
                    current_still_valid = True  # treat as a blip, keep it
                else:
                    self._missing_frontier_strikes.pop(self.current_target, None)

        if not reachable:
            if not current_still_valid:
                self.current_target = None
            return self.current_target

        reachable.sort(key=lambda entry: entry[0])
        best_dist, best_cell = reachable[0]

        # Hysteresis: stick with the current target unless it's no longer a
        # valid frontier, or a new candidate is meaningfully closer, or it
        # now requires too sharp a turn. This is what stops sensor-noise-
        # driven distance/heading jitter from causing target flip-flop
        # while the old target is still perfectly reachable.
        if current_still_valid:
            current_dist = dist_from_robot[self.current_target]
            current_turn_ok = (
                self._required_turn(robot_cell, robot_heading, self.current_target)
                <= self.max_heading_change
            )
            if (
                current_dist != -1
                and current_dist >= self.min_frontier_distance_cells
                and current_turn_ok
            ):
                if best_dist >= current_dist - self.switch_margin_cells:
                    return self.current_target

        self.current_target = best_cell
        return best_cell

    def is_exploration_complete(self, grid: np.ndarray) -> bool:
        """True once no unexplored frontier remains."""
        return len(self.find_frontiers(grid)) == 0
