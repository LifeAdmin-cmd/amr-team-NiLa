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
    target = explorer.select_target(grid, dist_from_robot)

    if target is None:
        # no reachable frontiers left -- exploration complete
        ...
"""

from __future__ import annotations

from typing import List, Optional, Set, Tuple

import numpy as np

Cell = Tuple[int, int]  # (row, col)


class FrontierExplorer:
    """Frontier-based exploration: always head for the closest reachable
    unexplored boundary. Frontiers that turn out to be unreachable are
    blacklisted so they aren't retried -- effectively marked as explored."""

    def __init__(
        self,
        free_threshold: float = 0.3,
        occ_threshold: float = 0.7,
        min_frontier_distance_cells: int = 5,
    ):
        """
        :param free_threshold: grid value below which a cell counts as free.
        :param occ_threshold: grid value above which a cell counts as occupied.
                               Anything in between is unknown.
        :param min_frontier_distance_cells: skip candidates closer than this
            (in cells) to the robot -- too close to actually drive to before
            the potential field planner already reports "reached".
        """
        self.free_threshold = free_threshold
        self.occ_threshold = occ_threshold
        self.min_frontier_distance_cells = min_frontier_distance_cells
        self.blacklist: Set[Cell] = set()

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

    def select_target(self, grid: np.ndarray, dist_from_robot: np.ndarray) -> Optional[Cell]:
        """Closest reachable frontier cell, or None if exploration is complete.

        :param grid: the occupancy probability grid.
        :param dist_from_robot: flood-fill distance grid rooted at the robot's
            current cell (e.g. FloodFillPlanner.flood_fill(robot_cell)).
            -1 means unreachable.
        """
        frontiers = self.find_frontiers(grid)

        reachable = []
        for cell in frontiers:
            d = dist_from_robot[cell]
            if d == -1:
                self.blacklist.add(cell)  # unreachable -- treat as explored
            elif d < self.min_frontier_distance_cells:
                continue  # too close to be worth driving to
            else:
                reachable.append((d, cell))

        if not reachable:
            return None

        reachable.sort(key=lambda entry: entry[0])
        return reachable[0][1]

    def is_exploration_complete(self, grid: np.ndarray) -> bool:
        """True once no unexplored frontier remains."""
        return len(self.find_frontiers(grid)) == 0
