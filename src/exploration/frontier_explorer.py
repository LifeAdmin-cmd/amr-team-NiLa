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
    def __init__(
        self,
        free_threshold: float = 0.3,
        occ_threshold: float = 0.7,
        min_frontier_distance_cells: int = 5,
        switch_margin_cells: float = 3.0,
        unreachable_confirm_count: int = 3,
    ):
        """
        :param switch_margin_cells: only abandon the current target for a new
            one if the new one is at least this many cells closer. Prevents
            noisy flood-fill distances from causing target flip-flop.
        :param unreachable_confirm_count: number of consecutive "-1" (unreachable)
            readings required before a cell is blacklisted. Guards against a
            single noisy flood-fill pass wrongly killing a valid frontier.
        """
        self.free_threshold = free_threshold
        self.occ_threshold = occ_threshold
        self.min_frontier_distance_cells = min_frontier_distance_cells
        self.switch_margin_cells = switch_margin_cells
        self.unreachable_confirm_count = unreachable_confirm_count

        self.blacklist: Set[Cell] = set()
        self._unreachable_strikes: dict[Cell, int] = {}
        self.current_target: Optional[Cell] = None

    def _is_unknown(self, value: float) -> bool:
        return self.free_threshold <= value <= self.occ_threshold

    def find_frontiers(self, grid: np.ndarray) -> List[Cell]:
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
        frontiers = self.find_frontiers(grid)
        frontier_set = set(frontiers)

        reachable = []
        for cell in frontiers:
            d = dist_from_robot[cell]
            if d == -1:
                strikes = self._unreachable_strikes.get(cell, 0) + 1
                self._unreachable_strikes[cell] = strikes
                if strikes >= self.unreachable_confirm_count:
                    self.blacklist.add(cell)
                    self._unreachable_strikes.pop(cell, None)
                continue
            else:
                # cell reported reachable this pass -- reset its strike count
                self._unreachable_strikes.pop(cell, None)
                if d < self.min_frontier_distance_cells:
                    continue
                reachable.append((d, cell))

        if not reachable:
            self.current_target = None
            return None

        reachable.sort(key=lambda entry: entry[0])
        best_dist, best_cell = reachable[0]

        # Hysteresis: stick with the current target unless it's no longer a
        # valid frontier, or a new candidate is meaningfully closer. This is
        # what stops sensor-noise-driven distance jitter from causing
        # target flip-flop while the old target is still perfectly reachable.
        if self.current_target is not None and self.current_target in frontier_set:
            current_dist = dist_from_robot[self.current_target]
            if current_dist != -1 and current_dist >= self.min_frontier_distance_cells:
                if best_dist >= current_dist - self.switch_margin_cells:
                    return self.current_target

        self.current_target = best_cell
        return best_cell

    def is_exploration_complete(self, grid: np.ndarray) -> bool:
        return len(self.find_frontiers(grid)) == 0
