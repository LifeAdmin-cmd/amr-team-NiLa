#!/usr/bin/env python3
"""
flood_fill_planner.py

Flood fill to get a path to the goal, then reduce each grid cell used down
to waypoints: p0 -> p1 -> p2 -> ... If p0 can see p4 with no occupied grid
in between, skip straight to p4 -- one waypoint instead of four.

Grid values in [0.0, 1.0]: 0 = free, 1.0 = obstacle, 0.5 = unexplored
(counted as obstacle too).

Usage
-----
    planner = FloodFillPlanner(grid, obstacle_threshold=0.5)
    path, waypoints = planner.get_waypoints(start=(2, 2), goal=(20, 25))
"""

from __future__ import annotations

from collections import deque
from typing import List, Optional, Tuple

import numpy as np

Cell = Tuple[int, int]  # (row, col)


class FloodFillPlanner:
    """Flood fill global planner over a 2D occupancy grid."""

    def __init__(self, grid: np.ndarray, obstacle_threshold: float = 0.5):
        self.grid = grid
        self.obstacle_threshold = obstacle_threshold
        self.n_rows, self.n_cols = grid.shape

    # ------------------------------------------------------------------ #
    def is_free(self, cell: Cell) -> bool:
        r, c = cell
        if r < 0 or r >= self.n_rows or c < 0 or c >= self.n_cols:
            return False
        return self.grid[r, c] < self.obstacle_threshold

    # ------------------------------------------------------------------ #
    # Flood fill: BFS out from the goal, so every free cell knows its
    # distance to the goal.
    # ------------------------------------------------------------------ #
    def flood_fill(self, goal: Cell) -> np.ndarray:
        """Distance-to-goal for every reachable cell (-1 = unreachable)."""
        dist = np.full((self.n_rows, self.n_cols), -1, dtype=int)

        if not self.is_free(goal):
            return dist

        dist[goal] = 0
        queue = deque([goal])
        neighbors_4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        while queue:
            r, c = queue.popleft()
            for dr, dc in neighbors_4:
                nr, nc = r + dr, c + dc
                if self.is_free((nr, nc)) and dist[nr, nc] == -1:
                    dist[nr, nc] = dist[r, c] + 1
                    queue.append((nr, nc))

        return dist

    # ------------------------------------------------------------------ #
    # Walk downhill from start to goal, one grid cell at a time, always
    # picking the neighbor closer to the goal.
    # ------------------------------------------------------------------ #
    def plan(self, start: Cell, goal: Cell) -> Optional[List[Cell]]:
        """Full grid-cell path from start to goal, or None if unreachable."""
        dist = self.flood_fill(goal)

        if dist[start] == -1:
            return None

        path = [start]
        current = start
        neighbors_4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        while current != goal:
            r, c = current
            best_next = None
            best_dist = dist[r, c]

            for dr, dc in neighbors_4:
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.n_rows and 0 <= nc < self.n_cols:
                    d = dist[nr, nc]
                    if d != -1 and d < best_dist:
                        best_dist = d
                        best_next = (nr, nc)

            if best_next is None:
                return None  # shouldn't happen if dist[start] != -1

            current = best_next
            path.append(current)

        return path

    # ------------------------------------------------------------------ #
    # Can p1 see p2 in a straight line, no occupied grid in between?
    # ------------------------------------------------------------------ #
    def has_line_of_sight(self, p1: Cell, p2: Cell) -> bool:
        """Bresenham line check. A diagonal step needs BOTH corner cells
        free too -- otherwise it's squeezing through a blocked pinch point
        that isn't really open, and the path has to take the L-shape
        detour around it instead.
        """
        r0, c0 = p1
        r1, c1 = p2

        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r0 < r1 else -1
        sc = 1 if c0 < c1 else -1
        err = dr - dc

        r, c = r0, c0
        if not self.is_free((r, c)):
            return False

        while (r, c) != (r1, c1):
            e2 = 2 * err
            step_r, step_c = 0, 0

            if e2 > -dc:
                err -= dc
                step_r = sr
            if e2 < dr:
                err += dr
                step_c = sc

            if step_r != 0 and step_c != 0:
                # diagonal step -- check both corners, not just the target
                if not self.is_free((r + step_r, c)) or not self.is_free((r, c + step_c)):
                    return False

            r += step_r
            c += step_c

            if not self.is_free((r, c)):
                return False

        return True

    # ------------------------------------------------------------------ #
    # p0 -> p1 -> p2 -> p3 -> p4: if p0 sees p4 directly, drop p1-p3 and
    # go straight to p4. Repeat from wherever we land.
    # ------------------------------------------------------------------ #
    def simplify_path(self, path: List[Cell]) -> List[Cell]:
        """Reduce the full grid path down to the fewest waypoints."""
        if len(path) <= 2:
            return list(path)

        waypoints = [path[0]]
        i = 0

        while i < len(path) - 1:
            farthest = i + 1
            for j in range(len(path) - 1, i, -1):
                if self.has_line_of_sight(path[i], path[j]):
                    farthest = j
                    break
            waypoints.append(path[farthest])
            i = farthest

        return waypoints

    # ------------------------------------------------------------------ #
    def get_waypoints(
        self, start: Cell, goal: Cell
    ) -> Tuple[Optional[List[Cell]], Optional[List[Cell]]]:
        """Flood fill -> full path -> reduced waypoints. (None, None) if unreachable."""
        path = self.plan(start, goal)
        if path is None:
            return None, None
        waypoints = self.simplify_path(path)
        return path, waypoints
