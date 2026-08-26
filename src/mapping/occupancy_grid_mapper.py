#!/usr/bin/env python3
"""
occupancy_grid_mapper.py

A pure-computation 2D Occupancy Grid Mapper using log-odds updates.
No ROS dependencies, making it easy to test and validate.

Grid values in [0.0, 1.0]: 0.5 = unvisited/unknown, 1.0 = occupied, 0.0 = free.

Usage
-----
    mapper = OccupancyGridMapper(size_m=20.0, resolution=0.1)
    mapper.update(robot_x, robot_y, robot_yaw, ranges, angle_min, angle_increment, range_max)
    prob_grid = mapper.get_probability_grid()
"""

import math
from typing import List, Sequence, Tuple

import numpy as np


class OccupancyGridMapper:
    """Log-odds occupancy grid mapper."""

    def __init__(
        self,
        size_m: float = 20.0,
        resolution: float = 0.1,
        p_occ: float = 0.1,
        p_free: float = 0.8,
        l_min: float = -10.0,
        l_max: float = 10.0,
        bound_min: float = -10.0,
        bound_max: float = 10.0,
    ):
        """
        :param size_m: side length of the (square) map in meters.
        :param resolution: size of each grid cell in meters.
        :param p_occ: inverse sensor model probability for occupied cells.
        :param p_free: inverse sensor model probability for free cells.
        :param l_min: clamp for accumulated log-odds (avoid overflow/drift).
        :param l_max: clamp for accumulated log-odds.
        :param bound_min: lower x/y bound (meters) around (0, 0). Cells outside
            [bound_min, bound_max] always read as occupied, regardless of sensor data.
        :param bound_max: upper x/y bound (meters) around (0, 0).
        """
        self.resolution = resolution
        self.size_cells = int(size_m / resolution)
        self.bound_min = bound_min
        self.bound_max = bound_max

        self.l_occ = math.log(p_occ / (1.0 - p_occ))
        self.l_free = math.log(p_free / (1.0 - p_free))
        self.l_min = l_min
        self.l_max = l_max

        # Log-odds grid, 0.0 = prior (p=0.5)
        self.log_odds = np.zeros((self.size_cells, self.size_cells), dtype=np.float32)

        # Map centered on the robot's starting pose
        self.origin_x = -size_m / 2.0
        self.origin_y = -size_m / 2.0

        self._out_of_bounds = self._compute_out_of_bounds_mask()

    def _compute_out_of_bounds_mask(self) -> np.ndarray:
        """Boolean mask of cells outside [bound_min, bound_max] on either axis."""
        rows, cols = np.indices((self.size_cells, self.size_cells))
        xs = self.origin_x + cols * self.resolution
        ys = self.origin_y + rows * self.resolution
        return (xs < self.bound_min) | (xs > self.bound_max) | (ys < self.bound_min) | (ys > self.bound_max)

    def _world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        return row, col

    def _in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.size_cells and 0 <= col < self.size_cells

    @staticmethod
    def _bresenham(r0: int, c0: int, r1: int, c1: int) -> List[Tuple[int, int]]:
        """Cells along the line from (r0, c0) to (r1, c1), excluding the endpoint."""
        cells = []
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r0 < r1 else -1
        sc = 1 if c0 < c1 else -1
        err = dr - dc

        r, c = r0, c0
        while (r, c) != (r1, c1):
            cells.append((r, c))
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r += sr
            if e2 < dr:
                err += dr
                c += sc

        return cells

    def update(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        range_min: float = 0.0,
        range_max: float = 10.0,
    ):
        """Inverse sensor model update from one laser scan at the given pose."""
        r_robot, c_robot = self._world_to_cell(robot_x, robot_y)

        free_cells = set()
        occ_cells = set()

        for i, r in enumerate(ranges):
            angle = robot_yaw + angle_min + i * angle_increment

            if math.isfinite(r) and range_min <= r <= range_max:
                hit = True
                end_x = robot_x + r * math.cos(angle)
                end_y = robot_y + r * math.sin(angle)
            else:
                hit = False
                end_x = robot_x + range_max * math.cos(angle)
                end_y = robot_y + range_max * math.sin(angle)

            r_end, c_end = self._world_to_cell(end_x, end_y)

            for cell in self._bresenham(r_robot, c_robot, r_end, c_end):
                if self._in_bounds(*cell):
                    free_cells.add(cell)

            if hit and self._in_bounds(r_end, c_end):
                occ_cells.add((r_end, c_end))

        # Free cells first, then occupied, to handle partially-occupied cells
        for row, col in free_cells:
            self.log_odds[row, col] = np.clip(
                self.log_odds[row, col] + self.l_free, self.l_min, self.l_max
            )
        for row, col in occ_cells:
            self.log_odds[row, col] = np.clip(
                self.log_odds[row, col] + self.l_occ, self.l_min, self.l_max
            )

    def get_probability_grid(self) -> np.ndarray:
        """Log-odds -> probability of occupancy, in [0, 1]."""
        prob_free = 1.0 / (1.0 + np.exp(-self.log_odds))
        grid = 1.0 - prob_free
        grid[self._out_of_bounds] = 1.0
        return grid
