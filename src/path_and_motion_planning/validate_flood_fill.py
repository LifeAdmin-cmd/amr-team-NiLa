#!/usr/bin/env python3
"""
validate_flood_fill.py

Standalone validation: feed a map (numpy occupancy grid) + start/goal into
FloodFillPlanner, and get a drawing of the full path and the reduced
waypoints, saved as a PNG.

Usage
-----
    from validate_flood_fill import visualize

    grid = np.zeros((30, 30))
    grid[10, 5:25] = 1.0  # a wall
    visualize(grid, start=(2, 2), goal=(25, 25), save_path="result.png")

Run directly for a built-in demo map:
    python3 validate_flood_fill.py
"""

import numpy as np
import matplotlib.pyplot as plt

from flood_fill_planner import FloodFillPlanner


def visualize(grid: np.ndarray, start, goal, save_path: str = "flood_fill_result.png"):
    planner = FloodFillPlanner(grid, obstacle_threshold=0.5, inflation_radius_cells=3)
    path, waypoints = planner.get_waypoints(start, goal)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(grid, cmap="Greys", vmin=0.0, vmax=1.0, origin="upper")

    if path is None:
        ax.set_title("No path found (goal unreachable)")
    else:
        path_rows = [p[0] for p in path]
        path_cols = [p[1] for p in path]
        ax.plot(path_cols, path_rows, "-", color="deepskyblue", linewidth=1.5,
                label=f"Full path ({len(path)} cells)")

        wp_rows = [w[0] for w in waypoints]
        wp_cols = [w[1] for w in waypoints]
        ax.plot(wp_cols, wp_rows, "-o", color="orange", linewidth=2, markersize=8,
                label=f"Waypoints ({len(waypoints)})")

        for i, (r, c) in enumerate(waypoints):
            ax.annotate(f"p{i}", (c, r), textcoords="offset points",
                        xytext=(6, 6), fontsize=9, color="darkorange")

        ax.set_title(f"Flood fill path: {len(path)} cells -> {len(waypoints)} waypoints")

    ax.plot(start[1], start[0], "gs", markersize=12, label="start")
    ax.plot(goal[1], goal[0], "r*", markersize=16, label="goal")
    ax.legend(loc="upper left", fontsize=9)
    ax.set_xlabel("col")
    ax.set_ylabel("row")

    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)

    print(f"Saved: {save_path}")
    if path is not None:
        print(f"Full path length: {len(path)} cells")
        print(f"Waypoints ({len(waypoints)}): {waypoints}")
    else:
        print("Goal unreachable from start.")

    return path, waypoints


def _demo_map() -> np.ndarray:
    """A small maze-like map to exercise flood fill + waypoint reduction."""
    grid = np.zeros((30, 30))

    grid[:, :] = np.where(np.random.rand(30, 30) < 0.02, 0.5, grid)  # sprinkle "unexplored"

    # Outer walls
    grid[0, :] = 1.0
    grid[-1, :] = 1.0
    grid[:, 0] = 1.0
    grid[:, -1] = 1.0

    # Internal walls spanning full width, each with one gap to force a detour
    grid[8, 1:29] = 1.0
    grid[8, 25] = 0.0  # gap near the right

    grid[15, 1:29] = 1.0
    grid[15, 5] = 0.0  # gap near the left

    grid[22, 1:29] = 1.0
    grid[22, 20] = 0.0  # gap near the right-middle

    return grid


if __name__ == "__main__":
    np.random.seed(0)
    grid = _demo_map()
    visualize(grid, start=(2, 2), goal=(27, 27), save_path="flood_fill_demo.png")
