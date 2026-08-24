#!/usr/bin/env python3
"""
particle_filter.py

A pure-computation Particle Filter (Monte Carlo Localisation) implementation.
No ROS dependencies, making it easy to test and validate.

Usage
-----
    pf = ParticleFilter(num_particles=500, grid_map=my_grid, resolution=0.1)
    
    # Movement update
    pf.predict(dx, dy, dtheta, noise_std=(0.05, 0.05, 0.02))
    
    # Sensor update
    pf.update(lidar_ranges, lidar_angles)
    
    # Resample
    pf.resample()
    
    # Get estimated pose
    x, y, theta = pf.get_estimated_pose()
"""

import math
import numpy as np
from typing import Tuple, Sequence, Optional


class ParticleFilter:
    """Monte Carlo Localisation (Particle Filter)"""

    def __init__(
        self,
        num_particles: int,
        grid_map: np.ndarray,
        resolution: float,
        origin_x: float = 0.0,
        origin_y: float = 0.0,
        initial_pose: Optional[Tuple[float, float, float]] = None,
        initial_noise: Tuple[float, float, float] = (0.5, 0.5, 0.2)
    ):
        """
        :param num_particles: Number of particles to use.
        :param grid_map: 2D numpy array where 1.0 is an obstacle, 0.0 is free.
        :param resolution: size of each grid cell in meters.
        :param origin_x: The x coordinate of the bottom-left corner of the grid.
        :param origin_y: The y coordinate of the bottom-left corner of the grid.
        :param initial_pose: (x, y, theta) or None (random global distribution).
        :param initial_noise: standard deviations for spreading initial particles.
        """
        self.num_particles = num_particles
        self.grid_map = grid_map
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        
        self.map_rows, self.map_cols = grid_map.shape

        # Particles state: [x, y, theta]
        self.particles = np.zeros((num_particles, 3))
        self.weights = np.ones(num_particles) / num_particles

        if initial_pose is not None:
            # Localized initialization
            self.particles[:, 0] = np.random.normal(initial_pose[0], initial_noise[0], num_particles)
            self.particles[:, 1] = np.random.normal(initial_pose[1], initial_noise[1], num_particles)
            self.particles[:, 2] = np.random.normal(initial_pose[2], initial_noise[2], num_particles)
        else:
            # Global initialization
            map_width = self.map_cols * self.resolution
            map_height = self.map_rows * self.resolution
            self.particles[:, 0] = np.random.uniform(origin_x, origin_x + map_width, num_particles)
            self.particles[:, 1] = np.random.uniform(origin_y, origin_y + map_height, num_particles)
            self.particles[:, 2] = np.random.uniform(-np.pi, np.pi, num_particles)

    def _normalize_angles(self, angles: np.ndarray) -> np.ndarray:
        return (angles + np.pi) % (2 * np.pi) - np.pi

    def predict(self, dx: float, dy: float, dtheta: float, noise_std: Tuple[float, float, float]):
        """
        Motion update (Prediction step).
        Applies odometry change to all particles with added Gaussian noise.
        
        :param dx: Odometry change in X (in the robot's local frame)
        :param dy: Odometry change in Y (in the robot's local frame)
        :param dtheta: Odometry change in heading
        :param noise_std: (std_x, std_y, std_theta) noise to add
        """
        # Noise
        noise_x = np.random.normal(0, noise_std[0], self.num_particles)
        noise_y = np.random.normal(0, noise_std[1], self.num_particles)
        noise_theta = np.random.normal(0, noise_std[2], self.num_particles)

        # Apply motion in the global frame of each particle
        theta = self.particles[:, 2]
        c, s = np.cos(theta), np.sin(theta)
        
        global_dx = (dx + noise_x) * c - (dy + noise_y) * s
        global_dy = (dx + noise_x) * s + (dy + noise_y) * c

        self.particles[:, 0] += global_dx
        self.particles[:, 1] += global_dy
        self.particles[:, 2] += dtheta + noise_theta
        
        self.particles[:, 2] = self._normalize_angles(self.particles[:, 2])

    def _world_to_grid(self, x: np.ndarray, y: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Convert world coordinates (x, y) to grid indices (row, col)"""
        col = np.round((x - self.origin_x) / self.resolution).astype(int)
        row = np.round((y - self.origin_y) / self.resolution).astype(int)
        return row, col

    def update(self, ranges: Sequence[float], angles: Sequence[float], max_range: float = 10.0):
        """
        Sensor update (Measurement step).
        Calculates likelihood of observation and updates weights.
        
        We'll use a simple endpoint model: project the lidar beam, check the grid cell.
        If it lands on an obstacle, high probability. Else low probability.
        """
        valid_idx = np.where((np.array(ranges) > 0.1) & (np.array(ranges) < max_range))[0]
        
        # Subsample rays for speed (e.g. check at most 30 rays)
        step = max(1, len(valid_idx) // 30)
        rays_to_check = valid_idx[::step]

        if len(rays_to_check) == 0:
            return  # No valid readings to update with

        # Measurement model parameters
        z_hit = 0.8
        z_rand = 0.2

        for i in range(self.num_particles):
            px, py, ptheta = self.particles[i]
            
            # If particle is outside the map, zero its weight
            pr, pc = self._world_to_grid(np.array([px]), np.array([py]))
            if pr[0] < 0 or pr[0] >= self.map_rows or pc[0] < 0 or pc[0] >= self.map_cols:
                self.weights[i] = 1e-10
                continue
            
            # Raycast from this particle
            prob_match = 1.0
            
            for idx in rays_to_check:
                r = ranges[idx]
                angle = angles[idx] + ptheta
                
                # End point of the ray
                ex = px + r * math.cos(angle)
                ey = py + r * math.sin(angle)
                
                er, ec = self._world_to_grid(np.array([ex]), np.array([ey]))
                
                # Check if end point is inside grid and is an obstacle
                if 0 <= er[0] < self.map_rows and 0 <= ec[0] < self.map_cols:
                    if self.grid_map[er[0], ec[0]] >= 0.5: # Obstacle
                        prob_match *= z_hit
                    else:
                        prob_match *= z_rand
                else:
                    # Ray goes outside map
                    prob_match *= z_rand
                    
            self.weights[i] *= prob_match

        # Normalize weights
        weight_sum = np.sum(self.weights)
        if weight_sum > 0:
            self.weights /= weight_sum
        else:
            # If all particles have 0 weight, reset uniformly
            self.weights = np.ones(self.num_particles) / self.num_particles

    def resample(self):
        """Low variance resampling (systematic resampling)."""
        cumulative_sum = np.cumsum(self.weights)
        cumulative_sum[-1] = 1.0  # Avoid rounding errors
        
        step = 1.0 / self.num_particles
        r = np.random.uniform(0, step)
        
        new_particles = np.zeros_like(self.particles)
        
        i = 0
        for j in range(self.num_particles):
            U = r + j * step
            while U > cumulative_sum[i]:
                i += 1
            new_particles[j] = self.particles[i]
            
        self.particles = new_particles
        self.weights = np.ones(self.num_particles) / self.num_particles

    def get_estimated_pose(self) -> Tuple[float, float, float]:
        """Returns the weighted average of the particles (x, y, theta)."""
        x = np.sum(self.particles[:, 0] * self.weights)
        y = np.sum(self.particles[:, 1] * self.weights)
        
        # Mean of angles requires circular mean
        sin_sum = np.sum(np.sin(self.particles[:, 2]) * self.weights)
        cos_sum = np.sum(np.cos(self.particles[:, 2]) * self.weights)
        theta = math.atan2(sin_sum, cos_sum)
        
        return x, y, theta

