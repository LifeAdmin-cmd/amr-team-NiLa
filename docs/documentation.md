# AMR Final Project: Robile Deployment

## Project Structure & Setup

### Project Overview
The goal of this project is to deploy path planning, localisation, and SLAM-based environment exploration on the Robile platform. We aim to transition these capabilities from a simulated environment to the physical robot, addressing real-world challenges along the way.

### Prerequisites & Network Setup
To run this project, ensure you have the following installed and configured:
- **Dependencies**: ROS 2 installed as per the course material.
- **Environment Variables**: Configure your ROS 2 environment, particularly `ROS_DOMAIN_ID`, to ensure proper communication between nodes and avoid interference with other teams. Set this in the provided `env.bat` script.
- **Network Configuration**: The system requires the generation of a FastRTPS XML profile for proper DDS communication over the network, especially when interfacing with the real Robile.

### Execution Instructions
1. Make the scripts executable (Linux/macOS):
   ```bash
   chmod +x env.bat launch_sim.bat robile_connection.bat
   ```
2. Adjust paths and environment variables (like `ROS_DOMAIN_ID`) in `env.bat` if necessary.

**Running the Simulation:**
To launch the Gazebo simulation with the Robile, run:
```bash
./launch_sim.bat
```

**Running on the Physical Robot:**
To connect to the physical Robile and manage its operations, use the provided menu-driven script for SSH access and teleoperation:
```bash
./robile_connection.bat
```

### Repository Management
- **Single Branch**: All code and documentation for this project are maintained on a single main branch.
- **.gitignore**: We utilize a `.gitignore` file to ensure that only necessary files are committed, preventing the upload of large, unnecessary folders (like build artifacts or unneeded data).

---

## Core Task Implementation

### 1. Path and Motion Planning
#### Motion planning as done in the assignments -> attraction based
Potential field planner from the assignments (`src/path_and_motion_planning/potential_field_planner.py`: `PotentialFieldPlanner`), ported as-is and wrapped into a python class for ease of use.
#### Flood fill with line of sight
Global planner on top (`src/path_and_motion_planning/flood_fill_planner.py`: `FloodFillPlanner`), for large maps where a single potential field goal gets stuck:
1. **Flood fill**: BFS from the goal gives every free cell a distance-to-goal value.
2. **Greedy descent**: from the start, always step to the neighbor closer to the goal -> full grid path.
3. **Waypoint reduction**: If full path is `p0 -> p1 -> p2 -> p3 -> p4 -> ...`. If `p0` can see `p4` in a straight line (no occupied cells), skip `p1`-`p3` and go straight `p0 -> p4`. Repeat from there. Diagonal cuts are blocked if either corner cell is occupied - that's a pinch point robot may not fits through, so the path takes the L-shape detour instead.
These waypoints feed one at a time into the potential field planner.
<img src="img/flood_fill_demo.png" width="600"/>

### First path and motion planning validation
Quick end-to-end validation before continuing: the planned path/waypoints (left) next to the actual robot driving them in Gazebo (right). Note the robot's position at controller start is treated as (0,0) with heading 0, so there's a slight offset/flip relative to the planning image, but it drives the waypoints correctly regardless - some fine-tuning likely still needed.  
<img src="img/square_path_validation.gif" width="600"/>

Note: for this square obstacle it looks like the path pinches the corner, but it actually just barely misses it. In real application the potential field planner takes care of the fine-grained corner avoidance, since the waypoint only marks the rough direction to head in.

> **Subject to change**: one thing still to be added: a minimum number of free grid squares to be considered passable for the robot when planning the path (a single free cell might not be enough clearance given the robot's actual size). Easiest fix here is to group occupancy cells together into robot-sized chunks before running flood fill. Currently this is not addressed due to the fact that we dont have propper map building yet for further testing. Only the prof of concept testing.

### Obstacle Inflation (Configuration Space / C-Space)

To prevent the robot from getting stuck on corners or choosing passages it cannot physically fit through, **obstacle inflation** has been integrated into the global planner. This solves the issue of treating the robot as a point mass during planning, allowing it to be considered with its actual physical dimensions instead.

* **How it works:** The map is pre-processed before the flood-fill algorithm is executed. Every grid cell marked as an obstacle is artificially "inflated" by a defined radius. By calculating the Euclidean distance, a circular buffer is created around the obstacles. The planner then searches for its path within this safe Configuration Space (C-Space).
* **Configuration:** The safety margin can be configured during the initialization of the `FloodFillPlanner` using the `inflation_radius_cells` parameter. Given a grid resolution of 0.1 m per cell, a value of `3` corresponds to a 30 cm buffer, which serves as a solid default for the Robile.


<img src="img/flood_fill_demo_inflated.png" width="600"/>

*Figure: The planner correctly identifies that the goal is unreachable because the narrow gaps in the walls (1 cell) are completely closed off by the robot's inflation radius (3 cells) in the Configuration Space.*

---

### 2. Localisation (Monte Carlo Localisation)

To accurately track the robot's position within a known map, a **Particle Filter (Monte Carlo Localisation)** has been implemented from scratch.

#### Pure Python Core (`src/localisation/particle_filter.py`)
The algorithm's mathematics are cleanly separated from ROS 2 infrastructure:
1. **Initialization:** 500 particles are generated to represent possible robot poses $(x, y, \theta)$.
2. **Prediction (Motion Model):** As the robot drives, the odometry changes $(\Delta x, \Delta y, \Delta \theta)$ are applied to every particle. To model real-world slippage, Gaussian noise is injected during this step.
3. **Update (Sensor Model):** A sub-sampled set of rays from the Lidar scan is projected from each particle's assumed pose into the map. A simple end-point model evaluates if the ray hits an obstacle ($Z_{hit}$) or empty space ($Z_{rand}$). The particle's weight is updated based on how well its simulated scan matches the real scan.
4. **Resampling:** A low-variance systematic resampling step eliminates low-probability particles and duplicates those that strongly align with the sensor data.

#### ROS 2 Integration (`src/localisation/mcl_node.py`)
A dedicated ROS 2 node runs alongside the controller:
- It subscribes to `/scan` for Lidar data and uses the `odom` $\rightarrow$ `base_link` transform for movement tracking.
- Currently, it tests the particle filter against a hardcoded Configuration Space mock map (mirroring the square obstacle in the Path Controller). 
- It publishes its best guess as a `PoseStamped` on `/mcl_pose` (for visualization in RViz) and broadcasts the standard `map` $\rightarrow$ `odom` TF transform.

---

### 3. Environment Exploration

To enable the Robile to autonomously discover its surroundings, an active environment exploration strategy has been implemented and integrated with a SLAM (Simultaneous Localization and Mapping) component.

#### Active Mapping Integration
The exploration strategy operates in parallel with an active mapping node (`src/mapping/mapping_node.py` & `occupancy_grid_mapper.py`). As the robot navigates the environment, the mapping node continuously processes Lidar scans and odometry data to build and update an occupancy grid map. This dynamic `/map` is fed back into both our global planner and the particle filter, replacing the hardcoded mock maps previously used for testing.

#### Frontier-Based Exploration Strategy
The core of our exploration logic relies on frontier-based pose selection:
- **Identifying Fringes:** The algorithm scans the current occupancy grid to find "frontiers" — the boundaries separating known, explored free space from unknown, unexplored regions.
- **Pose Selection Logic:** When the robot needs a new destination, the exploration node (`src/exploration/frontier_explorer.py`: `FrontierExplorer`) identifies the most optimal frontier cell. It calculates a target pose near this boundary, ensuring it lies within known free space, and orients the robot to face the unknown region.
- **Transitioning to Unexplored Regions:** This target pose is passed to the global flood-fill planner, which generates a path through the safe Configuration Space to the frontier. Once the robot reaches this fringe, the sensors sweep the unknown area, the mapping node updates the grid, and new frontiers are calculated. This iterative process expands the map until the entire accessible environment is fully explored.

---

## Challenges & Visual Documentation

### Troubleshooting & Sim2Real Gap
Transitioning from the Gazebo simulation to the physical Robile hardware presented several challenges:
- **Sensor Noise & Discrepancies:** Lidar data on the physical robot was noisier than initially simulated. However, because we already injected Gaussian noise in our simulation tests, the particle filter transferred reasonably well.
- **Dynamic Replanning:** To account for previously undetected walls or dynamic obstacles moving out of the way, the path to the goal is continuously replanned at set intervals (rather than just replanning the final end goal, which only changes if it becomes unreachable).
- **Mapping Drift & Robot Speed:** The control loop optimization limits how fast the robot can drive before significant drift occurs, causing mapping artifacts. The robot's speed in simulation feels much slower than reality. To mitigate this, we reduced the robot's speed on the physical hardware so that the controller and mapping loops could keep up, resulting in a much more accurate map with less drift.

### Visual Evidence
As documented above, we have validated our implementations visually:
- [Flood-fill with C-Space inflation demonstration](img/flood_fill_demo_inflated.png)
- [Square path validation (GIF)](img/square_path_validation.gif)

### Video Demonstration
You can view our final project video demonstrating path planning, localisation, and environment exploration running smoothly on the real robot in the lab here: 
**[Link to Final Project Video]()** *(Insert actual URL here)*

#### 1. Real-World Navigation & Dynamic Replanning
<video width="320" height="240" controls>
  <source src="./vid/Real/_____This is the good stuff__after_replan_threshold/video_robot.mp4" type="video/mp4">
</video>

This video showcases the physical Robile hardware navigating a real-world environment. 
**Key aspects demonstrated:**
* **Navigation and Path Execution:** The robot successfully follows its planned trajectory, demonstrating the integration of the global path planner and the local potential field planner.
* **Dynamic Replanning:** The video highlights the system's ability to handle sensor noise and dynamic environments. While moving towards the next waypoint, if the path is determined to be unreachable—perhaps due to a newly detected obstacle or a false positive from sensor noise—the robot halts, rotates, and recalculates a new path to the goal. 
* **Sim2Real Discrepancies:** This behavior explicitly illustrates the sim2real gap mentioned in the challenges section. In the Gazebo simulation, the environment is pristine, but in reality, lidar noise occasionally causes the robot to replan even when the original goal was technically still reachable. 

#### 2. Full System Integration (Path Planning & Mapping)
<video width="320" height="240" controls>
  <source src="./vid/Real/_____This is the good stuff__after_replan_threshold/navigating right space - with path planning and mapping.webm" type="video/webm">
</video>

*(Note: The previous `video_robot.mp4` is a highlighted clip taken from this longer run.)*

This video shows a longer, continuous run of the robot navigating within a confined space (room C69). 
**Key aspects demonstrated:**
* **Integrated System:** It serves as a proof of concept that all core components—the controller, path planner, and mapping tool—are functioning together on the physical hardware. 
* **Mapping:** The active mapping process is visible, building a representation of the environment as the robot moves. You can observe minor mapping artifacts, which ties back to the challenges discussed regarding control loop speeds and drift.
* **Environmental Constraints:** The run is relatively short due to space limitations in the testing area (the ideal testing space outside was occupied for leak repairs), but it effectively validates the underlying logic.

#### 3. Real-Time Occupancy Grid Mapping
<video width="320" height="240" controls>
  <source src="./vid/Real/_____This is the good stuff__after_replan_threshold/creating smi complete room map of c69.webm" type="video/webm">
</video>

This video provides a screen recording of the system's graphical interface during a mapping run. 
**Key aspects demonstrated:**
* **Occupancy Grid Mapping:** The video clearly shows the real-time construction of a 2D occupancy grid map. You can see the map expanding as the robot explores new areas, with walls and obstacles being actively plotted. 
* **System Diagnostics:** The terminal output running alongside the visual map provides a look at the system's internal processes. It shows continuous logging from the controller node, including the constant recalculation of bearings to waypoints and instances where the robot triggers a replanning event ("Replan to..."). 
* **Exploration Logic:** This video is excellent evidence for the "Environment Exploration" objective, visualizing how the robot interprets its surroundings to eventually define the map fringes for further exploration.

### Team Collaboration
Since it was hard to manage a project like this with multiple people at once as the implementation was building upon each other we decided to spilt the tasks up and work on them after another sepereately. Like this we split up the work to this constellation:

* Niels - Path finding and environment exploration
* Lars - Localization and Documentation