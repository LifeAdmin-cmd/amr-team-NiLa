# Autonomous Mobile Robotics (AMR) Final Project — Team NiLa

Deployment of an autonomous navigation, localisation, mapping, and exploration stack on the **Robile** mobile robot platform (both in Gazebo simulation and on physical hardware).

Developed by Team NiLa (**Ni**ls & **La**rs).

---

## Project Overview

The goal of this project is to implement and deploy an end-to-end autonomous navigation, localisation, and SLAM-based environment exploration stack on the Robile platform, transitioning seamlessly from a controlled simulation environment to physical hardware while overcoming real-world challenges.

Key components of the architecture include:
- **Global & Local Path Planning**: A global Flood-Fill planner (incorporating Configuration Space obstacle inflation and line-of-sight waypoint shortcutting) paired with an attractive/repulsive Potential Field Planner for local obstacle avoidance and trajectory tracking.
- **Monte Carlo Localisation (MCL)**: A custom particle filter tracking the robot pose within a map using motion prediction (odometry with a Gaussian noise model) and lidar scan observation updates (ray-cast sensor model).
- **Occupancy Grid Mapping**: Real-time 2D occupancy grid generation from lidar scan hits and misses using Bresenham ray-casting and log-odds updates.
- **Frontier-Based Autonomous Exploration**: Automated detection of frontiers (boundaries between explored free space and unexplored areas) to iteratively select navigation targets until the environment is fully mapped.
- **Physical Hardware Deployment (Sim2Real)**: Automated deployment pipeline, DDS network/FastRTPS isolation, dynamic path replanning, and speed calibration to address real-world sensor noise, latency, and drift.

---

## Prerequisites & Network Setup

To run this project, ensure you have the following installed and configured:
- **Dependencies**: ROS 2 installed as per the course material.
- **Environment Variables**: Configure your ROS 2 environment, particularly `ROS_DOMAIN_ID`, to ensure proper communication between nodes and avoid interference with other teams. Set this in the provided `config/env.bat` script.
- **Network Configuration**: The system requires the generation of a FastRTPS XML profile for proper DDS communication over the network, especially when interfacing with the real Robile.

---

## Repository & File Structure

```
amr-team-NiLa/
├── README.md                      # Project summary, structure, and documentation
├── launch/                        # Launch scripts for simulation and hardware deployment
│   ├── launch_sim.bat
│   ├── launch_controller_robile.bat
│   ├── robile_connection.bat
│   └── lars_launch_sim.sh
├── config/                        # Configuration files (e.g. env.bat)
│   └── env.bat
├── maps/                          # Map files (if any)
├── assets/                        # Documentation images, C-Space plots, and videos
│   ├── img/
│   └── vid/
└── src/                           # Python source code for ROS 2 nodes and robotics algorithms
    ├── controller.py              # Central coordination node
    ├── exploration/               # Frontier exploration logic
    ├── localisation/              # Particle filter and MCL node
    ├── mapping/                   # Occupancy grid generation
    ├── path_and_motion_planning/  # Potential fields, flood fill, obstacle inflation
    └── robot/
```

### Repository Management
- **Single Branch**: All code and documentation for this project are maintained on a single main branch.
- **.gitignore**: We utilize a `.gitignore` file to ensure that only necessary files are committed, preventing the upload of large, unnecessary folders (such as build artifacts or temporary data).

---

## Quick Start & Execution Instructions

### 0. Step up Robile (Ony for real robot)
Follow the setup steps for ros2, sim, robot config xml, on amr project page wiki. //**course wiki**

### 1. Setup & Permissions
Make all launch and utility scripts executable (Linux/macOS):
```bash
chmod +x config/env.bat launch/launch_sim.bat launch/launch_controller_robile.bat launch/robile_connection.bat
```
*(Verify or adjust ROS paths and environment variables like `ROS_DOMAIN_ID` in `config/env.bat` if your ROS 2 workspace is located elsewhere).*

### 2. Running in Simulation (Gazebo)
> **Note on Simulation Tuning**: The default robot parameters are calibrated for the physical hardware (velocity constraints, heading adjustments, etc.). The simulation will run out of the box, but you may want to increase the commanded robot speed for a faster testing loop.

To start the Gazebo simulation environment along with the Robile robot:
```bash
./launch/launch_sim.bat
```

### 3. Running on the Physical Robot

#### Connecting to the Robot
Connect your machine to the **Robile5G** Wi-Fi network and execute the automated connection pipeline:
```bash
# Connect to default robot (Robile 4) and launch all nodes
./launch/robile_connection.bat

# Or target a specific robot ID (e.g., Robile 3)
./launch/robile_connection.bat 3
```

The script automatically executes the entire deployment pipeline without manual menu navigation:
1. **Network Configuration**: Auto-detects the active Wi-Fi interface (e.g., `wlp1s0`), writes the FastRTPS XML profile whitelist (`~/ros2_network_config.xml`), and updates `~/.bashrc` with required DDS environment variables.
2. **Domain ID Synchronization**: Sets and exports `ROS_DOMAIN_ID` matching the target robot (`1`–`4`) to isolate DDS traffic.
3. **Connectivity & Topic Verification**: Pings the physical robot (`192.168.0.10X`), restarts the ROS 2 daemon if needed, and checks whether hardware topics are publishing.
4. **Local Node Launch**: Launches separate `gnome-terminal` tabs for the Path Controller (`src/controller.py`), MCL Localisation (`src/localisation/mcl_node.py`), and Occupancy Grid Mapping (`src/mapping/mapping_node.py`), each pre-configured with the correct `ROS_DOMAIN_ID`.

#### Starting the Control Task on the Physical Robot (`robile_bringup`)

The physical Robile hardware requires an onboard driver and control stack to translate high-level `/cmd_vel` velocity commands into low-level wheel torques, and to stream sensor measurements:
- **Kelo Tulip Drive Driver** (`kelo_tulip`): Interfaces with the Kelo smart wheel modules, computing kinematics and controlling the drive motors.
- **Lidar Scanner Driver**: Publishes `/scan` (using `sick_scan` for Sick LMS1XX on Robile 3, or `urg_node2` for Hokuyo on Robile 4).
- **Robot State Publisher & Transforms**: Publishes `/robot_description`, `/tf`, `/odom`, and static transforms (`base_link` $\rightarrow$ `base_laser`, `base_link` $\rightarrow$ `base_footprint`).

##### Option A: Automated Start (Remote Bringup via SSH)
You can start the onboard control task directly from your host using:
```bash
./launch/robile_connection.bat 4 --remote-bringup
```
This connects to the robot via SSH and launches the bringup stack inside a detached `tmux` session named `bringup`.

##### Option B: Manual Start & Monitoring (Best Practice)
Running the control task inside `tmux` is essential: if an SSH terminal drops due to Wi-Fi jitter, processes running directly in that terminal will terminate, stopping the robot mid-operation. A `tmux` session persists independently.

1. **SSH into the Robot**:
   ```bash
   ssh -x studentkelo@192.168.0.104    # Password: area5142
   # Or using the configured bashrc alias:
   robile4
   ```
2. **Open a Persistent Tmux Session**:
   ```bash
   tmux new -s bringup
   ```
3. **Launch the Control Task**:
   ```bash
   ros2 launch robile_bringup robot.launch.py
   ```
4. **Detach from Session**:
   Press `Ctrl + b`, release, and press `d`. You can now safely close the SSH terminal; the control task remains running.
5. **Re-attach or Terminate**:
   - To inspect running driver output: `tmux attach -t bringup`
   - To stop the robot control task: `tmux kill-session -t bringup`

---

## Core Task Implementation

### 1. Path and Motion Planning

#### Attraction-Based Motion Planning
The local motion planner is adapted from the assignment potential field planner (`src/path_and_motion_planning/potential_field_planner.py`: `PotentialFieldPlanner`), ported as-is and wrapped into a modular Python class for clean integration.

#### Flood Fill with Line-of-Sight Shortcuting
To solve the local minima problem where a pure potential field planner gets stuck on larger maps, a global planner sits on top (`src/path_and_motion_planning/flood_fill_planner.py`: `FloodFillPlanner`):
1. **Flood fill**: Breadth-First Search (BFS) from the goal assigns every free cell a distance-to-goal value.
2. **Greedy descent**: From the start position, the algorithm iteratively steps to the neighbor with the lowest distance-to-goal, producing a complete grid path.
3. **Waypoint reduction**: Given path `p0 -> p1 -> p2 -> p3 -> p4 -> ...`, if `p0` has direct line-of-sight to `p4` (no occupied cells along the ray), intermediate waypoints `p1`–`p3` are dropped, forming direct segment `p0 -> p4`. Diagonal cuts are strictly blocked if either corner cell is occupied to prevent clipping pinch points where the robot cannot fit, forcing an L-shaped detour instead.

These simplified waypoints feed one at a time into the local potential field planner.

<img src="assets/img/flood_fill_demo.png" width="600"/>

#### Path and Motion Planning Validation
End-to-end validation was performed comparing the planned path/waypoints (left) against the robot navigating the path in Gazebo (right):

<img src="assets/img/square_path_validation.gif" width="600"/>

*Note on Validation*: The robot's position at controller start is initialized as $(0,0)$ with heading $0$, introducing a slight offset/flip relative to the planning image coordinate frame, though the robot navigates the waypoints correctly. While the path appears to pinch the corner around square obstacles, it clears it safely; fine-grained avoidance is handled dynamically by the potential field planner.

> **Subject to change**: A planned extension is establishing a minimum number of free grid squares required for passability (a single free cell clearance is insufficient given the robot's footprint). A straightforward resolution is grouping occupancy cells into robot-sized chunks prior to flood-fill execution.

#### Obstacle Inflation (Configuration Space / C-Space)
To prevent the robot from colliding with corners or entering passages narrower than its chassis, **obstacle inflation** is applied directly within the global planner. This shifts planning from a point-mass assumption to accounting for the robot's physical dimensions.

* **How it works**: Before running flood-fill, the map is pre-processed. Every cell identified as an obstacle is inflated radially using Euclidean distance metrics, defining an obstacle buffer. The planner then computes its path exclusively within this safe Configuration Space (C-Space).
* **Configuration**: The safety margin is configured during `FloodFillPlanner` initialization via the `inflation_radius_cells` parameter. At a grid resolution of $0.1\,\text{m}$ per cell, a setting of `3` creates a $30\,\text{cm}$ buffer, matching the dimensions of the Robile platform.

<img src="assets/img/flood_fill_demo_inflated.png" width="600"/>

*Figure: The planner correctly identifies that the goal is unreachable because narrow gaps in walls (1 cell) are completely closed off by the robot's inflation radius (3 cells) in the Configuration Space.*

---

### 2. Localisation (Monte Carlo Localisation)

To accurately track the robot's position within a known map, a **Particle Filter (Monte Carlo Localisation)** was implemented from scratch.

#### Pure Python Core (`src/localisation/particle_filter.py`)
The algorithmic logic is cleanly decoupled from ROS 2 infrastructure:
1. **Initialization**: Generates 500 particles representing possible robot poses $(x, y, \theta)$.
2. **Prediction (Motion Model)**: Odometry increments $(\Delta x, \Delta y, \Delta \theta)$ are applied to every particle. A Gaussian noise model accounts for real-world wheel slip and odometry drift.
3. **Update (Sensor Model)**: Sub-sampled ray casts project from each particle pose into the map. An end-point model evaluates whether rays encounter obstacles ($Z_{\text{hit}}$) or open space ($Z_{\text{rand}}$), weighting each particle by how closely its simulated scan correlates with the true Lidar scan.
4. **Resampling**: Low-variance systematic resampling removes low-probability particles and replicates high-confidence particles.

#### ROS 2 Integration (`src/localisation/mcl_node.py`)
A dedicated node coordinates the filter within the ROS 2 ecosystem:
- Subscribes to `/scan` for Lidar returns and uses the `odom` $\rightarrow$ `base_link` transform for movement tracking.
- Validated against a Configuration Space mock map (mirroring the square obstacle in the Path Controller).
- Publishes the estimated state as a `PoseStamped` on `/mcl_pose` (for visualization in RViz) and broadcasts the `map` $\rightarrow$ `odom` TF transform.

---

### 3. Environment Exploration

To enable the Robile to autonomously discover its surroundings, an active exploration strategy operates alongside a SLAM component.

#### Active Mapping Integration
The exploration system runs concurrently with an active mapping node (`src/mapping/mapping_node.py` & `occupancy_grid_mapper.py`). As the robot explores, the mapping node continuously processes Lidar scans and odometry data using Bresenham ray-casting and log-odds updates to generate a 2D occupancy grid. This dynamic `/map` is fed back into both the global planner and the particle filter, superseding static mock maps.

#### Frontier-Based Exploration Strategy
Exploration targets are selected using frontier detection (`src/exploration/frontier_explorer.py`: `FrontierExplorer`):
- **Identifying Fringes**: The algorithm scans the occupancy grid to detect "frontiers"—the boundary cells separating known free space from unexplored, unknown space.
- **Pose Selection Logic**: When a new goal is required, the explorer evaluates candidate frontier clusters to pick an optimal frontier cell. It generates a target pose within known free space oriented toward the unexplored zone.
- **Transitioning to Unexplored Regions**: The target pose is dispatched to the global flood-fill planner, which generates a collision-free path across the C-Space. Upon reaching the frontier, the Lidar sweeps the unknown area, the occupancy grid updates, and new frontiers are identified until the reachable space is mapped.

---

## Challenges & Visual Documentation

### Troubleshooting, Sim2Real Gap & Comparison
Transitioning algorithms from Gazebo to the physical Robile platform exposed key discrepancies:
- **Sensor Noise & Map Fidelity**: Real-world Lidar data exhibits substantially more noise (picking up table legs, floor variations, reflective surfaces) than pristine simulation scans. While Gaussian noise simulation prepared the MCL filter, physical occupancy grids inherently contain more artifacts and fuzzier boundaries.
- **Dynamic Replanning Triggers**: Replanning acts as an autonomous escape mechanism when the robot enters constrained spaces. In the real world, sensor noise frequently creates "phantom" obstacles across paths, triggering replanning cycles even when the physical path is unobstructed.
- **Mapping Drift & Control Loop Speed**: Driving the physical robot at higher velocities caused the control and mapping loops to lag behind physical displacement, leading to severe map distortion. Throttling physical drive speed ensured the mapping and controller cycles kept pace with odometry, eliminating drift.

### Video Demonstration

[![Watch the simulation](./assets/vid/Directors_Cut/preview.gif?raw=true)](https://youtu.be/Cx1pNA0cUJQ)

*▶️ Click the preview above to watch the full video on YouTube.*

**About this video:**
This final "Director's Cut" video showcases the complete capabilities of our Autonomous Mobile Robot (AMR) system, bridging the gap between simulated testing and physical hardware. The video demonstrates the robot performing autonomous frontier exploration, dynamic path planning, and real-time occupancy grid mapping. You will see the system gracefully handle both a controlled Gazebo simulation and a real-world deployment in room C69, specifically highlighting its robust fallback replanning logic where the robot halts, rotates, and recalculates a new path when it encounters sensor noise or unreachable, constrained areas.

**Other Demonstration Videos:**
Raw, unedited footage—including simulation exploration runs, uninterrupted real-world navigation trials, and live screen captures of occupancy grid construction—can be found in the repository under `assets/vid/` (and archived under `docs/vid/Sim/` and `docs/vid/Real/`).

---

## Authors & Team Collaboration

Because the software stack components were highly interdependent, development was structured into sequential, focused milestones:

- **Nils**: Path and motion planning (potential fields, flood fill, obstacle inflation), occupancy grid mapping, frontier-based autonomous exploration, and initial robot connection bringup.
- **Lars**: Monte Carlo Localisation (particle filter design and ROS 2 integration), system integration, documentation, automated FastRTPS/DDS network isolation, and deployment pipelines.