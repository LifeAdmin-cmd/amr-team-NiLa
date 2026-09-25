# Autonomous Mobile Robotics (AMR) Final Project — Team NiLa

Deployment of an autonomous navigation, localisation, mapping, and exploration stack on the **Robile** mobile robot platform (both in Gazebo simulation and on physical hardware).

Developed by Team NiLa (**Ni**ls & **La**rs).

---

## Project Overview

This project implements an end-to-end robotics software stack enabling the Robile to autonomously navigate, map, and explore unknown environments:

- **Global & Local Path Planning**: A global Flood-Fill planner (with Configuration Space obstacle inflation and line-of-sight waypoint shortcutting) combined with an attractive/repulsive Potential Field Planner for local obstacle avoidance and trajectory tracking.
- **Monte Carlo Localisation (MCL)**: A custom particle filter tracking the robot pose within a map using motion prediction (odometry with Gaussian noise model) and lidar scan observation updates (ray-cast sensor model).
- **Occupancy Grid Mapping**: Real-time 2D occupancy grid generation from lidar scan hits and misses using Bresenham ray-casting and log-odds updates.
- **Frontier-Based Autonomous Exploration**: Automated detection of frontiers (boundaries between explored free space and unexplored areas) to iteratively select navigation targets until the environment is fully mapped.
- **Physical Hardware Deployment (Sim2Real)**: Automated deployment pipeline, DDS network/FastRTPS isolation, dynamic path replanning, and speed calibration to address real-world sensor noise and drift.

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
    ├── exploration/
    ├── localisation/
    ├── mapping/
    ├── path_and_motion_planning/
    └── robot/
```

---

## Quick Start

### 1. Setup & Permissions
Make all launch and utility scripts executable:
```bash
chmod +x config/env.bat launch/launch_sim.bat launch/launch_controller_robile.bat launch/robile_connection.bat
```
*(Verify or adjust ROS paths in `config/env.bat` if your ROS 2 workspace is located elsewhere).*

### 2. Running in Simulation (Gazebo)
>The current robot settings are for running in the real world (speed, heading adjustment etc.). The simulation will run but may need to be adjusted for a good experience e.g. increase robot speed.

To start the Gazebo simulation environment along with the Robile robot:
```bash
./launch/launch_sim.bat
```

### 3. Running on the Physical Robot
Start the robot controller bringup on Robile (prerequisite, only done once per session):
```bash
# use ssh by hand, keep session alive while robot is in use
ssh -x studentkelo@192.168.0.104
ros2 launch robile_bringup robot.launch.py
```

Connect to the **Robile5G** Wi-Fi network and execute the automated connection pipeline:
```bash
# Connect to default robot (Robile 4) and launch all nodes
./launch/robile_connection.bat

# Or connect to a specific robot ID (e.g., Robile 3)
./launch/robile_connection.bat 3
```

The script automatically executes the entire deployment pipeline:
1. **Network Configuration**: Auto-detects the active Wi-Fi interface, writes the FastRTPS XML profile whitelist, and updates `~/.bashrc` with required DDS environment variables.
2. **Domain ID Synchronization**: Sets and exports `ROS_DOMAIN_ID` matching the target robot (`1`-`4`) to isolate DDS traffic.
3. **Connectivity & Topic Verification**: Pings the physical robot and checks whether hardware topics are publishing.
4. **Local Node Launch**: Launches separate `gnome-terminal` tabs for the Path Controller, MCL Localisation, and Occupancy Grid Mapping.

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

<img src="assets/img/flood_fill_demo.png" width="600"/>

#### First path and motion planning validation
Quick end-to-end validation before continuing: the planned path/waypoints (left) next to the actual robot driving them in Gazebo (right). 
<img src="assets/img/square_path_validation.gif" width="600"/>

#### Obstacle Inflation (Configuration Space / C-Space)
To prevent the robot from getting stuck on corners or choosing passages it cannot physically fit through, **obstacle inflation** has been integrated into the global planner.
* **How it works:** The map is pre-processed before the flood-fill algorithm is executed. Every grid cell marked as an obstacle is artificially "inflated" by a defined radius.
* **Configuration:** The safety margin can be configured during the initialization of the `FloodFillPlanner` using the `inflation_radius_cells` parameter. 

<img src="assets/img/flood_fill_demo_inflated.png" width="600"/>

### 2. Localisation (Monte Carlo Localisation)
To accurately track the robot's position within a known map, a **Particle Filter (Monte Carlo Localisation)** has been implemented from scratch.
- **Pure Python Core (`src/localisation/particle_filter.py`)**: Handles Initialization, Prediction (with Gaussian noise), Update (ray-cast sensor model), and Resampling.
- **ROS 2 Integration (`src/localisation/mcl_node.py`)**: Runs alongside the controller, subscribes to `/scan`, uses TF for tracking, and publishes `/mcl_pose`.

### 3. Environment Exploration
To enable the Robile to autonomously discover its surroundings, an active environment exploration strategy has been implemented and integrated with a SLAM component.
- **Active Mapping Integration**: The exploration strategy operates in parallel with an active mapping node (`src/mapping/mapping_node.py`).
- **Frontier-Based Exploration Strategy**: The algorithm scans the current occupancy grid to find "frontiers" and computes an optimal target pose near the boundary (`src/exploration/frontier_explorer.py`). 

---

## Challenges & Visual Documentation

### Troubleshooting, Sim2Real Gap & Comparison
Transitioning from Gazebo simulation to the physical Robile hardware highlighted several discrepancies:
- **Sensor Noise & Map Fidelity**: Real-world Lidar data is significantly noisier, producing artifacts.
- **Dynamic Replanning Triggers**: Replanning is triggered much more frequently in the real world due to sensor noise creating "phantom" obstacles.
- **Mapping Drift & Control Loop Speed**: Driving the physical robot too fast caused the control loop to lag behind the physical movement, leading to severe mapping drift. We mitigated this by reducing the physical robot's speed. (As additionally documented in the original notes, this slower speed ensures the controller/mapping loop can keep up, reducing drift and artifacts).

### Video Demonstration
[![Watch the simulation](./assets/vid/Directors_Cut/preview.gif?raw=true)](https://youtu.be/Cx1pNA0cUJQ)
*▶️ Click the preview above to watch the full video on YouTube.*

**About this video:**
This final video showcases the complete capabilities of our Autonomous Mobile Robot (AMR) system, bridging the gap between simulated testing and physical hardware. The video demonstrates the robot performing autonomous frontier exploration, dynamic path planning, and real-time occupancy grid mapping. You will see the system gracefully handle both a controlled Gazebo simulation and a real-world deployment in room C69, specifically highlighting its robust fallback replanning logic where the robot halts, rotates, and recalculates a new path when it encounters sensor noise or unreachable, constrained areas.

**Other Demonstration Videos:**
If you would like to view the individual, unedited clips of our system in action—including the raw simulation footage of exploration, the continuous real-world hardware navigation runs, and the real-time screen recordings of the occupancy grid generation—they can all be found within the project repository under the `docs/vid/Sim/` and `docs/vid/Real/` directories.

## Authors

**Team NiLa**:
* **Nils** - Path and motion planning, mapping, environment exploration, basic connection bring up
* **Lars** - Particle filter localisation, system integration, documentation, clean connection workflow
