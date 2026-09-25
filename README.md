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

## Detailed Documentation

For in-depth explanations, mathematical foundations, implementation details, Sim2Real insights, and video demonstrations, please see:

📖 **[Full Project Documentation](docs/documentation.md)**

The documentation covers:
- **System Architecture & Prerequisites**: ROS 2 Humble setup, DDS / FastRTPS network profiles, and environment variables.
- **Physical Robot Deployment & Bringup**: SSH/tmux bringup of low-level Kelo tulip drives, lidar drivers (`sick_scan` / `urg_node2`), and TF transforms.
- **Core Algorithms**: In-depth breakdown of Path Planning (Flood Fill + Potential Fields + C-Space inflation), MCL Particle Filtering, and Frontier Exploration.
- **Sim2Real Challenges & Tuning**: Handling lidar noise, dynamic obstacle replanning, and odometry drift mitigation.
- **Video Demonstrations**: Recorded runs on the physical Robile robot in the lab showing dynamic replanning, continuous mapping, and exploration.
- **Team Contributions**: Individual task breakdown and collaboration details.

---

## Repository & File Structure

```
amr-team-NiLa/
├── README.md                      # Project summary, structure, and quickstart (this file)
├── env.bat                        # Environment configuration (ROS paths, PYTHONPATH, FastRTPS)
├── launch_sim.bat                 # Launches Gazebo simulation with the Robile robot and world
├── launch_controller_robile.bat   # Launches controller, MCL, and mapping nodes in separate terminal tabs
├── robile_connection.bat          # Automated deployment script for physical Robile (DDS, bringup, node launch)
├── lars_launch_sim.sh             # Simulation helper script
│
├── docs/                          # Comprehensive project documentation and media
│   ├── documentation.md           # Main project report & complete technical documentation
│   ├── sim_real_gap.md            # Notes on Sim2Real transfer and hardware tuning
│   ├── img/                       # Documentation images, C-Space plots, and validation animations
│   └── vid/                       # Video recordings of physical robot tests and interface screencasts
│
└── src/                           # Python source code for ROS 2 nodes and robotics algorithms
    ├── controller.py              # Central coordination node integrating planning, mapping, and driving
    ├── exploration/
    │   └── frontier_explorer.py   # Frontier detection and exploration goal selection
    ├── localisation/
    │   ├── particle_filter.py     # Pure-Python Monte Carlo Localisation (MCL) particle filter
    │   └── mcl_node.py            # ROS 2 node interfacing the particle filter with /scan and TF
    ├── mapping/
    │   ├── occupancy_grid_mapper.py # 2D Occupancy grid ray-casting and map representation
    │   └── mapping_node.py        # ROS 2 node publishing /map and processing odometry & scans
    ├── path_and_motion_planning/
    │   ├── flood_fill_planner.py  # Global BFS flood-fill planner with C-Space inflation & shortcutting
    │   ├── potential_field_planner.py # Attractive/repulsive potential field local controller
    │   └── validate_flood_fill.py # Offline unit test and validation script for flood-fill planner
    └── robot/
        └── robot.py               # Robot state abstraction, kinematics, and movement interfaces
```

---

## Quick Start

### 1. Setup & Permissions
Make all launch and utility scripts executable:
```bash
chmod +x env.bat launch_sim.bat launch_controller_robile.bat robile_connection.bat
```
*(Verify or adjust ROS paths in `env.bat` if your ROS 2 workspace is located elsewhere).*

### 2. Running in Simulation (Gazebo)
>The Current robot settings are for running is real world (speed, heading adjustment etc.) the simulation will run but may need to be adjusted for a good experience e.g. increase robot speed

To start the Gazebo simulation environment along with the Robile robot:
```bash
./launch_sim.bat
```

### 3. Running on the Physical Robot
Start the robot controller bringup on Robile (prerequisit) only done once per session:
```bash
# use ssh by hand, keep seesion alive while robot is in use
ssh -x studentkelo@192.168.0.104
ros2 launch robile_bringup robot.launch.py
```

Connect to the **Robile5G** Wi-Fi network and execute the automated connection pipeline:
```bash
# Connect to default robot (Robile 4) and launch all nodes
./robile_connection.bat

# Or connect to a specific robot ID (e.g., Robile 3)
./robile_connection.bat 3
```

---

## Authors

**Team NiLa**:
* **Nils** - Path and motion planning, mapping, environment exploration, basic connection bring up
* **Lars** - Particle filter localisation, system integration, documentation, clean connection workflow
