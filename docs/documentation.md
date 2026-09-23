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
To deploy on the physical Robile platform, connect your machine to the **Robile5G** Wi-Fi network and run the automated connection script:
```bash
# Connect to default robot (Robile 4) and launch all nodes
./robile_connection.bat

# Or target a specific robot (e.g., Robile 3)
./robile_connection.bat 3
```

The script automatically executes the entire deployment pipeline without manual menu navigation:
1. **Network Configuration**: Auto-detects the active Wi-Fi interface (e.g. `wlp1s0`), writes the FastRTPS XML profile whitelist (`~/ros2_network_config.xml`), and updates `~/.bashrc` with required DDS environment variables.
2. **Domain ID Synchronization**: Sets and exports `ROS_DOMAIN_ID` matching the target robot (`1`-`4`) to isolate DDS traffic.
3. **Connectivity & Topic Verification**: Pings the physical robot (`192.168.0.10X`), restarts the ROS 2 daemon if needed, and checks whether hardware topics are publishing.
4. **Local Node Launch**: Launches separate `gnome-terminal` tabs for the Path Controller (`src/controller.py`), MCL Localisation (`src/localisation/mcl_node.py`), and Occupancy Grid Mapping (`src/mapping/mapping_node.py`), each pre-configured with the correct `ROS_DOMAIN_ID`.

#### Starting the Control Task on the Physical Robot (`robile_bringup`)

The physical Robile hardware requires an onboard driver and control stack to translate high-level `/cmd_vel` velocity commands into low-level wheel torques, and to stream sensor measurements:
- **Kelo Tulip Drive Driver** (`kelo_tulip`): Interfaces with the Kelo smart wheel modules, computing kinematics and controlling the drive motors.
- **Lidar Scanner Driver**: Publishes `/scan` (using `sick_scan` for Sick LMS1XX on Robile 3, or `urg_node2` for Hokuyo on Robile 4).
- **Robot State Publisher & Transforms**: Publishes `/robot_description`, `/tf`, `/odom`, and static transforms (`base_link` $\rightarrow$ `base_laser`, `base_link` $\rightarrow$ `base_footprint`).

##### 1. Automated Start (Remote Bringup via SSH)
You can start the onboard control task directly from your host using:
```bash
./robile_connection.bat 4 --remote-bringup
```
This connects to the robot via SSH and launches the bringup stack inside a detached `tmux` session named `bringup`.

##### 2. Manual Start & Monitoring (Best Practice)
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
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   export ROS_DOMAIN_ID=4
   export ROBOT_NAME=robile4
   ros2 launch robile_bringup robot.launch.py
   ```
4. **Detach from Session**:
   Press `Ctrl + b`, then release and press `d`. You can now safely close the SSH terminal; the control task remains running.
5. **Re-attach or Terminate**:
   - To inspect running driver output: `tmux attach -t bringup`
   - To stop the robot control task: `tmux kill-session -t bringup`

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

### Troubleshooting, Sim2Real Gap & Comparison
Transitioning the core algorithms from the pristine Gazebo simulation to the physical Robile hardware highlighted several key discrepancies, commonly known as the sim2real gap. Comparing our simulation runs with the real-world tests reveals the following:

- **Sensor Noise & Map Fidelity:** In the simulation (`exploration.webm`, `mapping demo.webm`), the simulated Lidar produces clean, sharp boundaries, resulting in a highly accurate and crisp occupancy grid. On the physical robot, Lidar data is significantly noisier (picking up table legs, uneven surfaces, etc.). While injecting Gaussian noise in simulation helped prepare the particle filter, the real-world map inherently exhibits more artifacts and fuzzier edges compared to the simulation baseline.
- **Dynamic Replanning Triggers:** The simulation video (`path_recalculate_stuck.webm`) demonstrates our replanning logic acting as a robust fallback; when the robot is genuinely stuck in a tight corner, it hits a failure threshold and intelligently selects a new frontier. However, in the real world (`video_robot.mp4`), this replanning is triggered much more frequently. Sensor noise often creates "phantom" obstacles that temporarily block the calculated path, causing the robot to halt and replan even when the physical path is technically clear.
- **Mapping Drift & Control Loop Speed:** In simulation, the compute resources easily handle the control and mapping loops, allowing for steady, reliable movement. On the physical hardware, we discovered that driving the robot too fast caused the control loop to lag behind the physical movement, leading to severe mapping drift and artifacts. To mitigate this, we had to deliberately reduce the physical robot's speed so the processing could keep pace, resulting in a much more accurate map.

### Visual Evidence
As documented above, we have validated our implementations visually during the development phase:
- [Flood-fill with C-Space inflation demonstration](img/flood_fill_demo_inflated.png)
- [Square path validation (GIF)](img/square_path_validation.gif)

### Video Demonstration: Simulation Baseline
Before deploying to the physical robot, the system was thoroughly validated in the Gazebo simulator. These videos demonstrate the intended behavior in a controlled environment.

#### 1. Autonomous Exploration & Mapping (Simulation)
<video width="320" height="240" controls>
  <source src="./vid/Sim/exploration.webm" type="video/webm">
</video>

This video shows the robot autonomously navigating a Gazebo maze. It highlights the clean construction of the occupancy grid and the successful execution of the frontier exploration logic, moving smoothly from one map boundary to the next.

#### 2. Robustness and Fallback Replanning (Simulation)
<video width="320" height="240" controls>
  <source src="./vid/Sim/path_recalculate_stuck.webm" type="video/webm">
</video>

This recording of the system terminal and live map visualizes the fallback logic. When the robot navigates into a constrained area and fails to reach its target, the controller logs the failed replan attempts. Once a threshold is reached (e.g., 10 failed attempts), it successfully discards the unreachable target and selects a new frontier, preventing the system from freezing.

### Video Demonstration: Real-World Deployment
You can view our final project videos demonstrating path planning, localisation, and environment exploration running on the real robot in the lab here: 
**[Link to Final Project Video]()** *(Insert actual URL here)*

#### 1. Real-World Navigation & Dynamic Replanning
<video width="320" height="240" controls>
  <source src="./vid/Real/_____This is the good stuff__after_replan_threshold/video_robot.mp4" type="video/mp4">
</video>

This video showcases the physical hardware navigating room C69. It highlights the system handling physical sensor noise. When a path is deemed unreachable (often due to noise), the robot halts, rotates, and recalculates a new path, demonstrating the real-world application of the replanning logic seen in the simulation.

#### 2. Full System Integration (Path Planning & Mapping)
<video width="320" height="240" controls>
  <source src="./vid/Real/_____This is the good stuff__after_replan_threshold/navigating right space - with path planning and mapping.webm" type="video/webm">
</video>

*(Note: The previous `video_robot.mp4` is a highlighted clip taken from this longer run.)*
This shows a continuous run of the robot navigating within a confined space, proving that the controller, path planner, and active mapping process function together on the physical hardware despite minor drift artifacts.

#### 3. Real-Time Occupancy Grid Mapping
<video width="320" height="240" controls>
  <source src="./vid/Real/_____This is the good stuff__after_replan_threshold/creating smi complete room map of c69.webm" type="video/webm">
</video>

This provides a screen recording of the system's graphical interface during the physical run. You can see the 2D occupancy grid expanding in real-time, visualizing how the robot interprets its physical surroundings and defining map fringes for exploration.

### Team Collaboration
Since it was hard to manage a project like this with multiple people at once as the implementation was building upon each other we decided to split the tasks up and work on them after another separately. Like this we split up the work to this constellation:

* Niels - Path finding and environment exploration
* Lars - Localization and Documentation