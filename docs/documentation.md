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
2. **Open a Persistent Tmux Session (Tmux specific - optional)**:
   ```bash
   tmux new -s bringup
   ```
3. **Launch the Control Task**:
   ```bash
   ros2 launch robile_bringup robot.launch.py
   ```
4. **Detach from Session (Tmux specific)**:
   Press `Ctrl + b`, then release and press `d`. You can now safely close the SSH terminal; the control task remains running.
5. **Re-attach or Terminate (Tmux specific)**:
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

### Video Demonstration

[![Watch the simulation](https://github.com/LifeAdmin-cmd/amr-team-NiLa/blob/main/docs/vid/Directors_Cut/preview.gif?raw=true)](https://youtu.be/Cx1pNA0cUJQ)
*▶️ Click the preview above to watch the full video on YouTube.*

**About this video:**
This final "Director's Cut" video showcases the complete capabilities of our Autonomous Mobile Robot (AMR) system, bridging the gap between simulated testing and physical hardware. The video demonstrates the robot performing autonomous frontier exploration, dynamic path planning, and real-time occupancy grid mapping. You will see the system gracefully handle both a controlled Gazebo simulation and a real-world deployment in room C69, specifically highlighting its robust fallback replanning logic where the robot halts, rotates, and recalculates a new path when it encounters sensor noise or unreachable, constrained areas.

**Other Demonstration Videos:**
If you would like to view the individual, unedited clips of our system in action—including the raw simulation footage of exploration, the continuous real-world hardware navigation runs, and the real-time screen recordings of the occupancy grid generation—they can all be found within the project repository under the `docs/vid/Sim/` and `docs/vid/Real/` directories.

### Team Collaboration
Since it was hard to manage a project like this with multiple people at once as the implementation was building upon each other we decided to split the tasks up and work on them after another separately. Like this we split up the work to this constellation:

* Nils - Path and motion planning, mapping, environment exploration, basic connection bring up
* Lars - Particle filter localisation, system integration, documentation, clean connection workflow

