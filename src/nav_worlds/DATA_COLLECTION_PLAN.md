# Automated Simulation Data Collection

We will implement a Python-based orchestrator to repeatedly sample start/end coordinates from a static map, spawn the Gazebo simulation headlessly at the start coordinates, record ROS topics and a lightweight CSV state log, and drive the robot to the goal using Nav2.

## Web Search Findings

I performed a web search for existing ROS 2 Nav2 simulation data collection tools. While the Nav2 stack provides the **Simple Commander API** for scripting waypoints and there are benchmarking tools in `navigation2/tools` (like `opennav_robotics_workload_benchmark`), there is no widely-adopted, drop-in tool that handles the full lifecycle we need: automatically parsing free-space from a map, handling Gazebo headless startup/teardown per run, and organizing dataset folders. The community standard is exactly what we are planning: building a custom Python orchestrator that utilizes `subprocess`, `ros2 bag`, and `rclpy` action clients.


## Proposed Changes

### 1. Waypoint Generation (Extensible)

We will create a script that reads any given static map, identifies free space, and randomly selects valid start and end coordinates.

#### [NEW] [generate_waypoints.py](file:///home/pgrau/ros2_ws/src/nav_worlds/scripts/generate_waypoints.py)
- **Extensibility**: Accepts `--map_yaml` as an argument to support any map (defaults to `warehouse.yaml`).
- Loads the YAML, parses the corresponding `.pgm` image.
- Finds all free pixels (value > free_thresh).
- Converts pixel coordinates to real-world coordinates `(x, y)` using the map's origin and resolution.
- Randomly samples $N$ pairs of `(start_x, start_y, end_x, end_y)`.
- Enforces a minimum Euclidean distance between start and end points.
- Outputs to `waypoints.csv`.

### 2. Data Logging

We will create a lightweight node to log state variables directly to CSV for quick ML iteration without parsing rosbags.

#### [NEW] [log_state.py](file:///home/pgrau/ros2_ws/src/nav_worlds/scripts/log_state.py)
- **Extensibility**: Accepts `--odom_topic` and `--cmd_vel_topic` arguments to support different robots (e.g. `/a300_0000/odom`).
- Subscribes to the specified odometry and velocity topics.
- Writes a CSV file containing: `timestamp`, `pose_x`, `pose_y`, `pose_yaw`, `twist_linear_x`, `twist_angular_z`.
- Designed to run in the background and gracefully close the CSV on `SIGINT`.

### 3. Sweep Orchestration & Data Organization

We will create the main orchestrator script that loops through the generated waypoints, drives the simulation lifecycle, and organizes the output data.

#### [NEW] [run_sweep.py](file:///home/pgrau/ros2_ws/src/nav_worlds/scripts/run_sweep.py)
- **Real-Time Factor (RTF)**: For the initial test runs, the Gazebo physics cap will be kept near real-time (RTF ~ 1.0) to monitor performance and avoid starving system resources before pushing it faster.
- **Extensibility**: Accepts CLI arguments for `--namespace` (e.g. `a200_0000` or `a300_0000`), `--launch_pkg`, `--launch_file`, and a `--topics` list for the bagger. This ensures we can easily swap to the A300 platform and 3D lidar later without changing the orchestrator code.
- **Data Organization**: Creates a timestamped parent directory for the entire sweep, containing the metadata and a sub-folder for each run:
  ```text
  dataset_output/
    sweep_YYYYMMDD_HHMMSS/
      sweep_config.json        # Record of parameters used (map, robot, namespace, etc.)
      waypoints.csv            # Copy of the generated waypoints
      run_000/
        bag/                   # The rosbag directory (minimal topics: odom, tf, cmd_vel, scan)
        state.csv              # The lightweight state log
      run_001/
        ...
  ```
- **Lifecycle Management**: For each pair in the waypoint list:
  1. Starts the specified launch file via `subprocess` (headless).
  2. Spawns `ros2 bag record` to capture the requested topics into `run_XXX/bag/`.
  3. Spawns `log_state.py` to capture `run_XXX/state.csv`.
  4. Monitors the launch process stdout for the `READY` banner.
  5. Spawns `send_goal.py END_X END_Y --ns <namespace>` and waits for it to exit (success or timeout).
  6. Sends `SIGINT` to teardown the simulation, bagger, and logger cleanly.
  7. Pauses briefly to ensure ports/processes are freed before the next iteration.

## Verification Plan

### Manual Verification
- Generate a batch of 3 waypoints using `generate_waypoints.py` using the warehouse map.
- Run `run_sweep.py` on the 3 waypoints.
- Verify that Gazebo spawns, Nav2 plans a path, the robot moves, and upon completion, the simulation is torn down and restarts with the next waypoint.
- Verify the output directory structure matches the `sweep_YYYYMMDD_HHMMSS` design.
- Verify the bag files contain the necessary reproducible topics (and omit large camera data).
